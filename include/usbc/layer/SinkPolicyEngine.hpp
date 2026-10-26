/*
 * USB PD sink policy engine (PE_SNK) on top of the protocol layer: the
 * engine running the sink table (state/SinkPolicyEngine.hpp).
 *
 * Integration: the engine is the client of the port's protocol layer
 * while live - its layer port, an observer of the engine's machine,
 * executes the states' prl::reset_action annotations and transmits the
 * pd_message a state publishes through values() - and runs from
 * construction on, resting in PE_SNK_Discovery until the attach. The
 * engine observes the connection machine itself: Attached.SNK's
 * attach fact (tc::attached_as_sink: a fresh attach or VBUS back
 * after the hard-reset window goes live, a completed power role swap
 * continues; the source's Rp seeds PD3 collision avoidance), the data
 * role the connection holds, a DRP's swap standby toward the sink
 * role (this engine takes the layer over mid-swap), Attached.SRC
 * after a completed swap (this engine retires) and the PD
 * connection's exit (the detach). The application injects its own
 * observers into the machine; the power side is one of them: derive
 * from SinkPower (interface/Power.hpp) and implement setLimit/
 * onContract/onContractLost. Extra observers (e.g. for logging) see
 * exactly the annotated edges of the DOT diagram. After a hard reset
 * the engine waits in Discovery for the connection layer to report
 * the returning VBUS. Everything runs in the stack's serialized
 * context; observer callbacks may originate from the timer context
 * (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Policy.hpp>
#include <usbc/layer/PolicyEngine.hpp>
#include <usbc/layer/ProtocolLayer.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Pdo.hpp>
#include <usbc/spec/Units.hpp>
#include <usbc/state/PolicyEngine.hpp>
#include <usbc/state/SinkPolicyEngine.hpp>

#include <mtl/StateMachine.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace usbc {

template<
    concepts::protocol_layer PRL,
    fsm::concepts::timer TIMER,
    concepts::sink_policy POLICY,
    typename... OBSERVERs>
class SinkPolicyEngine : public pe::PolicyEngineBase<
                             SinkPolicyEngine<PRL, TIMER, POLICY, OBSERVERs...>,
                             power_role::sink,
                             pe::pe_sink_table,
                             PRL,
                             TIMER,
                             POLICY,
                             OBSERVERs...>,
                         public fsm::observing<SinkPolicyEngine<PRL, TIMER, POLICY, OBSERVERs...>> {
    using base = pe::PolicyEngineBase<
        SinkPolicyEngine,
        power_role::sink,
        pe::pe_sink_table,
        PRL,
        TIMER,
        POLICY,
        OBSERVERs...>;
    friend base;

public:
    // The observers are injected into the engine's machine after the
    // layer port; a SinkPower-derived one supplies the power side.
    // pe_timer runs the engine's states, swap_timer the sub-states of a
    // swap exchange (never armed without a swap feature)
    SinkPolicyEngine(
        PRL& prl,
        TIMER& pe_timer,
        TIMER& swap_timer,
        std::span<sink_capability const> capabilities,
        POLICY& policy,
        OBSERVERs&... observers
    )
        : base(prl, pe_timer, swap_timer), policy_(policy),
          sm_(this->timed_, this->port_, gates_, policy_, observers...)
    {
        this->sink_capabilities_ = capabilities;
        sm_.process(pe::event::started{}); // rest in Discovery until VBUS
    }

    // --- the connection machine's facts ---------------------------------------

    // the data role the connection holds, ahead of the attach (set
    // order): the header carries it from the first message on
    void notifyEntry(tc::attached_data_role attached) { data_role_ = attached.role; }

    // Attached.SNK entered: a fresh attach or VBUS back after the
    // hard-reset window starts the negotiation, the completion of a
    // power role swap continues it (this engine is live mid-swap); the
    // source's Rp seeds the PD3 collision-avoidance view, CC reports
    // keep it current from here
    void notifyEntry(tc::attached_as_sink const& attach)
    {
        sinkTxChanged(tc::sinkTxOk(attach.cc));
        if (attach.origin == tc::attach_origin::completed_power_role_swap) {
            finishSwap();
        } else {
            vbusPresent();
        }
    }

    // Attached.SRC entered completing a power role swap: this engine
    // ran the swap and retires, the source engine continues the
    // connection on the layer
    void notifyEntry(tc::attached_as_source const& attach)
    {
        if (attach.origin == tc::attach_origin::completed_power_role_swap) {
            vbusRemoved();
        }
    }

    // The old source asserted Rd mid PR_Swap: this engine takes the
    // layer over for the PS_RDY exchange (the standby toward the
    // source role leaves this engine live)
    void notifyEntry(tc::swap_standby standby)
    {
        if (standby.to == power_role::sink) {
            startSwapWaitSourceOn();
        }
    }

    // the one real detach; the engine that is live retires
    void notifyExit(tc::pd_connection)
    {
        if (this->isLive()) {
            vbusRemoved();
        }
    }

    // PD3 collision avoidance: the connection layer forwards every CC
    // report, the source's Rp reads as SinkTxOk (3.0 A) / SinkTxNG
    void onCcStatus(cc_status cc) { sinkTxChanged(tc::sinkTxOk(cc)); }

    // --- the same, as calls ----------------------------------------------------

    // Attach: for a sink, VBUS is present; the engine goes live as the
    // layer's client
    void vbusPresent()
    {
        this->takeLayer(data_role_);
        sm_.process(pe::event::vbus_present{});
    }

    // ... and detach: negotiation state is gone, back to Discovery
    void vbusRemoved()
    {
        this->leaveLayer();
        this->setBistTestData(false); // the test mode ends with the partner
        pending_sequence_.reset();
        sm_.process(pe::event::vbus_removed{});
        sm_.process(pe::event::started{});
    }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there). Under PD3
    // collision avoidance a request during SinkTxNG parks and fires
    // when the source's Rp says SinkTxOk
    bool requestPowerSwap()
    {
        if constexpr (!base::pr_swap_capable) {
            return false; // the feature is compiled out
        } else {
            return request(pe::atomic_message_sequence::power_role_swap);
        }
    }

    bool requestDataSwap()
    {
        if constexpr (!base::dr_swap_capable) {
            return false; // the feature is compiled out
        } else {
            return request(pe::atomic_message_sequence::data_role_swap);
        }
    }

    // PD3 collision avoidance: the source's Rp signals whether the
    // sink may initiate an AMS (SinkTxOk = 3.0 A, SinkTxNG = 1.5 A).
    // The port layer feeds every CC report; parked and gated requests
    // fire on the flip to Ok
    void sinkTxChanged(bool ok)
    {
        sink_tx_ok_ = ok;
        if (ok) {
            fireGated();
            firePending();
        }
    }

    // The port was the source and asserted Rd mid PR_Swap: announce
    // the supply is off and await the new source's PS_RDY. The event
    // carries the preserved data role (the entered state seeds the
    // context with it); the negotiated revision and the MessageID
    // lifecycle hold for the connection - a swap is no reset trigger
    // (6.7.1) - on the one layer both engines share
    void startSwapWaitSourceOn()
    {
        this->takeLayer(data_role_);
        sm_.process(pe::event::swap_wait_source_on{data_role_});
    }

    // A DRP announces its source-role capabilities: Get_Source_Cap is
    // answered with them instead of Not_Supported
    void provideSourceCapabilities(std::span<std::uint32_t const> capabilities)
    {
        this->source_capabilities_ = capabilities;
    }

    // The swap completed into Attached.SNK: resume the sink flow (the
    // new source's PS_RDY implies VBUS is live)
    void finishSwap()
    {
        sm_.process(pe::event::swap_done{});
        sm_.process(pe::event::vbus_present{});
    }

private:
    // A retry whose gate opens while SinkTxOk already holds fires
    // right away: the retry_gated annotation marks the gate states,
    // and the queued machine delivers the retry after the gate's
    // entry completes (the parked case fires from sinkTxChanged)
    struct gate_watch : fsm::observing<gate_watch> {
        explicit gate_watch(SinkPolicyEngine& engine_ref) : engine(engine_ref) {}

        void notifyEntry(pe::retry_gated)
        {
            if (engine.sinkTxAllows()) {
                engine.fireGated();
            }
        }

        SinkPolicyEngine& engine;
    };

    // --- the base's hooks ---------------------------------------------------

    // The sink's own messages: Source_Capabilities to evaluate, and
    // Get_Source_Cap answered by a DRP with its source-role list
    bool dispatchRole(pd_header const& header, pd_message const& message)
    {
        if (isData(header, data_message_type::source_capabilities)) {
            evaluate(message, header.num_data_objects);
            return true;
        }
        if (isControl(header, control_message_type::get_source_cap)) {
            sendSourceCapabilities();
            return true;
        }
        return false;
    }

    // the contract's voltage while an explicit contract holds (BIST is
    // honored at vSafe5V only)
    std::optional<millivolt> contractVoltage() const
    {
        auto const& negotiation = sm_.template context<pe::pe_negotiation>();
        if (!negotiation.explicit_contract) {
            return std::nullopt;
        }
        return negotiation.request.voltage;
    }

    // Advances the transient spec states after a hard reset:
    // Transition_to_default -> Startup -> Discovery, where the engine
    // waits for the Type-C layer to report the returning VBUS
    void afterHardReset()
    {
        sm_.process(pe::event::default_level_reached{});
        sm_.process(pe::event::started{});
    }

    // --- collision avoidance --------------------------------------------------

    // A swap request from Ready: sent now, or parked until SinkTxOk
    bool request(pe::atomic_message_sequence sequence)
    {
        if (!sm_.template annotation<pe::ready_for_atomic_message_sequence>()) {
            return false;
        }
        if (!sinkTxAllows()) {
            pending_sequence_ = sequence;
            return true;
        }
        initiate(sequence);
        return true;
    }

    // Collision avoidance applies under PD3 with an explicit contract;
    // otherwise the sink initiates freely
    bool sinkTxAllows() const
    {
        if (this->prl_.revision() != pd_revision::rev_3_x) {
            return true;
        }
        return !sm_.template context<pe::pe_negotiation>().explicit_contract || sink_tx_ok_;
    }

    // A retry that timed out under SinkTxNG waits in its gate state,
    // whose annotation says which sequence (a gate of a compiled-out
    // feature never exists, its event is refused at compile time)
    void fireGated()
    {
        if (auto const gated = sm_.template annotation<pe::retry_gated>()) {
            initiate(gated->sequence);
        }
    }

    // A swap request parked under SinkTxNG fires once Ready under Ok
    void firePending()
    {
        auto const pending = pending_sequence_;
        pending_sequence_.reset();
        if (pending && sm_.template annotation<pe::ready_for_atomic_message_sequence>()) {
            initiate(*pending);
        }
    }

    void initiate(pe::atomic_message_sequence sequence)
    {
        switch (sequence) {
        case pe::atomic_message_sequence::request: sm_.process(pe::event::request_retry{}); break;
        case pe::atomic_message_sequence::power_role_swap:
            sm_.process(pe::event::send_pr_swap{});
            break;
        case pe::atomic_message_sequence::data_role_swap:
            sm_.process(pe::event::send_dr_swap{});
            break;
        }
    }

    // --- the sink's messages ------------------------------------------------

    // PE_SNK_Evaluate_Capability: ask the policy, fall back to the
    // first PDO with the Capability Mismatch flag
    void evaluate(pd_message const& message, std::uint8_t count)
    {
        if (!sm_.process(pe::event::source_capabilities{})) {
            return; // not in a state that evaluates capabilities
        }
        std::array<std::uint32_t, 7> objects{};
        auto const n = std::min<std::uint8_t>(count, objects.size());
        for (std::uint8_t index = 0; index < n; ++index) {
            objects[index] = dataObjectAt(message, index);
        }
        auto const offered = std::span<std::uint32_t const>{objects.data(), n};

        auto terms = policy_.select(offered, this->sink_capabilities_);
        if (!terms && n > 0) {
            terms = contract_request{
                .position = 1,
                .voltage = pdo::fixedVoltage(objects[0]),
                .operating_current = pdo::fixedMaxCurrent(objects[0]),
                .maximum_current = pdo::fixedMaxCurrent(objects[0]),
                .mismatch = true
            };
        }
        if (terms) {
            sm_.process(pe::event::capabilities_evaluated{.terms = *terms});
        }
    }

    // PE_DR_SNK_Give_Source_Cap; a sink-only port answers Not_Supported
    void sendSourceCapabilities()
    {
        if (this->source_capabilities_.empty()) {
            sm_.process(pe::event::unsupported{});
            return;
        }
        sm_.process(
            pe::event::send_source_caps{pe::makeSourceCapabilitiesMessage(
                power_role::sink,
                this->dataRole(),
                this->source_capabilities_
            )}
        );
    }

    POLICY& policy_;
    data_role data_role_ = data_role::ufp; // the connection's, as the attach delivers it
    bool sink_tx_ok_ = true;               // last Rp seen (SinkTxOk/NG)
    std::optional<pe::atomic_message_sequence> pending_sequence_; // parked under SinkTxNG
    gate_watch gates_{*this};
    // the policy rides in the pack to answer the table's questions
    fsm::QueuedMachine<
        pe::pe_sink_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        typename base::timeouts,
        typename base::PrlPort,
        gate_watch,
        POLICY,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
