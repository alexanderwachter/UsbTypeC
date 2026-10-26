/*
 * USB PD source policy engine (PE_SRC) on top of the protocol layer:
 * the engine running the source table (state/SourcePolicyEngine.hpp).
 *
 * Integration: mirror image of the sink engine. The engine is the
 * client of the port's protocol layer while live - its layer port, an
 * observer of the machine, executes the states' prl::reset_action
 * annotations and transmits the pd_message they publish through
 * values() - and observes the connection machine itself:
 * Attached.SRC's attach fact (tc::attached_as_source: a fresh attach
 * advertises, a completed power role swap continues the connection as
 * the new source), the data role the connection holds, a DRP's swap
 * standby toward the sink role (this engine retires, the sink engine
 * takes the layer over) and the PD connection's exit (the detach).
 * The application injects its own observers - derive from SourcePower
 * (interface/Power.hpp) and implement onContract/onContractLost. The
 * supply derives from SourceSupply (interface/Power.hpp, implement
 * setOutput): injected into the machine, it applies the operating
 * points the states annotate, and the engine feeds its settled
 * callback back as an event. Everything runs in the stack's
 * serialized context; observer callbacks may originate from the timer
 * context (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Policy.hpp>
#include <usbc/interface/Power.hpp>
#include <usbc/layer/PolicyEngine.hpp>
#include <usbc/layer/ProtocolLayer.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Pdo.hpp>
#include <usbc/spec/Units.hpp>
#include <usbc/state/PolicyEngine.hpp>
#include <usbc/state/SourcePolicyEngine.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <optional>
#include <span>

namespace usbc {

template<
    concepts::cc CC,
    concepts::protocol_layer PRL,
    fsm::concepts::timer TIMER,
    concepts::source_policy POLICY,
    concepts::source_supply SUPPLY,
    typename... OBSERVERs>
class SourcePolicyEngine
    : public pe::PolicyEngineBase<
          SourcePolicyEngine<CC, PRL, TIMER, POLICY, SUPPLY, OBSERVERs...>,
          power_role::source,
          pe::pe_source_table,
          PRL,
          TIMER,
          POLICY,
          OBSERVERs...>,
      public fsm::observing<SourcePolicyEngine<CC, PRL, TIMER, POLICY, SUPPLY, OBSERVERs...>> {
    using base = pe::PolicyEngineBase<
        SourcePolicyEngine,
        power_role::source,
        pe::pe_source_table,
        PRL,
        TIMER,
        POLICY,
        OBSERVERs...>;
    friend base;

public:
    // The observers are injected into the engine's machine after the
    // layer port and the supply driver; a SourcePower-derived one
    // supplies the contract notifications. The CC driver presents the
    // PD3 collision-avoidance Rp (SinkTxOk/SinkTxNG) under an explicit
    // contract and the port's advertisement otherwise. pe_timer runs
    // the engine's states, swap_timer the sub-states of a swap exchange
    // (never armed without a swap feature)
    SourcePolicyEngine(
        CC& cc,
        PRL& prl,
        TIMER& pe_timer,
        TIMER& swap_timer,
        std::span<std::uint32_t const> capabilities,
        rp_value advertisement,
        POLICY& policy,
        SUPPLY& supply,
        OBSERVERs&... observers
    )
        : base(prl, pe_timer, swap_timer), cc_(cc), advertisement_(advertisement), policy_(policy),
          supply_(supply),
          sm_(this->timed_, this->port_, action_driver_, supply_, policy_, observers...)
    {
        this->source_capabilities_ = capabilities;
        supply_.setReadyCallback(
            [](void* self, bool at_target) {
                auto& engine = *static_cast<SourcePolicyEngine*>(self);
                if (at_target) {
                    engine.sm_.process(pe::event::supply_settled{});
                }
            },
            this
        );
    }

    // --- the connection machine's facts ---------------------------------------

    // the data role the connection holds, ahead of the attach (set
    // order): the header carries it from the first message on
    void notifyEntry(tc::attached_data_role attached) { data_role_ = attached.role; }

    // Attached.SRC entered: a fresh attach advertises, the completion
    // of a power role swap continues the connection as the new source
    // (the sink engine retired on the same fact, in either order)
    void notifyEntry(tc::attached_as_source const& attach)
    {
        if (attach.origin == tc::attach_origin::completed_power_role_swap) {
            attachedAfterSwap();
        } else {
            attached();
        }
    }

    // The old source asserted Rd mid PR_Swap: this engine ran the swap
    // and retires, the sink engine takes the layer over
    void notifyEntry(tc::swap_standby standby)
    {
        if (standby.to == power_role::sink) {
            detached();
        }
    }

    // the one real detach; the engine that is live retires
    void notifyExit(tc::pd_connection)
    {
        if (this->isLive()) {
            detached();
        }
    }

    // --- the same, as calls ----------------------------------------------------

    // The attached sink: the engine goes live as the layer's client
    // and advertises
    void attached()
    {
        this->takeLayer(data_role_);
        sm_.process(pe::event::attached{});
    }

    // ... and the detach: everything resets, back to Startup
    void detached()
    {
        this->leaveLayer();
        this->setBistTestData(false); // the test mode ends with the partner
        sm_.process(pe::event::detached{});
    }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there). Under PD3
    // the source signals SinkTxNG and waits tSinkTx before the request
    // goes out (collision avoidance)
    bool requestPowerSwap()
    {
        if constexpr (!base::pr_swap_capable) { // the feature is compiled out
            return false;
        } else {
            if (this->prl_.revision() == pd_revision::rev_3_x) {
                return sm_.process(pe::event::begin_pr_swap{});
            }
            return sm_.process(pe::event::send_pr_swap{});
        }
    }

    bool requestDataSwap()
    {
        if constexpr (!base::dr_swap_capable) { // the feature is compiled out
            return false;
        } else {
            if (this->prl_.revision() == pd_revision::rev_3_x) {
                return sm_.process(pe::event::begin_dr_swap{});
            }
            return sm_.process(pe::event::send_dr_swap{});
        }
    }

    // The port was the sink and asserted Rp mid PR_Swap: drive VBUS to
    // vSafe5V, announce PS_RDY, pause tSwapSourceStart, then advertise.
    // The event carries the preserved data role (the entered state
    // seeds the context with it); the negotiated revision and the
    // MessageID lifecycle hold for the connection - a swap is no reset
    // trigger (6.7.1) - on the one layer both engines share
    void attachedAfterSwap()
    {
        this->takeLayer(data_role_);
        sm_.process(pe::event::attached_swap{data_role_});
    }

    // A DRP announces its sink-role capabilities: Get_Sink_Cap is
    // answered with them instead of Not_Supported
    void provideSinkCapabilities(std::span<sink_capability const> capabilities)
    {
        this->sink_capabilities_ = capabilities;
    }

private:
    // Executes the states' engine-directed annotations, delivered
    // from their sets by overload: the Source_Capabilities
    // transmission and the PD3 collision-avoidance Rp - only
    // meaningful under an explicit contract with a PD3 partner, and
    // the port's advertisement stands again once the contract is gone
    // (spec 5.2.2: the Rp specifies the Type-C current during the
    // default or implicit contract); the supply's operating points
    // reach the injected SourceSupply directly
    struct ActionDriver : fsm::observing<ActionDriver> {
        explicit ActionDriver(SourcePolicyEngine& engine_ref) : engine(engine_ref) {}

        void notifyEntry(pe::send_capabilities_action) { engine.transmitSourceCaps(); }
        void notifyEntry(pe::sink_tx tx)
        {
            if (engine.prl_.revision() != pd_revision::rev_3_x ||
                !engine.sm_.template context<pe::src_negotiation>().explicit_contract) {
                return;
            }
            engine.cc_.setRp(tx == pe::sink_tx::ok ? rp_value::p_3a0 : rp_value::p_1a5);
        }
        void notifyEntry(pe::power_level level)
        {
            if (level != pe::power_level::explicit_contract) {
                engine.cc_.setRp(engine.advertisement_);
            }
        }

        SourcePolicyEngine& engine;
    };

    // --- the base's hooks ---------------------------------------------------

    // The source's own messages: the Request to negotiate, and
    // Get_Source_Cap answered by re-advertising
    bool dispatchRole(pd_header const& header, pd_message const& message)
    {
        if (isData(header, data_message_type::request)) {
            negotiate(message);
            return true;
        }
        if (isControl(header, control_message_type::get_source_cap)) {
            sm_.process(pe::event::get_source_caps{});
            return true;
        }
        return false;
    }

    // the contract's voltage while an explicit contract holds (BIST is
    // honored at vSafe5V only)
    std::optional<millivolt> contractVoltage() const
    {
        auto const& negotiation = sm_.template context<pe::src_negotiation>();
        if (!negotiation.explicit_contract) {
            return std::nullopt;
        }
        return negotiation.target.voltage;
    }

    // --- the source's messages ----------------------------------------------

    void transmitSourceCaps()
    {
        this->prl_.transmit(
            pe::makeSourceCapabilitiesMessage(
                power_role::source,
                this->dataRole(),
                this->source_capabilities_
            )
        );
    }

    // PE_SRC_Negotiate_Capability: the policy's verdict advances the
    // machine with request_ok or request_bad
    void negotiate(pd_message const& message)
    {
        if (!sm_.process(pe::event::request{})) {
            return; // not in a state that takes a Request
        }
        auto const rdo = dataObjectAt(message, 0);
        if (auto const target = policy_.evaluate(rdo, this->source_capabilities_)) {
            sm_.process(pe::event::request_ok{.target = *target});
        } else {
            sm_.process(pe::event::request_bad{});
        }
    }

    CC& cc_;
    rp_value advertisement_;
    POLICY& policy_;
    SUPPLY& supply_;
    data_role data_role_ = data_role::dfp; // the connection's, as the attach delivers it
    ActionDriver action_driver_{*this};
    // the policy rides in the pack to answer the table's questions
    fsm::QueuedMachine<
        pe::pe_source_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        typename base::timeouts,
        typename base::PrlPort,
        ActionDriver,
        SUPPLY,
        POLICY,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
