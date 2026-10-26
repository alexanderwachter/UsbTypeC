/*
 * The engine base carrying what both USB PD policy engines run alike
 * around their tables (state/PolicyEngine.hpp and the role's table):
 * the port on the port's protocol layer, the message dispatch, BIST,
 * the capability answers. The sink and source engines live in
 * SinkPolicyEngine.hpp and SourcePolicyEngine.hpp next to this file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/ProtocolLayer.hpp>
#include <usbc/interface/PdPhy.hpp>
#include <usbc/interface/Policy.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/layer/ProtocolLayer.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Pdo.hpp>
#include <usbc/state/PolicyEngine.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <span>
#include <type_traits>

namespace usbc {

namespace pe {

// What both engine facades run alike, behind the role-specific
// SinkPolicyEngine / SourcePolicyEngine (CRTP): the port on the
// port's protocol layer (the engine is the layer's client while
// live), the message dispatch with the role's own messages left to
// the derived engine (dispatchRole), the BIST entry, the Get_Sink_Cap
// answer, the swap and VCONN requests, and the machine's timers - one
// per level: the engine's states, and the sub-states of the swap
// exchanges where the policy brings a swap feature in. The derived
// engine owns the machine (its observers are the role's) and
// befriends this base to let it drive the machine
template<
    typename DERIVED,
    power_role ROLE,
    fsm::concepts::transition_table TABLE,
    concepts::protocol_layer PRL,
    fsm::concepts::timer TIMER,
    typename POLICY,
    typename... OBSERVERs>
class PolicyEngineBase {
public:
    using protocol_layer = PRL;

    // VCONN_Swap: request the swap (the application's decision),
    // transmit our PS_RDY once the switch is on, and escalate a failed
    // hand-off - the last two are the vconn machine's directions, the
    // port executes them on the live engine
    bool requestVconnSwap() { return derived().sm_.process(event::send_vconn_swap{}); }
    bool sendVconnPsRdy() { return derived().sm_.process(event::send_vconn_ps_rdy{}); }
    bool hardReset() { return derived().sm_.process(event::hard_reset_request{}); }

    // Live: the engine is the protocol layer's client - from its
    // role's attach to its retirement
    bool isLive() const { return prl_.clientIs(port_); }

protected:
    // The optional features follow the injected objects: the machine
    // keeps a feature's states exactly when the policy or an injected
    // observer answers the feature's question; the engine's own
    // branches ask the library the same thing
    static constexpr bool pr_swap_capable =
        fsm::feature_enabled_v<pr_swap_feature, POLICY, std::remove_cvref_t<OBSERVERs>...>;
    static constexpr bool dr_swap_capable =
        fsm::feature_enabled_v<dr_swap_feature, POLICY, std::remove_cvref_t<OBSERVERs>...>;
    static constexpr bool vconn_capable =
        fsm::feature_enabled_v<vconn_feature, POLICY, std::remove_cvref_t<OBSERVERs>...>;

    // The machine's levels: the swap exchanges are composite states,
    // the second level is theirs (its timer idles while the policy
    // brings no swap feature in)
    using queued_timer = fsm::QueuedTimer<TIMER>;
    using timeouts = fsm::timed<queued_timer&, fsm::levels_v<TABLE>>;

    // swap_timer runs the timeouts inside a swap exchange
    PolicyEngineBase(protocol_layer& prl, TIMER& pe_timer, TIMER& swap_timer)
        : prl_(prl), pe_timer_(pe_timer), swap_timer_(swap_timer), timed_(pe_timer_, swap_timer_)
    {
    }

    // Going live: the layer reports to this engine and stamps its
    // roles into the header, the data role as the connection holds it
    void takeLayer(data_role data_role) { prl_.setClient(port_, ROLE, data_role); }
    void leaveLayer() { prl_.clearClient(port_); }

    // The partner's swap request is the table's question (the Ready
    // rows on the *_swap_received events): the injected policy's
    // answer accepts, the catch-all refuses - Reject where the
    // feature exists, the non-DRP Not_Supported where it does not. A
    // request outside Ready is discarded (another Atomic Message
    // Sequence is running)
    template<bool CAPABLE>
    static constexpr control_message_type refusal =
        CAPABLE ? control_message_type::reject : control_message_type::not_supported;

    DERIVED& derived() { return static_cast<DERIVED&>(*this); }
    DERIVED const& derived() const { return static_cast<DERIVED const&>(*this); }

    data_role dataRole() const { return derived().sm_.template context<pe_connection>().data; }

    // The engine's port on the protocol layer: the layer's client,
    // forwarding its reports into the engine, and the observer of the
    // engine's machine executing the states' layer actions - for the
    // layer's client only, so a retiring engine's Startup does not
    // reset the connection's MessageIDs the relieving engine continues
    // (a PR_Swap is no reset trigger, 6.7.1). The observing contract
    // delivers the static set before the instance values, so a state
    // carrying both resets first and its message goes out with
    // MessageID 0
    struct PrlPort : fsm::observing<PrlPort> {
        explicit PrlPort(PolicyEngineBase& engine_ref) : engine(engine_ref) {}

        PolicyEngineBase& engine;

        void notifyEntry(prl::reset_action)
        {
            if (engine.isLive()) {
                engine.prl_.reset(sop_type::sop);
            }
        }
        void notifyEntry(prl::hard_reset_action)
        {
            if (engine.isLive()) {
                engine.prl_.transmitHardReset();
            }
        }
        void notifyEntry(pd_message const& message)
        {
            if (engine.isLive()) {
                engine.prl_.transmit(message);
            }
        }
        // an agreed DR_Swap: the header follows
        void notifyEntry(data_role_changed changed)
        {
            if (engine.isLive()) {
                engine.prl_.setDataRole(changed.role);
            }
        }

        void onMessage(pd_message const& message) { engine.dispatch(message); }
        void onTxDone() { engine.derived().sm_.process(event::message_sent{}); }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { engine.derived().sm_.process(event::protocol_error{}); }
        void onHardReset()
        {
            engine.setBistTestData(false); // a hard reset ends the test mode
            engine.derived().sm_.process(event::hard_reset_received{});
            engine.derived().afterHardReset();
        }
        void onHardResetSent()
        {
            engine.setBistTestData(false);
            engine.derived().sm_.process(event::hard_reset_complete{});
            engine.derived().afterHardReset();
        }
    };

    // Hook: what follows a hard reset in the machine, if anything (the
    // sink pumps its transients, the source's supply drives them)
    void afterHardReset() {}

    // A received message becomes the engine's event: the role's own
    // messages (capabilities, Request, Get_Source_Cap) go to the
    // derived engine first, everything else is common to both roles
    void dispatch(pd_message const& message)
    {
        if (bist_test_data_) {
            return; // BIST test data mode: deaf until hard reset/detach
        }
        auto const header = pd_header::decode(message.header);
        if (header.extended) {
            auto const extended = extended_header::decode(
                static_cast<std::uint16_t>(message.payload[0]) |
                (static_cast<std::uint16_t>(message.payload[1]) << 8u)
            );
            if (extended.chunked) {
                derived().sm_.process(event::chunked_message{});
            } else {
                derived().sm_.process(event::unsupported{});
            }
            return;
        }
        if (derived().dispatchRole(header, message)) {
            return;
        }
        if (header.num_data_objects > 0) {
            if (header.message_type == static_cast<std::uint8_t>(data_message_type::bist)) {
                enterBist(dataObjectAt(message, 0));
            } else {
                derived().sm_.process(event::unsupported{});
            }
            return;
        }
        switch (static_cast<control_message_type>(header.message_type)) {
        case control_message_type::accept: derived().sm_.process(event::accept{}); break;
        case control_message_type::reject: derived().sm_.process(event::reject{}); break;
        case control_message_type::wait: derived().sm_.process(event::wait{}); break;
        case control_message_type::ps_rdy: derived().sm_.process(event::ps_rdy{}); break;
        case control_message_type::dr_swap:
            derived().sm_.process(
                event::dr_swap_received{
                    .refusal = refusal<dr_swap_capable>,
                    .to_take = otherDataRole(dataRole()),
                }
            );
            break;
        case control_message_type::pr_swap:
            derived().sm_.process(
                event::pr_swap_received{
                    .refusal = refusal<pr_swap_capable>,
                    .to_take = otherPowerRole(ROLE),
                }
            );
            break;
        case control_message_type::vconn_swap:
            derived().sm_.process(event::vconn_swap_received{.refusal = refusal<vconn_capable>});
            break;
        case control_message_type::get_sink_cap: sendSinkCapabilities(); break;
        case control_message_type::soft_reset:
            derived().sm_.process(event::soft_reset_received{});
            break;
        case control_message_type::good_crc:
        case control_message_type::ping: break;
        default:
            // answered from Ready only; ignored while negotiating
            derived().sm_.process(event::unsupported{});
        }
    }

    // Get_Sink_Cap: the sink-role capabilities, which a sink has and a
    // DRP sourcing provides - a source-only port answers Not_Supported
    void sendSinkCapabilities()
    {
        if (sink_capabilities_.empty()) {
            derived().sm_.process(event::unsupported{});
            return;
        }
        derived().sm_.process(
            event::send_sink_capabilities{
                makeSinkCapabilitiesMessage(ROLE, dataRole(), sink_capabilities_)
            }
        );
    }

    // BIST entry, honored only under an explicit vSafe5V contract
    // (spec): Carrier Mode 2 transmits the test carrier for
    // tBISTContMode; Test Data silences the engine until a hard reset
    // or detach while the PHY keeps answering GoodCRC
    void enterBist(std::uint32_t bdo)
    {
        auto const contract_voltage = derived().contractVoltage();
        if (contract_voltage != v_safe_5v) {
            return;
        }
        switch (bist::modeOf(bdo)) {
        case bist::mode::carrier_mode_2:
            if (derived().sm_.process(event::bist_carrier{})) {
                prl_.transmit(transmit_signal::bist_carrier_mode_2);
            }
            break;
        case bist::mode::test_data:
            if (derived().sm_.template annotation<ready_for_atomic_message_sequence>()) {
                setBistTestData(true);
            }
            break;
        default: break; // other modes are not supported
        }
    }

    void setBistTestData(bool enable)
    {
        bist_test_data_ = enable;
        prl_.setBistTestData(enable); // hardware may discard for us
    }

    // the capability lists: the own role's from construction, the other
    // role's provided by a DRP (empty: not a DRP, Get_*_Cap answers
    // Not_Supported)
    std::span<sink_capability const> sink_capabilities_{};
    std::span<std::uint32_t const> source_capabilities_{};
    bool bist_test_data_ = false;
    PrlPort port_{*this};
    // the port's one protocol layer, shared by a DRP's two engines
    protocol_layer& prl_;
    queued_timer pe_timer_;
    queued_timer swap_timer_;
    timeouts timed_;
};

} // namespace pe

} // namespace usbc
