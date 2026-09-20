/*
 * USB PD sink port: the complete single-role port behind one class.
 * Composes the Type-C sink connection layer with the sink policy
 * engine and wires the parts an integration must not miss: the attach
 * feeds the engine, the PD alert bits reach it, the hard-reset window
 * holds the attach while the source legitimately cycles VBUS through
 * vSafe0V (instead of detaching and renegotiating from scratch), and
 * an exhausted HardResetCounter escalates to Type-C Error Recovery.
 * Engine-tearing actions run straight from the observing hooks: every
 * machine is queued (fsm::QueuedMachine), so an action feeding back
 * into the reporting machine is delivered after the running
 * transition instead of re-entering it.
 *
 * The user provides the domain pieces only: the drivers (TCPC, VBUS),
 * the timers, the capabilities, the selection policy, and the power
 * effects (a SinkPower-derived class). Extra observers may be
 * injected into the connection machine. The optional features
 * (PR_Swap/DR_Swap/VCONN) stay compiled out - a sink-only port
 * answers the partner's swap requests with Not_Supported.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/TypeCSink.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <span>

namespace usbc {

// The timers the port runs on: the connection layer's, the protocol
// layer's, and the engine's. One bundle owned by the caller
template<fsm::concepts::timer TIMER>
struct pd_sink_timers {
    TIMER tc;
    TIMER prl;
    TIMER pe;
};

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         concepts::sink_policy POLICY, typename POWER, typename... OBSERVERs>
class PdSink {
    // The single-role facade wires none of the tag-enabled features:
    // an enabling observer here would be silently ignored
    static_assert(((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::pr_swap_feature> &&
                    !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::dr_swap_feature> &&
                    !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::vconn_feature>) &&
                   ...),
                  "PdSink does not wire the optional features (swaps, VCONN); "
                  "a port offering them is a PdDrp");

public:
    PdSink(TCPC& tcpc, VBUS& vbus, pd_sink_timers<TIMER>& timers,
           std::span<sink_capability const> capabilities, POLICY& policy, POWER& power,
           OBSERVERs&... observers)
        : watch_{*this},
          engine_(tcpc, timers.prl, timers.pe, capabilities, policy, power, watch_),
          router_{*this},
          sink_(tcpc, vbus, timers.tc, router_, observers...)
    {
    }

    // Go live: present Rd and negotiate when a source attaches
    void start() { sink_.start(); }

private:
    // Injected into the engine's machine: watches the states' port
    // requests (the swap and VCONN observations cannot occur - their
    // states are compiled out). Both actions tear the engine down or
    // suspend its inputs; the queued machines order the fallout
    struct port_watch : fsm::observing<port_watch> {
        explicit port_watch(PdSink& port_ref) : port(port_ref) {}

        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engine
        void notifyEntry(pe::request_error_recovery) { port.sink_.errorRecovery(); }
        // the attach is held while VBUS legitimately cycles; the
        // engine stays live to await the capabilities
        void notifyEntry(pe::hard_reset_window) { port.sink_.hardResetWindow(); }

        PdSink& port;
    };

    using SinkEngine = SinkPolicyEngine<TCPC, TIMER, POLICY, POWER, port_watch>;

    // The port's internal wiring in the connection machine: feeds the
    // engine the attach, the detach, the CC status (PD3 collision
    // avoidance), and the PD alerts - and keeps it live through the
    // hard-reset window's legitimate VBUS cycle
    struct router {
        PdSink& port;
        bool active = false;

        // the entry needs the state entered only: one body per state
        template<typename STATE, typename MACHINE>
        void onEnter(MACHINE& machine)
        {
            if constexpr (std::is_same_v<STATE, tc::state::attached_snk>) {
                // seed the collision-avoidance view of the source's Rp
                port.engine_.sinkTxChanged(
                    tc::sinkTxOk(machine.template getIf<STATE>()->context.cc));
                active = true;
                port.engine_.vbusPresent(); // fresh attach, or VBUS back
            }
        }

        // the exit depends on where the machine goes: the edge form
        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onExitFrom(MACHINE&)
        {
            // the hard-reset window keeps the engine live: it still
            // awaits the source's capabilities; every other exit of
            // the attached group is a real detach
            if constexpr (std::is_same_v<OLD_STATE, tc::state::attached_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::hard_reset_snk>) {
                    detach();
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::hard_reset_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::hard_reset_recover_snk>) {
                    detach(); // window timed out: dead port
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::hard_reset_recover_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                    detach(); // window timed out: dead port
                }
            }
        }

        void onCcStatus(cc_status cc)
        {
            if (active) {
                port.engine_.sinkTxChanged(tc::sinkTxOk(cc));
            }
        }

        void onPdAlert(alert_status alerts) { port.engine_.onAlert(alerts); }

    private:
        void detach()
        {
            active = false;
            port.engine_.vbusRemoved();
        }
    };

    using Sink = TypeCSink<TCPC, VBUS, TIMER, router, OBSERVERs...>;

    port_watch watch_;
    SinkEngine engine_;
    router router_;
    Sink sink_;
};

} // namespace usbc
