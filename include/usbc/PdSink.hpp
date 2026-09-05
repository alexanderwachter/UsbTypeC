/*
 * USB PD sink port: the complete single-role port behind one class.
 * Composes the Type-C sink connection layer with the sink policy
 * engine and wires the parts an integration must not miss: the attach
 * feeds the engine, the PD alert bits reach it, the hard-reset window
 * holds the attach while the source legitimately cycles VBUS through
 * vSafe0V (instead of detaching and renegotiating from scratch), and
 * an exhausted HardResetCounter escalates to Type-C Error Recovery.
 * Engine-tearing actions are deferred and run from pump() once the
 * reporting machine is idle, exactly like the DRP facade.
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
    static_assert(((!pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::pr_swap_feature> &&
                    !pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::dr_swap_feature> &&
                    !pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
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
        engine_.setIdleHook([](void* self) { static_cast<PdSink*>(self)->pump(); }, this);
    }

    // Go live: present Rd and negotiate when a source attaches
    void start() { sink_.start(); }

private:
    // What a completed engine step asks the Type-C layer to do; both
    // actions tear the engine down or suspend its inputs, so they
    // never run inside the reporting machine's process() - pump()
    // executes them once it is idle
    enum class pending_action : std::uint8_t {
        none,
        error_recovery,    // nHardResetCount exhausted
        hard_reset_window, // hold the attach while VBUS cycles
    };

    // Injected into the engine's machine: watches the states'
    // portReport() observations (the swap and VCONN observations
    // cannot occur - their states are compiled out)
    struct port_watch : fsm::observing<port_watch> {
        explicit port_watch(PdSink& port_ref) : port(port_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.portReport()))
        {
            return state.portReport();
        }
        void notifyEntry(pe::request_error_recovery)
        {
            port.pending_ = pending_action::error_recovery;
        }
        void notifyEntry(pe::hard_reset_window)
        {
            port.pending_ = pending_action::hard_reset_window;
        }

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

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onEnterState(MACHINE& machine)
        {
            if constexpr (std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                // seed the collision-avoidance view of the source's Rp
                port.engine_.sinkTxChanged(
                    tc::sinkTxOk(machine.template getIf<NEW_STATE>()->context.cc));
                active = true;
                port.engine_.vbusPresent(); // fresh attach, or VBUS back
            }
        }

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onExitState(MACHINE&)
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

        void onPdAlert(alert_status alerts)
        {
            port.engine_.onAlert(alerts);
            port.pump(); // actions recorded while routing run now
        }

    private:
        void detach()
        {
            active = false;
            port.engine_.vbusRemoved();
        }
    };

    // Executes the recorded Type-C action; called only when the
    // engine's machine is idle (after alert routing, and from the
    // engine's timer hook)
    void pump()
    {
        auto const action = pending_;
        pending_          = pending_action::none;
        switch (action) {
        case pending_action::error_recovery:
            // terminations removed for tErrorRecovery, resolution
            // restarts; the teardown resets the engine
            sink_.errorRecovery();
            break;
        case pending_action::hard_reset_window:
            // the attach is held while VBUS legitimately cycles; the
            // engine stays live to await the capabilities
            sink_.hardResetWindow();
            break;
        case pending_action::none: break;
        }
    }

    using Sink = TypeCSink<TCPC, VBUS, TIMER, router, OBSERVERs...>;

    port_watch watch_;
    SinkEngine engine_;
    router router_;
    Sink sink_;
    pending_action pending_ = pending_action::none;
};

} // namespace usbc
