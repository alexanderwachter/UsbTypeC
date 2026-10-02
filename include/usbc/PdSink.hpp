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
#include <mtl/Typelist.hpp>

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

template<
    concepts::tcpc TCPC,
    concepts::vbus VBUS,
    fsm::concepts::timer TIMER,
    concepts::sink_policy POLICY,
    typename POWER,
    typename... OBSERVERs>
class PdSink {
    // The single-role facade wires none of the tag-enabled features:
    // an enabling observer here would be silently ignored
    static_assert(
        ((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::pr_swap_feature> &&
          !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::dr_swap_feature> &&
          !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::vconn_feature>) &&
         ...),
        "PdSink does not wire the optional features (swaps, VCONN); "
        "a port offering them is a PdDrp"
    );

public:
    PdSink(
        TCPC& tcpc,
        VBUS& vbus,
        pd_sink_timers<TIMER>& timers,
        std::span<sink_capability const> capabilities,
        POLICY& policy,
        POWER& power,
        OBSERVERs&... observers
    )
        : watch_{*this}, engine_(tcpc, timers.prl, timers.pe, capabilities, policy, power, watch_),
          router_{*this}, sink_(tcpc, vbus, timers.tc, router_, observers...)
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

    // The port's internal wiring in the connection machine, observing
    // the states' declared facts: the PD connection's exit is the one
    // real detach (its tag spans the hard-reset window, so the
    // legitimate VBUS cycle is none), every entry of Attached.SNK -
    // fresh, or VBUS back after the window - reaches the engine as the
    // partner value, and the CC status (PD3 collision avoidance) and
    // the PD alerts are forwarded from the driver frontend
    struct router : fsm::observing<router> {
        explicit router(PdSink& port_ref) : port(port_ref) {}

        using observes = fsm::annotations<tc::pd_connection, tc::attached_partner>;

        // the one real detach: the window timing out included
        void notifyExit(tc::pd_connection) { port.engine_.vbusRemoved(); }

        // Attached.SNK entered: seed the collision-avoidance view of
        // the source's Rp, then the engine learns VBUS is present
        void notifyEntry(tc::attached_partner const& partner)
        {
            port.engine_.sinkTxChanged(tc::sinkTxOk(partner.cc));
            port.engine_.vbusPresent();
        }

        // the engine's view of the source's Rp; it acts on it only
        // through its Ready-state annotations, so no attach gate here
        void onCcStatus(cc_status cc) { port.engine_.sinkTxChanged(tc::sinkTxOk(cc)); }

        void onPdAlert(alert_status alerts) { port.engine_.onAlert(alerts); }

        PdSink& port;
    };

    using Sink = TypeCSink<TCPC, VBUS, TIMER, router, OBSERVERs...>;

    port_watch watch_;
    SinkEngine engine_;
    router router_;
    Sink sink_;
};

} // namespace usbc
