/*
 * USB PD sink port: the complete single-role port behind one class.
 * Composes the Type-C sink connection layer, the port's protocol layer
 * and the sink policy engine: the layer and the engine observe the
 * connection machine (the attach goes live, the hard-reset window
 * holds the attach while the source legitimately cycles VBUS through
 * vSafe0V instead of detaching and renegotiating from scratch, the
 * connection's exit is the detach), and the engine's directions to the
 * connection layer - the hard-reset window, Error Recovery after an
 * exhausted HardResetCounter - are executed on it from the engine's
 * machine. Every machine is queued (fsm::QueuedMachine), so an action
 * feeding back into the reporting machine is delivered after the
 * running transition instead of re-entering it.
 *
 * The user provides the domain pieces only: the hardware blocks (CC,
 * PD PHY, VBUS), the timers, the capabilities, the selection policy,
 * and the power effects (a SinkPower-derived class). Extra observers
 * are handed to every machine of the port - connection, engine,
 * protocol layer - so a tracer sees all of them. The optional features
 * (PR_Swap/DR_Swap/VCONN) stay compiled out - a sink-only port
 * answers the partner's swap requests with Not_Supported.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/ProtocolLayer.hpp>
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
    TIMER cc; // the CC debounce
    TIMER prl;
    TIMER pe;
    TIMER swap; // the engine's swap exchanges: a single-role port never arms it
};

template<
    concepts::cc CC,
    concepts::pd_phy PHY,
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
        CC& cc,
        PHY& phy,
        VBUS& vbus,
        pd_sink_timers<TIMER>& timers,
        std::span<sink_capability const> capabilities,
        POLICY& policy,
        POWER& power,
        OBSERVERs&... observers
    )
        : prl_(phy, timers.prl, observers...), engine_(
                                                   prl_,
                                                   timers.pe,
                                                   timers.swap,
                                                   capabilities,
                                                   policy,
                                                   power,
                                                   directions_,
                                                   observers...
                                               ),
          directions_{*this}, sink_(cc, vbus, timers.tc, timers.cc, prl_, engine_, observers...)
    {
    }

    // Go live: present Rd and negotiate when a source attaches
    void start() { sink_.start(); }

private:
    // The engine's directions to the connection layer, executed on it
    // from the engine's machine. A forwarding object, because the two
    // machines' types are made of their observers' types: the
    // connection machine lists the engine, so the engine's machine
    // cannot list the connection port in turn
    struct connection_directions : fsm::observing<connection_directions> {
        explicit connection_directions(PdSink& port_ref) : port(port_ref) {}

        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engine
        void notifyEntry(tc::request_error_recovery) { port.sink_.errorRecovery(); }
        // the attach is held while VBUS legitimately cycles; the
        // engine stays live to await the capabilities
        void notifyEntry(tc::hard_reset_window) { port.sink_.hardResetWindow(); }

        PdSink& port;
    };

    using Layer = ProtocolLayer<PHY, TIMER, OBSERVERs...>;
    using SinkEngine =
        SinkPolicyEngine<Layer, TIMER, POLICY, POWER, connection_directions, OBSERVERs...>;
    // the layer ahead of the engine: it holds the PHY and the CC line
    // before the engine's first transmission
    using Sink = TypeCSink<CC, VBUS, TIMER, Layer, SinkEngine, OBSERVERs...>;

    Layer prl_;
    SinkEngine engine_;
    connection_directions directions_;
    Sink sink_;
};

} // namespace usbc
