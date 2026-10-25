/*
 * USB PD source port: the complete single-role port behind one class.
 * Composes the Type-C source connection layer, the port's protocol
 * layer and the source policy engine: the layer and the engine observe
 * the connection machine (the attach starts the advertisement, the
 * connection's exit is the detach), and the engine's one direction to
 * the connection layer - Error Recovery after an exhausted
 * HardResetCounter - is executed on it from the engine's machine. A
 * hard reset's VBUS cycle is the engine's own doing (Transition_to_
 * default drives the supply through vSafe0V and back) and needs no
 * connection-layer window - source detach detection is CC-based.
 * Every machine is queued (fsm::QueuedMachine), so the fallout is
 * ordered delivery, not re-entrancy.
 *
 * The user provides the domain pieces only: the hardware blocks (CC,
 * PD PHY, VBUS), the timers, the capabilities, the request policy,
 * and the power effects (the supply and a SourcePower-derived
 * monitor). Extra observers are handed to every machine of the port -
 * connection, engine, protocol layer - so a tracer sees all of them.
 * The optional features (PR_Swap/DR_Swap/VCONN) stay compiled out - a
 * source-only port answers the partner's swap requests with
 * Not_Supported.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/ProtocolLayer.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCSource.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <cstdint>
#include <span>

namespace usbc {

// The timers the port runs on: the connection layer's, the protocol
// layer's, and the engine's. One bundle owned by the caller
template<fsm::concepts::timer TIMER>
struct pd_source_timers {
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
    concepts::source_policy POLICY,
    concepts::source_supply SUPPLY,
    typename POWER,
    typename... OBSERVERs>
class PdSource {
    // The single-role facade wires none of the tag-enabled features:
    // an enabling observer here would be silently ignored
    static_assert(
        ((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::pr_swap_feature> &&
          !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::dr_swap_feature> &&
          !fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>, pe::vconn_feature>) &&
         ...),
        "PdSource does not wire the optional features (swaps, VCONN); "
        "a port offering them is a PdDrp"
    );

public:
    PdSource(
        CC& cc,
        PHY& phy,
        VBUS& vbus,
        pd_source_timers<TIMER>& timers,
        std::span<std::uint32_t const> capabilities,
        POLICY& policy,
        SUPPLY& supply,
        POWER& power,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : prl_(phy, timers.prl, observers...), engine_(
                                                   cc,
                                                   prl_,
                                                   timers.pe,
                                                   timers.swap,
                                                   capabilities,
                                                   advertisement,
                                                   policy,
                                                   supply,
                                                   power,
                                                   directions_,
                                                   observers...
                                               ),
          directions_{*this},
          source_(cc, vbus, timers.tc, timers.cc, advertisement, prl_, engine_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    PdSource(
        CC& cc,
        PHY& phy,
        VBUS& vbus,
        pd_source_timers<TIMER>& timers,
        std::span<std::uint32_t const> capabilities,
        POLICY& policy,
        SUPPLY& supply,
        POWER& power,
        OBSERVERs&... observers
    )
        : PdSource(
              cc,
              phy,
              vbus,
              timers,
              capabilities,
              policy,
              supply,
              power,
              rp_value::usb_default,
              observers...
          )
    {
    }

    // Go live: present Rp and advertise when a sink attaches
    void start() { source_.start(); }

private:
    // The engine's direction to the connection layer, executed on it
    // from the engine's machine. A forwarding object, because the two
    // machines' types are made of their observers' types: the
    // connection machine lists the engine, so the engine's machine
    // cannot list the connection port in turn
    struct connection_directions : fsm::observing<connection_directions> {
        explicit connection_directions(PdSource& port_ref) : port(port_ref) {}

        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engine
        void notifyEntry(tc::request_error_recovery) { port.source_.errorRecovery(); }

        PdSource& port;
    };

    using Layer = ProtocolLayer<PHY, TIMER, OBSERVERs...>;
    using SourceEngine = SourcePolicyEngine<
        CC,
        Layer,
        TIMER,
        POLICY,
        SUPPLY,
        POWER,
        connection_directions,
        OBSERVERs...>;
    // the layer ahead of the engine: it holds the PHY and the CC line
    // before the engine's first transmission
    using Source = TypeCSource<CC, VBUS, TIMER, Layer, SourceEngine, OBSERVERs...>;

    Layer prl_;
    SourceEngine engine_;
    connection_directions directions_;
    Source source_;
};

} // namespace usbc
