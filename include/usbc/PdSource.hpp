/*
 * USB PD source port: the complete single-role port behind one class.
 * Composes the Type-C source connection layer with the source policy
 * engine: the attach starts the advertisement, the PD alert bits
 * reach the engine, and an exhausted HardResetCounter escalates to
 * Type-C Error Recovery. A hard reset's VBUS cycle is the engine's
 * own doing (Transition_to_default drives the supply through vSafe0V
 * and back) and needs no connection-layer window - source detach
 * detection is CC-based. Engine-tearing actions run straight from
 * the observing hooks: every machine is queued (fsm::QueuedMachine),
 * so the fallout is ordered delivery, not re-entrancy.
 *
 * The user provides the domain pieces only: the drivers (TCPC, VBUS),
 * the timers, the capabilities, the request policy, and the power
 * effects (the supply and a SourcePower-derived monitor). Extra
 * observers may be injected into the connection machine. The optional
 * features (PR_Swap/DR_Swap/VCONN) stay compiled out - a source-only
 * port answers the partner's swap requests with Not_Supported.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

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
    TIMER prl;
    TIMER pe;
};

template<
    concepts::tcpc TCPC,
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
        TCPC& tcpc,
        VBUS& vbus,
        pd_source_timers<TIMER>& timers,
        std::span<std::uint32_t const> capabilities,
        POLICY& policy,
        SUPPLY& supply,
        POWER& power,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : watch_{*this},
          engine_(tcpc, timers.prl, timers.pe, capabilities, policy, supply, power, watch_),
          router_{*this}, source_(tcpc, vbus, timers.tc, advertisement, router_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    PdSource(
        TCPC& tcpc,
        VBUS& vbus,
        pd_source_timers<TIMER>& timers,
        std::span<std::uint32_t const> capabilities,
        POLICY& policy,
        SUPPLY& supply,
        POWER& power,
        OBSERVERs&... observers
    )
        : PdSource(
              tcpc,
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
    // Injected into the engine's machine: watches the states' port
    // requests (the swap and VCONN observations cannot occur - their
    // states are compiled out). Error Recovery tears the engine down;
    // the queued machines order the fallout
    struct port_watch : fsm::observing<port_watch> {
        explicit port_watch(PdSource& port_ref) : port(port_ref) {}

        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engine
        void notifyEntry(pe::request_error_recovery) { port.source_.errorRecovery(); }

        PdSource& port;
    };

    using SourceEngine = SourcePolicyEngine<TCPC, TIMER, POLICY, SUPPLY, POWER, port_watch>;

    // The port's internal wiring in the connection machine, observing
    // the states' declared facts: the PD connection brackets the
    // engine's life, the PD alerts are forwarded from the driver
    // frontend
    struct router : fsm::observing<router> {
        explicit router(PdSource& port_ref) : port(port_ref) {}

        using observes = mtl::typelist<tc::pd_connection>;

        void notifyEntry(tc::pd_connection) { port.engine_.attached(); }
        void notifyExit(tc::pd_connection) { port.engine_.detached(); }

        void onPdAlert(alert_status alerts) { port.engine_.onAlert(alerts); }

        PdSource& port;
    };

    using Source = TypeCSource<TCPC, VBUS, TIMER, router, OBSERVERs...>;

    port_watch watch_;
    SourceEngine engine_;
    router router_;
    Source source_;
};

} // namespace usbc
