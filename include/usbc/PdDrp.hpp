/*
 * USB PD dual-role port: the complete port behind one class. Composes
 * the Type-C DRP connection layer with one policy engine per power
 * role and routes between them internally - the resolved role's engine
 * is activated through the attach observation, the PD alert bits reach
 * the active engine, and the TCPC's message header tracks the current
 * power and data roles. A role swap tears the old role's engine down
 * and brings the other one up through the same path as a plug-in.
 *
 * The user provides the domain pieces only: the drivers (TCPC, VBUS),
 * the timers, the capabilities and policies of both roles, the power
 * effects (a SinkPower- and a SourcePower-derived class, and the
 * supply), and drives the port through start() and the role swap
 * calls. No knowledge of the state machines is required; extra
 * observers (e.g. usbc::zephyr::StateLogger) may still be injected
 * into the connection machine, and may veto swaps via
 * allowSwap(power_role/data_role).
 *
 * The role swap phases follow TypeCDrp: begin, then completeSwap() on
 * the partner's PS_RDY (the PR_Swap/DR_Swap messaging is not part of
 * the engines yet - until then the calls stand in for it).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCDrp.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <optional>
#include <span>

namespace usbc {

// The timers the port runs on: the connection layer's, and a protocol
// plus an engine timer per role. One bundle owned by the caller
template<fsm::concepts::timer TIMER>
struct pd_drp_timers {
    TIMER tc;
    TIMER sink_prl;
    TIMER sink_pe;
    TIMER source_prl;
    TIMER source_pe;
};

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         concepts::sink_policy SINK_POLICY, typename SINK_POWER,
         concepts::source_policy SOURCE_POLICY, concepts::source_supply SUPPLY,
         typename SOURCE_POWER, drp_timing const& TIMING = default_drp_timing,
         drp_preference PREFERENCE = drp_preference::none, typename... OBSERVERs>
class PdDrp {
public:
    PdDrp(TCPC& tcpc, VBUS& vbus, pd_drp_timers<TIMER>& timers,
          std::span<sink_capability const> sink_capabilities, SINK_POLICY& sink_policy,
          SINK_POWER& sink_power, std::span<std::uint32_t const> source_capabilities,
          SOURCE_POLICY& source_policy, SUPPLY& supply, SOURCE_POWER& source_power,
          rp_value advertisement, OBSERVERs&... observers)
        : sink_engine_(tcpc, timers.sink_prl, timers.sink_pe, sink_capabilities, sink_policy,
                       sink_power),
          source_engine_(tcpc, timers.source_prl, timers.source_pe, source_capabilities,
                         source_policy, supply, source_power),
          router_{{}, tcpc, sink_engine_, source_engine_},
          drp_(tcpc, vbus, timers.tc, advertisement, router_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    PdDrp(TCPC& tcpc, VBUS& vbus, pd_drp_timers<TIMER>& timers,
          std::span<sink_capability const> sink_capabilities, SINK_POLICY& sink_policy,
          SINK_POWER& sink_power, std::span<std::uint32_t const> source_capabilities,
          SOURCE_POLICY& source_policy, SUPPLY& supply, SOURCE_POWER& source_power,
          OBSERVERs&... observers)
        : PdDrp(tcpc, vbus, timers, sink_capabilities, sink_policy, sink_power,
                source_capabilities, source_policy, supply, source_power,
                rp_value::usb_default, observers...)
    {
    }

    // Go live: toggle Rd/Rp and resolve the roles with the partner
    void start() { drp_.start(); }

    // Role swaps, in TypeCDrp's phases; the injected observers may veto
    bool beginSwapToSource() { return drp_.beginSwapToSource(); }
    bool beginSwapToSink() { return drp_.beginSwapToSink(); }
    bool completeSwap() { return drp_.completeSwap(); }
    bool abortSwap() { return drp_.abortSwap(); }
    bool swapDataRole() { return drp_.swapDataRole(); }

    std::optional<power_role> powerRole() const { return drp_.powerRole(); }
    std::optional<data_role> dataRole() const { return drp_.dataRole(); }

private:
    using SinkEngine = SinkPolicyEngine<TCPC, TIMER, SINK_POLICY, SINK_POWER>;
    using SourceEngine =
        SourcePolicyEngine<TCPC, TIMER, SOURCE_POLICY, SUPPLY, SOURCE_POWER>;

    // The port's internal wiring: activates the engine the resolved
    // role needs, routes the PD alerts to it, accepts the swaps this
    // class initiates, and keeps the message header's roles current
    struct router : fsm::observing<router> {
        TCPC& tcpc;
        SinkEngine& snk;
        SourceEngine& src;

        enum class active_role { none, sink, source };
        active_role active   = active_role::none;
        data_role data       = data_role::ufp;
        bool swapping        = false; // a power swap preserves the data role

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.attachedInfo()))
        {
            return state.attachedInfo();
        }
        void notifyEntry(tc::attach_info)
        {
            if (!swapping) {
                data = data_role::ufp; // a fresh sink attach is UFP
            }
            swapping = false;
            header(power_role::sink);
            active = active_role::sink;
            snk.vbusPresent();
        }
        void notifyExit(tc::attach_info)
        {
            snk.vbusRemoved();
            active = active_role::none;
        }
        void notifyEntry(plug_orientation)
        {
            if (!swapping) {
                data = data_role::dfp; // a fresh source attach is DFP
            }
            swapping = false;
            header(power_role::source);
            active = active_role::source;
            src.attached();
        }
        void notifyExit(plug_orientation)
        {
            src.detached();
            active = active_role::none;
        }

        void onPdAlert(alert_status alerts)
        {
            switch (active) {
            case active_role::sink: snk.onAlert(alerts); break;
            case active_role::source: src.onAlert(alerts); break;
            case active_role::none: break; // nobody negotiating
            }
        }

        // The swaps enter through this class's methods - the port says
        // yes, additional injected observers may still veto
        bool allowSwap(power_role)
        {
            swapping = true;
            return true;
        }
        bool allowSwap(data_role) { return true; }
        void onDataRole(data_role role)
        {
            data = role;
            header(active == active_role::source ? power_role::source : power_role::sink);
        }

        void header(power_role power)
        {
            tcpc.setMessageHeaderInfo({power, data, pd_revision::rev_3_x});
        }
    };

    using Drp = TypeCDrp<TCPC, VBUS, TIMER, TIMING, PREFERENCE, router, OBSERVERs...>;

    SinkEngine sink_engine_;
    SourceEngine source_engine_;
    router router_;
    Drp drp_;
};

} // namespace usbc
