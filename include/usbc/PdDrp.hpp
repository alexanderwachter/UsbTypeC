/*
 * USB PD dual-role port: the complete port behind one class. Composes
 * the Type-C DRP connection layer with one policy engine per power
 * role and routes between them internally - the resolved role's engine
 * is activated through the attach observation, the PD alert bits reach
 * the active engine, and the TCPC's message header tracks the current
 * power and data roles.
 *
 * Role swaps are PD-negotiated (PR_Swap/DR_Swap): swapPowerRole() and
 * swapDataRole() send the request through the active engine, and the
 * partner's requests are arbitrated by the injected observers'
 * allowSwap hooks. The engines run the message choreography (Accept,
 * Transition_to_off, the PS_RDY exchange); this class watches their
 * swap states and flips the Type-C terminations at the spec's
 * Assert_Rd/Assert_Rp moments. A flip tears the old role's engine down
 * and brings the other one up, which must not happen while the
 * reporting machine still processes - such actions are deferred and
 * run from pump() once the call chain unwound.
 *
 * The user provides the domain pieces only: the drivers (TCPC, VBUS),
 * the timers, the capabilities and policies of both roles, the power
 * effects (a SinkPower- and a SourcePower-derived class, and the
 * supply). No knowledge of the state machines is required; extra
 * observers (e.g. usbc::zephyr::StateLogger) may still be injected
 * into the connection machine, and may veto swaps via
 * allowSwap(power_role/data_role).
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
        : sink_policy_{sink_policy, *this},
          source_policy_{source_policy, *this},
          watch_{*this},
          sink_engine_(tcpc, timers.sink_prl, timers.sink_pe, sink_capabilities, sink_policy_,
                       sink_power, watch_),
          source_engine_(tcpc, timers.source_prl, timers.source_pe, source_capabilities,
                         source_policy_, supply, source_power, watch_),
          router_{tcpc, sink_engine_, source_engine_, *this},
          drp_(tcpc, vbus, timers.tc, advertisement, router_, observers...)
    {
        source_engine_.setIdleHook([](void* self) { static_cast<PdDrp*>(self)->pump(); },
                                   this);
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

    // Sends a PR_Swap to the partner; the engines run the message
    // exchange and the VBUS hand-off from there. False when not
    // attached with an explicit contract, a swap is already running,
    // or an injected observer vetoes
    bool swapPowerRole()
    {
        auto const role = drp_.powerRole();
        if (!role) {
            return false;
        }
        if (*role == power_role::sink) {
            return drp_.swapAllowed(power_role::source) && sink_engine_.requestPowerSwap();
        }
        return drp_.swapAllowed(power_role::sink) && source_engine_.requestPowerSwap();
    }

    // Sends a DR_Swap; on the partner's Accept both sides flip and the
    // TCPC header follows
    bool swapDataRole()
    {
        auto const current = drp_.dataRole();
        auto const role    = drp_.powerRole();
        if (!current || !role) {
            return false;
        }
        auto const target = *current == data_role::ufp ? data_role::dfp : data_role::ufp;
        if (!drp_.swapAllowed(target)) {
            return false;
        }
        return *role == power_role::sink ? sink_engine_.requestDataSwap()
                                         : source_engine_.requestDataSwap();
    }

    std::optional<power_role> powerRole() const { return drp_.powerRole(); }
    std::optional<data_role> dataRole() const { return drp_.dataRole(); }

private:
    // The user's policies extended with the swap arbitration: the
    // engines consult allowSwap for the partner's PR_Swap/DR_Swap, and
    // the verdict is the connection layer's (every injected observer
    // may veto; the internal router says yes for the port itself)
    struct sink_policy_proxy {
        SINK_POLICY& inner;
        PdDrp& port;

        std::optional<contract_request> select(
            std::span<std::uint32_t const> source_capabilities,
            std::span<sink_capability const> capabilities) const
        {
            return inner.select(source_capabilities, capabilities);
        }
        bool allowSwap(power_role role) { return port.drp_.swapAllowed(role); }
        bool allowSwap(data_role role) { return port.drp_.swapAllowed(role); }
    };

    struct source_policy_proxy {
        SOURCE_POLICY& inner;
        PdDrp& port;

        std::optional<supply_target> evaluate(
            std::uint32_t rdo, std::span<std::uint32_t const> capabilities) const
        {
            return inner.evaluate(rdo, capabilities);
        }
        bool allowSwap(power_role role) { return port.drp_.swapAllowed(role); }
        bool allowSwap(data_role role) { return port.drp_.swapAllowed(role); }
    };

    // What a completed engine step asks the Type-C layer to do; flips
    // tear an engine down, so they never run inside the reporting
    // machine's process() - pump() executes them once it is idle
    enum class pending_action : std::uint8_t {
        none,
        standby_to_source, // agreed SNK->SRC swap: hold the standby
        complete_to_src,   // PE_PRS_SNK_SRC_Assert_Rp
        begin_to_sink,     // PE_PRS_SRC_SNK_Assert_Rd
        complete_to_snk,   // the new source's PS_RDY arrived
    };

    // Injected into both engines' machines: watches the swap states'
    // swapReport() observations
    struct swap_watch : fsm::observing<swap_watch> {
        explicit swap_watch(PdDrp& port_ref) : port(port_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.swapReport()))
        {
            return state.swapReport();
        }
        // an agreed DR_Swap only touches the header and the Type-C
        // context - safe to apply synchronously
        void notifyEntry(pe::data_role_changed) { port.drp_.applyDataRoleSwap(); }
        void notifyEntry(pe::enter_swap_standby)
        {
            port.pending_ = pending_action::standby_to_source;
        }
        void notifyEntry(pe::assert_new_role role)
        {
            port.pending_ = role.role == power_role::source ? pending_action::complete_to_src
                                                            : pending_action::begin_to_sink;
        }
        void notifyEntry(pe::swap_completed) { port.pending_ = pending_action::complete_to_snk; }

        PdDrp& port;
    };

    using SinkEngine =
        SinkPolicyEngine<TCPC, TIMER, sink_policy_proxy, SINK_POWER, swap_watch>;
    using SourceEngine = SourcePolicyEngine<TCPC, TIMER, source_policy_proxy, SUPPLY,
                                            SOURCE_POWER, swap_watch>;

    // The port's internal wiring: activates the engine the resolved
    // role needs, routes the PD alerts to it, and keeps the message
    // header's roles current. Library plumbing may look at the machine:
    // the raw hooks read the attached state's data role from the shared
    // context, which a power swap preserves and a fresh attach defaults
    struct router {
        TCPC& tcpc;
        SinkEngine& snk;
        SourceEngine& src;
        PdDrp& port;

        enum class active_role { none, sink, source };
        active_role active = active_role::none;
        data_role data     = data_role::ufp; // for the header between hooks
        // carried across the engine handover of a power role swap: the
        // negotiated revision holds for the connection
        pd_revision swap_revision = pd_revision::rev_3_x;

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onEnterState(MACHINE& machine)
        {
            if constexpr (std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                data = machine.template getIf<NEW_STATE>()->dataRole();
                header(power_role::sink);
                if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_snk>) {
                    snk.finishSwap(); // already active mid PR_Swap
                } else {
                    active = active_role::sink;
                    snk.vbusPresent();
                }
            } else if constexpr (std::is_same_v<NEW_STATE, tc::state::attached_src>) {
                data = machine.template getIf<NEW_STATE>()->dataRole();
                header(power_role::source);
                if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_src>) {
                    swap_revision = snk.negotiatedRevision();
                    snk.vbusRemoved(); // the sink engine's half is done
                    active = active_role::source;
                    src.attachedAfterSwap(data, swap_revision); // continue the PR_Swap
                } else {
                    active = active_role::source;
                    src.attached();
                }
            } else if constexpr (std::is_same_v<NEW_STATE, tc::drp::swap_standby_to_snk>) {
                // the old source asserted Rd mid PR_Swap: the sink
                // engine takes over the PS_RDY exchange
                header(power_role::sink);
                active = active_role::sink;
                snk.startSwapWaitSourceOn(data, swap_revision);
            }
        }

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onExitState(MACHINE&)
        {
            if constexpr (std::is_same_v<OLD_STATE, tc::state::attached_snk>) {
                // entering the swap standby keeps the sink engine
                // live: it still awaits the old source's PS_RDY
                if constexpr (!std::is_same_v<NEW_STATE, tc::drp::swap_standby_to_src>) {
                    snk.vbusRemoved();
                    active = active_role::none;
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::attached_src>) {
                if constexpr (std::is_same_v<NEW_STATE, tc::drp::swap_standby_to_snk>) {
                    swap_revision = src.negotiatedRevision(); // before the reset
                }
                src.detached();
                active = active_role::none;
            } else if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_src>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_src>) {
                    snk.vbusRemoved(); // the swap failed: engine resets
                    active = active_role::none;
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                    snk.vbusRemoved(); // the swap failed: engine resets
                    active = active_role::none;
                }
            }
        }

        void onPdAlert(alert_status alerts)
        {
            switch (active) {
            case active_role::sink: snk.onAlert(alerts); break;
            case active_role::source: src.onAlert(alerts); break;
            case active_role::none: break; // nobody negotiating
            }
            port.pump(); // flips recorded while routing run now
        }

        // The port itself has no veto - the user's injected observers
        // are asked alongside through the connection layer
        bool allowSwap(power_role) { return true; }
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

    // Executes the recorded Type-C action; called only when both
    // engine machines are idle (after alert routing, and from the
    // source engine's settle hook)
    void pump()
    {
        auto const action = pending_;
        pending_          = pending_action::none;
        switch (action) {
        case pending_action::standby_to_source:
            // the swap is agreed: the standby suspends detach
            // detection while the old source collapses VBUS, and its
            // tPSSourceOff timeout restarts resolution if the PS_RDY
            // never comes; the sink engine stays live for it
            drp_.beginSwapToSource();
            break;
        case pending_action::complete_to_src:
            // Assert_Rp: the source engine takes over, drives VBUS and
            // answers with PS_RDY
            drp_.completeSwap();
            break;
        case pending_action::begin_to_sink:
            // Assert_Rd and hold the standby: the sink engine sends
            // our PS_RDY and the standby's tPSSourceOn guards the
            // partner's (a timeout restarts connection resolution)
            drp_.beginSwapToSink();
            break;
        case pending_action::complete_to_snk:
            drp_.completeSwap();
            break;
        case pending_action::none: break;
        }
    }

    using Drp = TypeCDrp<TCPC, VBUS, TIMER, TIMING, PREFERENCE, router, OBSERVERs...>;

    sink_policy_proxy sink_policy_;
    source_policy_proxy source_policy_;
    swap_watch watch_;
    SinkEngine sink_engine_;
    SourceEngine source_engine_;
    router router_;
    Drp drp_;
    pending_action pending_ = pending_action::none;
};

} // namespace usbc
