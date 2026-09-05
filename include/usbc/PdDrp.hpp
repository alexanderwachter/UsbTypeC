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
 * into the connection machine.
 *
 * PR_Swap, DR_Swap and VCONN are optional features, enabled by tag:
 * an injected observer declaring `using enables = pe::..._feature;`
 * (or an mtl::typelist of tags) switches the feature on and becomes
 * its arbitration voice - it must satisfy the feature's contract
 * (allowSwap(power_role/data_role); the VCONN enabler is also the
 * switch-hardware connector, concepts::vconn_port). Without an
 * enabler the feature's engine states are filtered from the tables
 * and the partner's requests are answered Not_Supported/Reject.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCDrp.hpp>
#include <usbc/Vconn.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace usbc {

// The timers the port runs on: the connection layer's, a protocol
// plus an engine timer per role, and the vconn machine's. One bundle
// owned by the caller
template<fsm::concepts::timer TIMER>
struct pd_drp_timers {
    TIMER tc;
    TIMER tc_deadline; // the Try phases' hard walls (preference != none)
    TIMER sink_prl;
    TIMER sink_pe;
    TIMER source_prl;
    TIMER source_pe;
    TIMER vconn;
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
          vconn_timer_{timers.vconn, *this},
          vconn_(pickVconnPort(observers...), vconn_timer_, watch_),
          router_{tcpc, sink_engine_, source_engine_, *this},
          drp_(tcpc, vbus, timers.tc, timers.tc_deadline, advertisement, router_,
               observers...)
    {
        sink_engine_.setIdleHook([](void* self) { static_cast<PdDrp*>(self)->pump(); }, this);
        source_engine_.setIdleHook([](void* self) { static_cast<PdDrp*>(self)->pump(); },
                                   this);
        // a DRP answers Get_Source_Cap/Get_Sink_Cap in either role
        sink_engine_.provideSourceCapabilities(source_capabilities);
        source_engine_.provideSinkCapabilities(sink_capabilities);
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

    // Sends a VCONN_Swap; giving the role up needs no capability,
    // taking it is arbitrated through the injected observers'
    // allowSwap(vconn_source_role)
    bool swapVconnRole()
    {
        auto const role = drp_.powerRole();
        if (!role || !allowVconnSwap()) {
            return false;
        }
        return *role == power_role::sink ? sink_engine_.requestVconnSwap()
                                         : source_engine_.requestVconnSwap();
    }

    std::optional<power_role> powerRole() const { return drp_.powerRole(); }
    std::optional<data_role> dataRole() const { return drp_.dataRole(); }
    bool isVconnSource() const { return vconn_.isVconnSource(); }

private:
    // The optional features, detected by tag over the injected pack
    template<typename TAG>
    static constexpr bool feature_enabled =
        (pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>, TAG> || ...);

    static constexpr bool pr_swap_enabled = feature_enabled<pe::pr_swap_feature>;
    static constexpr bool dr_swap_enabled = feature_enabled<pe::dr_swap_feature>;
    // VCONN only matters for emarked/high-speed cables and contracts
    // above 3 A; without an enabler the vconn machine is a stub
    static constexpr bool vconn_enabled = feature_enabled<pe::vconn_feature>;

    // An enabling observer is the feature's arbitration voice (and,
    // for VCONN, the switch-hardware connector) - hold it to the
    // feature's contract right where the tag is honored
    static_assert(((!pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::pr_swap_feature> ||
                    concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, power_role>) &&
                   ...),
                  "an observer enabling pr_swap_feature must provide "
                  "allowSwap(power_role) -> bool");
    static_assert(((!pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::dr_swap_feature> ||
                    concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, data_role>) &&
                   ...),
                  "an observer enabling dr_swap_feature must provide "
                  "allowSwap(data_role) -> bool");
    static_assert(((!pe::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::vconn_feature> ||
                    concepts::vconn_port<std::remove_cvref_t<OBSERVERs>>) &&
                   ...),
                  "an observer enabling vconn_feature must satisfy "
                  "concepts::vconn_port (setVconn(bool) + allowSwap(vconn_source_role))");

    // The inert hardware stand-in when the feature is compiled out
    struct no_vconn_port {
        bool setVconn(bool) { return true; }
        bool allowSwap(vconn_source_role) { return false; }
    };

    template<typename T>
    struct is_vconn_enabler
        : std::bool_constant<pe::observer_enables_v<T, pe::vconn_feature>> {};

    // Lazy: the filtered pack is only fronted when the feature exists
    template<bool ENABLED, typename = void>
    struct vconn_port_type : std::type_identity<no_vconn_port> {};
    template<typename DUMMY>
    struct vconn_port_type<true, DUMMY>
        : std::type_identity<
              mtl::front_t<mtl::filter_t<mtl::typelist<std::remove_cvref_t<OBSERVERs>...>,
                                         is_vconn_enabler>>> {};

    using vconn_port_t = typename vconn_port_type<vconn_enabled>::type;

    // The first vconn-enabling observer of the pack, or the stand-in
    auto& pickVconnPort() { return dummy_vconn_port_; }
    template<typename FIRST, typename... REST>
    auto& pickVconnPort(FIRST& first, REST&... rest)
    {
        if constexpr (pe::observer_enables_v<std::remove_cvref_t<FIRST>, pe::vconn_feature>) {
            return first;
        } else {
            return pickVconnPort(rest...);
        }
    }

    // The user's policies extended with the swap arbitration: the
    // engines consult allowSwap for the partner's PR_Swap/DR_Swap, and
    // the verdict is the connection layer's (every enabling observer
    // may veto). Each hook exists only while its feature is enabled -
    // its absence is what filters the feature's states from the
    // engines' tables
    struct sink_policy_proxy {
        SINK_POLICY& inner;
        PdDrp& port;

        std::optional<contract_request> select(
            std::span<std::uint32_t const> source_capabilities,
            std::span<sink_capability const> capabilities) const
        {
            return inner.select(source_capabilities, capabilities);
        }
        bool allowSwap(power_role role)
            requires(pr_swap_enabled)
        {
            return port.drp_.swapAllowed(role);
        }
        bool allowSwap(data_role role)
            requires(dr_swap_enabled)
        {
            return port.drp_.swapAllowed(role);
        }
        bool allowSwap(vconn_source_role)
            requires(vconn_enabled)
        {
            return port.allowVconnSwap();
        }
    };

    struct source_policy_proxy {
        SOURCE_POLICY& inner;
        PdDrp& port;

        std::optional<supply_target> evaluate(
            std::uint32_t rdo, std::span<std::uint32_t const> capabilities) const
        {
            return inner.evaluate(rdo, capabilities);
        }
        bool allowSwap(power_role role)
            requires(pr_swap_enabled)
        {
            return port.drp_.swapAllowed(role);
        }
        bool allowSwap(data_role role)
            requires(dr_swap_enabled)
        {
            return port.drp_.swapAllowed(role);
        }
        bool allowSwap(vconn_source_role)
            requires(vconn_enabled)
        {
            return port.allowVconnSwap();
        }
    };

    // Relinquishing VCONN is always fine; becoming the VCONN source is
    // board-dependent and must match the VIF's claim - the injected
    // observers' allowSwap(vconn_source_role) decides, refused with none
    bool allowVconnSwap()
    {
        return vconn_.isVconnSource() || drp_.swapAllowed(vconn_source_role{});
    }

    // What a completed engine step asks the Type-C layer to do; flips
    // tear an engine down, so they never run inside the reporting
    // machine's process() - pump() executes them once it is idle
    enum class pending_action : std::uint8_t {
        none,
        standby_to_source, // agreed SNK->SRC swap: hold the standby
        complete_to_src,   // PE_PRS_SNK_SRC_Assert_Rp
        begin_to_sink,     // PE_PRS_SRC_SNK_Assert_Rd
        complete_to_snk,   // the new source's PS_RDY arrived
        error_recovery,    // nHardResetCount exhausted
        hard_reset_window, // hold the attach while VBUS cycles
        vconn_announce,    // the switch is on: transmit our PS_RDY
        hard_reset,        // a VCONN hand-off timed out
    };

    // Injected into both engines' machines: watches the swap states'
    // portReport() observations
    struct swap_watch : fsm::observing<swap_watch> {
        explicit swap_watch(PdDrp& port_ref) : port(port_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.portReport()))
        {
            return state.portReport();
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
        void notifyEntry(pe::request_error_recovery)
        {
            port.pending_ = pending_action::error_recovery;
        }
        void notifyEntry(pe::hard_reset_window)
        {
            port.pending_ = pending_action::hard_reset_window;
        }
        // the VCONN hand-off: the engines' progress feeds the vconn
        // machine (a different machine - safe synchronously); its own
        // requests defer like every engine-touching action
        void notifyEntry(pe::vconn_swap_agreed) { port.vconn_.swapAgreed(); }
        void notifyEntry(pe::vconn_partner_on) { port.vconn_.partnerPsRdy(); }
        void notifyEntry(pe::vconn_ps_rdy_sent) { port.vconn_.psRdySent(); }
        void notifyEntry(pe::announce_vconn_on)
        {
            port.pending_ = pending_action::vconn_announce;
        }
        void notifyEntry(pe::request_hard_reset) { port.pending_ = pending_action::hard_reset; }

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
                // seed the PD3 collision-avoidance view of the
                // source's Rp; CC alerts keep it current from here
                snk.sinkTxChanged(sinkTxOk(machine.template getIf<NEW_STATE>()->context.cc));
                if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_snk>) {
                    snk.finishSwap(); // already active mid PR_Swap
                } else if constexpr (std::is_same_v<OLD_STATE,
                                                    tc::state::hard_reset_recover_snk>) {
                    snk.vbusPresent(); // already active: VBUS is back
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
                    // only a fresh source attach starts VCONN, and
                    // only when the cable presents Ra
                    port.vconn_.attachedSource(raPresent(
                        machine.template getIf<NEW_STATE>()->context.cc));
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
                // the swap standby and the hard-reset window keep the
                // sink engine live: it still awaits the partner
                if constexpr (!std::is_same_v<NEW_STATE, tc::drp::swap_standby_to_src> &&
                              !std::is_same_v<NEW_STATE, tc::state::hard_reset_snk>) {
                    snk.vbusRemoved();
                    active = active_role::none;
                    port.vconn_.detached();
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::hard_reset_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::hard_reset_recover_snk>) {
                    snk.vbusRemoved(); // window timed out: dead port
                    active = active_role::none;
                    port.vconn_.detached();
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::hard_reset_recover_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                    snk.vbusRemoved(); // window timed out: dead port
                    active = active_role::none;
                    port.vconn_.detached();
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::state::attached_src>) {
                if constexpr (std::is_same_v<NEW_STATE, tc::drp::swap_standby_to_snk>) {
                    swap_revision = src.negotiatedRevision(); // before the reset
                } else {
                    port.vconn_.detached(); // a real detach, not a swap
                }
                src.detached();
                active = active_role::none;
            } else if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_src>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_src>) {
                    snk.vbusRemoved(); // the swap failed: engine resets
                    active = active_role::none;
                    port.vconn_.detached();
                }
            } else if constexpr (std::is_same_v<OLD_STATE, tc::drp::swap_standby_to_snk>) {
                if constexpr (!std::is_same_v<NEW_STATE, tc::state::attached_snk>) {
                    snk.vbusRemoved(); // the swap failed: engine resets
                    active = active_role::none;
                    port.vconn_.detached();
                }
            }
        }

        // PD3 collision avoidance: the sink reads the source's Rp as
        // SinkTxOk (3.0 A) / SinkTxNG on every CC report
        void onCcStatus(cc_status cc)
        {
            if (active == active_role::sink) {
                snk.sinkTxChanged(sinkTxOk(cc));
            }
        }

        static bool sinkTxOk(cc_status cc) { return tc::sinkTxOk(cc); }

        static bool raPresent(cc_status cc)
        {
            return cc.cc1 == cc_state::src_ra || cc.cc2 == cc_state::src_ra;
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

        // No allowSwap here: the enabling observers are the sole
        // arbitration voice, asked through the connection layer
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
        case pending_action::error_recovery:
            // terminations removed for tErrorRecovery, resolution
            // restarts; the teardown resets the engines
            drp_.errorRecovery();
            break;
        case pending_action::hard_reset_window:
            // the attach is held while VBUS legitimately cycles; the
            // sink engine stays live to await the capabilities
            drp_.hardResetWindow();
            break;
        case pending_action::vconn_announce:
            // the vconn machine turned the switch on: the active
            // engine announces it with PS_RDY
            if (router_.active == router::active_role::sink) {
                sink_engine_.sendVconnPsRdy();
            } else if (router_.active == router::active_role::source) {
                source_engine_.sendVconnPsRdy();
            }
            break;
        case pending_action::hard_reset:
            // the VCONN hand-off timed out: escalate, VCONN stays here
            if (router_.active == router::active_role::sink) {
                sink_engine_.hardReset();
            } else if (router_.active == router::active_role::source) {
                source_engine_.hardReset();
            }
            vconn_.swapFailureHandled();
            break;
        case pending_action::none: break;
        }
    }

    // The vconn machine's timer, wrapped: its timeout records a port
    // request that must run once the machine finished processing
    struct VconnTimer {
        TIMER& inner;
        PdDrp& port;
        fsm::timer_callback callback = nullptr;
        void* context                = nullptr;

        void start(std::chrono::milliseconds duration, fsm::timer_callback cb, void* ctx)
        {
            callback = cb;
            context  = ctx;
            inner.start(
                duration,
                [](void* self) {
                    auto& timer = *static_cast<VconnTimer*>(self);
                    timer.callback(timer.context);
                    timer.port.pump();
                },
                this);
        }
        void stop() { inner.stop(); }
    };

    // Stand-in when VCONN is compiled out: every call site stays
    // valid, nothing ever sources
    struct no_vconn {
        no_vconn(auto&, VconnTimer&, swap_watch&) {}
        void attachedSource(bool) {}
        void detached() {}
        void swapAgreed() {}
        void partnerPsRdy() {}
        void psRdySent() {}
        void swapFailureHandled() {}
        bool isVconnSource() const { return false; }
    };

    using Drp = TypeCDrp<TCPC, VBUS, TIMER, TIMING, PREFERENCE, router, OBSERVERs...>;

    sink_policy_proxy sink_policy_;
    source_policy_proxy source_policy_;
    swap_watch watch_;
    SinkEngine sink_engine_;
    SourceEngine source_engine_;
    VconnTimer vconn_timer_;
    [[no_unique_address]] no_vconn_port dummy_vconn_port_{};
    std::conditional_t<vconn_enabled, VconnMachine<vconn_port_t, VconnTimer, swap_watch>,
                       no_vconn>
        vconn_;
    router router_;
    Drp drp_;
    pending_action pending_ = pending_action::none;
};

} // namespace usbc
