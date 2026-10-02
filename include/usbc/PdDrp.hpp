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
 * partner's requests are the engine tables' questions
 * (pe::pr_swap_allowed and kin), answered by this class's policy
 * proxies from the injected observers' allowSwap hooks. The engines
 * run the message choreography (Accept,
 * Transition_to_off, the PS_RDY exchange); this class watches their
 * swap states and flips the Type-C terminations at the spec's
 * Assert_Rd/Assert_Rp moments. A flip tears the old role's engine down
 * and brings the other one up, right from the observing hooks: every
 * machine is queued (fsm::QueuedMachine), so what feeds back into the
 * reporting machine is delivered after its running transition instead
 * of re-entering it.
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
    TIMER tc_try; // the timeouts inside a Try phase (preference != none)
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
          vconn_(pickVconnPort(observers...), timers.vconn, watch_),
          router_(tcpc, sink_engine_, source_engine_, *this),
          drp_(tcpc, vbus, timers.tc, timers.tc_try, advertisement, router_,
               observers...)
    {
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
        auto const role = drp_.powerRole();
        if (!role || !dataSwapAllowed()) {
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
        fsm::feature_enabled_v<TAG, std::remove_cvref_t<OBSERVERs>...>;

    static constexpr bool pr_swap_enabled = feature_enabled<pe::pr_swap_feature>;
    static constexpr bool dr_swap_enabled = feature_enabled<pe::dr_swap_feature>;
    // VCONN only matters for emarked/high-speed cables and contracts
    // above 3 A; without an enabler the vconn machine is a stub
    static constexpr bool vconn_enabled = feature_enabled<pe::vconn_feature>;

    // An enabling observer is the feature's arbitration voice (and,
    // for VCONN, the switch-hardware connector) - hold it to the
    // feature's contract right where the tag is honored
    static_assert(((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::pr_swap_feature> ||
                    concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, power_role>) &&
                   ...),
                  "an observer enabling pr_swap_feature must provide "
                  "allowSwap(power_role) -> bool");
    static_assert(((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
                                            pe::dr_swap_feature> ||
                    concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, data_role>) &&
                   ...),
                  "an observer enabling dr_swap_feature must provide "
                  "allowSwap(data_role) -> bool");
    static_assert(((!fsm::observer_enables_v<std::remove_cvref_t<OBSERVERs>,
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

    // The vconn-enabling observer is the port; the stand-in without one
    using vconn_enabler_t =
        fsm::feature_enabler_t<pe::vconn_feature,
                               mtl::typelist<std::remove_cvref_t<OBSERVERs>...>>;
    using vconn_port_t = std::conditional_t<std::is_same_v<vconn_enabler_t, mtl::nil_type>,
                                            no_vconn_port, vconn_enabler_t>;

    // The first vconn-enabling observer of the pack, or the stand-in
    auto& pickVconnPort() { return dummy_vconn_port_; }
    template<typename FIRST, typename... REST>
    auto& pickVconnPort(FIRST& first, REST&... rest)
    {
        if constexpr (fsm::observer_enables_v<std::remove_cvref_t<FIRST>, pe::vconn_feature>) {
            return first;
        } else {
            return pickVconnPort(rest...);
        }
    }

    // The user's policies extended with the answers to the engine
    // tables' swap questions (pe::pr_swap_allowed and kin, asked on
    // the partner's request from Ready): the verdict is the connection
    // layer's (every enabling observer may veto), asked with the role
    // this engine's port would take. Each answer exists only while its
    // feature is enabled - its absence is what filters the feature's
    // states from the engine's table
    struct sink_policy_proxy {
        SINK_POLICY& inner;
        PdDrp& port;

        std::optional<contract_request> select(
            std::span<std::uint32_t const> source_capabilities,
            std::span<sink_capability const> capabilities) const
        {
            return inner.select(source_capabilities, capabilities);
        }
        bool check(pe::pr_swap_allowed)
            requires(pr_swap_enabled)
        {
            return port.drp_.swapAllowed(power_role::source); // a sink swaps to sourcing
        }
        bool check(pe::dr_swap_allowed)
            requires(dr_swap_enabled)
        {
            return port.dataSwapAllowed();
        }
        bool check(pe::vconn_swap_allowed)
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
        bool check(pe::pr_swap_allowed)
            requires(pr_swap_enabled)
        {
            return port.drp_.swapAllowed(power_role::sink); // a source swaps to sinking
        }
        bool check(pe::dr_swap_allowed)
            requires(dr_swap_enabled)
        {
            return port.dataSwapAllowed();
        }
        bool check(pe::vconn_swap_allowed)
            requires(vconn_enabled)
        {
            return port.allowVconnSwap();
        }
    };

    // A data role swap is asked with the role the port would take:
    // the opposite of the attached pair's; refused while not attached
    bool dataSwapAllowed()
    {
        auto const current = drp_.dataRole();
        if (!current) {
            return false;
        }
        return drp_.swapAllowed(*current == data_role::ufp ? data_role::dfp : data_role::ufp);
    }

    // Relinquishing VCONN is always fine; becoming the VCONN source is
    // board-dependent and must match the VIF's claim - the injected
    // observers' allowSwap(vconn_source_role) decides, refused with none
    bool allowVconnSwap()
    {
        return vconn_.isVconnSource() || drp_.swapAllowed(vconn_source_role{});
    }

    // Injected into both engines' machines: watches the swap states'
    // annotations, and executes the requested port actions right from
    // the hooks: every machine is queued, so an action that feeds back
    // into the reporting machine (an engine flip, a vconn
    // acknowledgment) is delivered after the running transition
    struct swap_watch : fsm::observing<swap_watch> {
        explicit swap_watch(PdDrp& port_ref) : port(port_ref) {}

        // an agreed DR_Swap only touches the header and the Type-C
        // context
        void notifyEntry(pe::data_role_changed) { port.drp_.applyDataRoleSwap(); }
        // the swap is agreed: the standby suspends detach detection
        // while the old source collapses VBUS, and its tPSSourceOff
        // timeout restarts resolution if the PS_RDY never comes; the
        // sink engine stays live for it
        void notifyEntry(pe::enter_swap_standby) { port.drp_.beginSwapToSource(); }
        // Assert_Rp completes into the source role (the source engine
        // takes over, drives VBUS and answers with PS_RDY); Assert_Rd
        // holds the standby whose tPSSourceOn guards the partner's
        // PS_RDY
        void notifyEntry(pe::assert_new_role role)
        {
            if (role.role == power_role::source) {
                port.drp_.completeSwap();
            } else {
                port.drp_.beginSwapToSink();
            }
        }
        void notifyEntry(pe::swap_completed) { port.drp_.completeSwap(); }
        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engines
        void notifyEntry(pe::request_error_recovery) { port.drp_.errorRecovery(); }
        // the attach is held while VBUS legitimately cycles; the sink
        // engine stays live to await the capabilities
        void notifyEntry(pe::hard_reset_window) { port.drp_.hardResetWindow(); }
        // the VCONN hand-off: the engines' progress feeds the vconn
        // machine, whose own requests come back through the hooks below
        void notifyEntry(pe::vconn_swap_agreed) { port.vconn_.swapAgreed(); }
        void notifyEntry(pe::vconn_partner_on) { port.vconn_.partnerPsRdy(); }
        void notifyEntry(pe::vconn_ps_rdy_sent) { port.vconn_.psRdySent(); }
        // the vconn machine turned the switch on: the active engine
        // announces it with PS_RDY
        void notifyEntry(pe::announce_vconn_on) { port.announceVconnOn(); }
        // a VCONN hand-off timed out: escalate, VCONN stays here
        void notifyEntry(pe::request_hard_reset) { port.escalateVconnFailure(); }

        PdDrp& port;
    };

    using SinkEngine =
        SinkPolicyEngine<TCPC, TIMER, sink_policy_proxy, SINK_POWER, swap_watch>;
    using SourceEngine = SourcePolicyEngine<TCPC, TIMER, source_policy_proxy, SUPPLY,
                                            SOURCE_POWER, swap_watch>;

    // The port's internal wiring, observing the connection machine's
    // declared facts: the PD connection brackets the engines' life (its
    // tag spans the hard-reset window and the swap standbys, so neither
    // is a detach), the attached role names the engine the resolved
    // role needs, the swap standby hands the engines over, and every
    // entry of an Attached state - fresh, resumed, or completing a swap
    // - reaches the live engine as the partner value (data role for
    // the message header, the CC status for collision avoidance and
    // the cable's Ra). The PD alerts and CC reports are forwarded from
    // the driver frontend
    struct router : fsm::observing<router> {
        router(TCPC& tcpc_ref, SinkEngine& sink_engine, SourceEngine& source_engine,
               PdDrp& port_ref)
            : tcpc(tcpc_ref), snk(sink_engine), src(source_engine), port(port_ref)
        {
        }

        using observes = mtl::typelist<tc::pd_connection, tc::attached_role,
                                       tc::drp::swap_standby, tc::attached_partner>;

        TCPC& tcpc;
        SinkEngine& snk;
        SourceEngine& src;
        PdDrp& port;

        enum class active_role { none, sink, source };
        active_role active = active_role::none;
        data_role data     = data_role::ufp; // the message header's data role

        // the one real detach: the live engine resets, VCONN goes down
        void notifyExit(tc::pd_connection)
        {
            switch (active) {
            case active_role::sink: snk.vbusRemoved(); break;
            case active_role::source: src.detached(); break;
            case active_role::none: break;
            }
            active = active_role::none;
            port.vconn_.detached();
        }

        // the resolved role names the live engine (a swap's completion
        // included: the standby carries no role)
        void notifyEntry(tc::attached_role attached)
        {
            active = attached.role == power_role::sink ? active_role::sink : active_role::source;
        }

        // The old source asserted Rd mid PR_Swap: the sink engine takes
        // over the PS_RDY exchange, continuing the connection's
        // negotiated revision and MessageID lifecycle (a swap is no
        // reset trigger, 6.7.1) - captured before the retiring engine's
        // teardown resets its layer. The other direction hands over on
        // the swap's completion, the sink engine stays live until then
        void notifyEntry(tc::drp::swap_standby standby)
        {
            if (standby.to == power_role::sink) {
                auto const revision = src.negotiatedRevision();
                auto const ids      = src.messageIds();
                src.detached();
                header(power_role::sink);
                active = active_role::sink;
                snk.startSwapWaitSourceOn(data, revision, ids);
            }
        }

        // An Attached state entered, the live engine acts: a fresh
        // attach or the resume after a hard-reset window starts its
        // negotiation, the completion of a power swap continues it
        void notifyEntry(tc::attached_partner const& partner)
        {
            data = partner.data;
            bool const completing =
                partner.origin == tc::attach_origin::completed_power_role_swap;
            if (active == active_role::sink) {
                header(power_role::sink);
                // seed the PD3 collision-avoidance view of the
                // source's Rp; CC alerts keep it current from here
                snk.sinkTxChanged(tc::sinkTxOk(partner.cc));
                if (completing) {
                    snk.finishSwap(); // already live mid PR_Swap
                } else {
                    snk.vbusPresent(); // fresh attach, or VBUS back
                }
            } else {
                header(power_role::source);
                if (completing) {
                    // the sink engine's half is done: its revision
                    // and MessageIDs continue in the source engine
                    auto const revision = snk.negotiatedRevision();
                    auto const ids      = snk.messageIds();
                    snk.vbusRemoved();
                    src.attachedAfterSwap(data, revision, ids);
                } else {
                    src.attached();
                    // only a fresh source attach starts VCONN, and
                    // only when the cable presents Ra
                    port.vconn_.attachedSource(raPresent(partner.cc));
                }
            }
        }

        // PD3 collision avoidance: the sink reads the source's Rp as
        // SinkTxOk (3.0 A) / SinkTxNG on every CC report
        void onCcStatus(cc_status cc)
        {
            if (active == active_role::sink) {
                snk.sinkTxChanged(tc::sinkTxOk(cc));
            }
        }

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

    // The vconn machine turned the switch on: the active engine
    // announces it with PS_RDY
    void announceVconnOn()
    {
        if (router_.active == router::active_role::sink) {
            sink_engine_.sendVconnPsRdy();
        } else if (router_.active == router::active_role::source) {
            source_engine_.sendVconnPsRdy();
        }
    }

    // A VCONN hand-off timed out: escalate, VCONN stays here. The
    // acknowledgment feeds back into the reporting vconn machine -
    // queued, delivered after its timeout transition completed
    void escalateVconnFailure()
    {
        if (router_.active == router::active_role::sink) {
            sink_engine_.hardReset();
        } else if (router_.active == router::active_role::source) {
            source_engine_.hardReset();
        }
        vconn_.swapFailureHandled();
    }

    // Stand-in when VCONN is compiled out: every call site stays
    // valid, nothing ever sources
    struct no_vconn {
        no_vconn(auto&, TIMER&, swap_watch&) {}
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
    [[no_unique_address]] no_vconn_port dummy_vconn_port_{};
    std::conditional_t<vconn_enabled, VconnMachine<vconn_port_t, TIMER, swap_watch>,
                       no_vconn>
        vconn_;
    router router_;
    Drp drp_;
};

} // namespace usbc
