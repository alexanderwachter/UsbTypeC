/*
 * USB PD dual-role port: the complete port behind one class. Composes
 * the Type-C DRP connection layer, the port's one protocol layer, one
 * policy engine per power role and the VCONN machine, each observing
 * the others' machines: the layer and both engines observe the
 * connection machine (the resolved role's engine goes live on its
 * attach fact, the layer holds the PHY for the connection, the swap
 * standby hands the layer from one engine to the other, the
 * connection's exit is the detach), the vconn machine observes the
 * connection machine (a source attach with Ra) and the engines (the
 * VCONN_Swap's progress), and the engines' directions to the
 * connection layer - the swap phases, the hard-reset window, Error
 * Recovery - and the vconn machine's to the live engine - our PS_RDY,
 * a hard reset - are executed from the reporting machines. Every
 * machine is queued (fsm::QueuedMachine), so what feeds back into the
 * reporting machine is delivered after its running transition instead
 * of re-entering it.
 *
 * Role swaps are PD-negotiated (PR_Swap/DR_Swap): swapPowerRole() and
 * swapDataRole() send the request through the live engine, and the
 * partner's requests are the engine tables' questions
 * (pe::pr_swap_allowed and kin), answered by the injected observers
 * themselves - every observer reaches every machine of the port - and
 * a holder of the VCONN Source role accepts (pe::vconn_source_role_held,
 * answered by the vconn machine). The engines run the message
 * choreography (Accept, Transition_to_off, the PS_RDY exchange) and
 * their states direct the Type-C terminations at the spec's
 * Assert_Rd/Assert_Rp moments. One protocol layer serves both engines
 * in turn, so a power role swap continues the connection's MessageID
 * lifecycle and negotiated revision (a swap is no reset trigger,
 * 6.7.1).
 *
 * The user provides the domain pieces only: the hardware blocks (CC,
 * PD PHY, VBUS), the timers, the capabilities and policies of both
 * roles, the power effects (a SinkPower- and a SourcePower-derived
 * class, and the supply). No knowledge of the state machines is
 * required; extra observers (e.g. mtl::zephyr::TraceLogger) are handed
 * to every machine of the port - connection, protocol layer, both
 * engines, vconn.
 *
 * PR_Swap, DR_Swap and VCONN are optional features, enabled by
 * answering: an injected observer answering the engines' question
 * (bool check(pe::pr_swap_allowed), dr_swap_allowed,
 * vconn_swap_allowed - with the request's role to take where it
 * matters) brings the feature's states in and is its arbitration
 * voice; the VCONN answerer is also the switch-hardware connector
 * (concepts::vconn_port). Without an answerer the feature's engine
 * states are filtered from the tables and the partner's requests are
 * answered Not_Supported. A swap to source also passes the role lock
 * (usbc::sourcing_allowed) on the sink engine's row.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/TypeCDrp.hpp>
#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Cc.hpp>
#include <usbc/interface/PdPhy.hpp>
#include <usbc/interface/Policy.hpp>
#include <usbc/interface/Power.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/layer/ProtocolLayer.hpp>
#include <usbc/layer/SinkPolicyEngine.hpp>
#include <usbc/layer/SourcePolicyEngine.hpp>
#include <usbc/layer/Vconn.hpp>

#include <mtl/StateMachine.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace usbc {

// The timers the port runs on: the connection layer's, the protocol
// layer's, an engine timer per role and one for the timeouts inside
// the engine's swap exchanges, and the vconn machine's. One bundle
// owned by the caller
template<fsm::concepts::timer TIMER>
struct pd_drp_timers {
    TIMER tc;
    TIMER cc; // the CC debounce
    TIMER prl;
    TIMER sink_pe;
    TIMER sink_swap; // the timeouts inside a swap exchange (a swap feature on)
    TIMER source_pe;
    TIMER source_swap;
    TIMER vconn;
};

template<
    concepts::cc CC,
    concepts::pd_phy PHY,
    concepts::vbus VBUS,
    fsm::concepts::timer TIMER,
    concepts::sink_policy SINK_POLICY,
    typename SINK_POWER,
    concepts::source_policy SOURCE_POLICY,
    concepts::source_supply SUPPLY,
    typename SOURCE_POWER,
    drp_timing const& TIMING = default_drp_timing,
    drp_preference PREFERENCE = drp_preference::none,
    typename... OBSERVERs>
class PdDrp {
public:
    PdDrp(
        CC& cc,
        PHY& phy,
        VBUS& vbus,
        pd_drp_timers<TIMER>& timers,
        std::span<sink_capability const> sink_capabilities,
        SINK_POLICY& sink_policy,
        SINK_POWER& sink_power,
        std::span<std::uint32_t const> source_capabilities,
        SOURCE_POLICY& source_policy,
        SUPPLY& supply,
        SOURCE_POWER& source_power,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : connection_directions_{*this}, engine_directions_{*this},
          prl_(phy, timers.prl, observers...),
          vconn_(timers.vconn, engine_directions_, observers...), sink_engine_(
                                                                      prl_,
                                                                      timers.sink_pe,
                                                                      timers.sink_swap,
                                                                      sink_capabilities,
                                                                      sink_policy,
                                                                      sink_power,
                                                                      vconn_,
                                                                      connection_directions_,
                                                                      observers...
                                                                  ),
          source_engine_(
              cc,
              prl_,
              timers.source_pe,
              timers.source_swap,
              source_capabilities,
              advertisement,
              source_policy,
              supply,
              source_power,
              vconn_,
              connection_directions_,
              observers...
          ),
          drp_(
              cc,
              vbus,
              timers.tc,
              timers.cc,
              advertisement,
              prl_,
              vconn_,
              sink_engine_,
              source_engine_,
              observers...
          )
    {
        // a DRP answers Get_Source_Cap/Get_Sink_Cap in either role
        sink_engine_.provideSourceCapabilities(source_capabilities);
        source_engine_.provideSinkCapabilities(sink_capabilities);
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    PdDrp(
        CC& cc,
        PHY& phy,
        VBUS& vbus,
        pd_drp_timers<TIMER>& timers,
        std::span<sink_capability const> sink_capabilities,
        SINK_POLICY& sink_policy,
        SINK_POWER& sink_power,
        std::span<std::uint32_t const> source_capabilities,
        SOURCE_POLICY& source_policy,
        SUPPLY& supply,
        SOURCE_POWER& source_power,
        OBSERVERs&... observers
    )
        : PdDrp(
              cc,
              phy,
              vbus,
              timers,
              sink_capabilities,
              sink_policy,
              sink_power,
              source_capabilities,
              source_policy,
              supply,
              source_power,
              rp_value::usb_default,
              observers...
          )
    {
    }

    // Go live: toggle Rd/Rp and resolve the roles with the partner
    void start() { drp_.start(); }

    // Sends a PR_Swap to the partner: the application's decision (the
    // partner's request is the injected policy's, check(pe::
    // pr_swap_allowed)); the engines run the message exchange and the
    // VBUS hand-off from there. False when not attached with an
    // explicit contract, a swap is already running, the role lock
    // refuses sourcing, or the port has no PR_Swap feature
    bool swapPowerRole()
    {
        if constexpr (!pr_swap_enabled) {
            return false;
        } else {
            auto const role = drp_.powerRole();
            if (!role) {
                return false;
            }
            if (*role == power_role::sink) {
                return drp_.sourcingAllowed() && sink_engine_.requestPowerSwap();
            }
            return source_engine_.requestPowerSwap();
        }
    }

    // Sends a DR_Swap, likewise the application's decision; on the
    // partner's Accept both sides flip and the header follows
    bool swapDataRole()
    {
        if constexpr (!dr_swap_enabled) {
            return false;
        }
        return onLiveEngine([](auto& engine) { return engine.requestDataSwap(); });
    }

    // Sends a VCONN_Swap: the application's decision (the partner's
    // request is the enabling observer's, check(pe::vconn_swap_allowed));
    // refused by a port without the feature
    bool swapVconnRole()
    {
        if constexpr (!vconn_enabled) {
            return false;
        } else {
            return onLiveEngine([](auto& engine) { return engine.requestVconnSwap(); });
        }
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
    // above 3 A; without an enabler the vconn machine runs vconn_off
    // alone
    static constexpr bool vconn_enabled = feature_enabled<pe::vconn_feature>;

    // The observer answering the VCONN question is also the switch
    // hardware's connector - hold it to the feature's contract right
    // where the answer is honored
    static_assert(
        ((!fsm::observer_answers_for_v<pe::vconn_feature, std::remove_cvref_t<OBSERVERs>> ||
          concepts::vconn_port<std::remove_cvref_t<OBSERVERs>>) &&
         ...),
        "an observer answering pe::vconn_swap_allowed must satisfy concepts::vconn_port "
        "(notifyEntry(vconn::vconn_switch) as well)"
    );

    // The live engine - the layer's client - takes the action; false
    // while neither is live
    template<typename ACTION>
    bool onLiveEngine(ACTION action)
    {
        if (sink_engine_.isLive()) {
            return action(sink_engine_);
        }
        if (source_engine_.isLive()) {
            return action(source_engine_);
        }
        return false;
    }

    // The engines' directions to the connection layer, executed on it
    // from the engines' machines: the Type-C terminations flip at the
    // spec's Assert_Rd/Assert_Rp moments, the attach is held through a
    // hard reset's VBUS cycle, an exhausted HardResetCounter removes
    // the terminations. A forwarding object, because the two machines'
    // types are made of their observers' types: the connection machine
    // lists the engines, so an engine's machine cannot list the
    // connection port in turn
    struct connection_directions : fsm::observing<connection_directions> {
        explicit connection_directions(PdDrp& port_ref) : port(port_ref) {}

        // the swap is agreed: the standby suspends detach detection
        // while the old source collapses VBUS, and its tPSSourceOff
        // timeout restarts resolution if the PS_RDY never comes; the
        // sink engine stays live for it
        void notifyEntry(tc::enter_swap_standby standby)
        {
            if (standby.role == power_role::source) {
                port.drp_.beginSwapToSource();
            }
        }
        // Assert_Rp completes into the source role (the source engine
        // takes over, drives VBUS and answers with PS_RDY); Assert_Rd
        // holds the standby whose tPSSourceOn guards the partner's
        // PS_RDY
        void notifyEntry(tc::assert_new_role asserted)
        {
            if (asserted.role == power_role::source) {
                port.drp_.completeSwap();
            } else {
                port.drp_.beginSwapToSink();
            }
        }
        void notifyEntry(tc::swap_completed) { port.drp_.completeSwap(); }
        // an agreed DR_Swap flips the connection's data role (the
        // header follows through the engine's layer port)
        void notifyEntry(pe::data_role_changed) { port.drp_.applyDataRoleSwap(); }
        // terminations removed for tErrorRecovery, resolution
        // restarts; the teardown resets the engines
        void notifyEntry(tc::request_error_recovery) { port.drp_.errorRecovery(); }
        // the attach is held while VBUS legitimately cycles; the sink
        // engine stays live to await the capabilities
        void notifyEntry(tc::hard_reset_window) { port.drp_.hardResetWindow(); }

        PdDrp& port;
    };

    // The vconn machine's directions to the live engine, executed from
    // the vconn machine; the same forwarding for the same reason (the
    // engines' machines list the vconn machine)
    struct engine_directions : fsm::observing<engine_directions> {
        explicit engine_directions(PdDrp& port_ref) : port(port_ref) {}

        // the switch is on: the live engine announces it with PS_RDY
        void notifyEntry(pe::announce_vconn_on)
        {
            port.onLiveEngine([](auto& engine) { return engine.sendVconnPsRdy(); });
        }
        // a VCONN hand-off timed out: escalate, VCONN stays here (the
        // vconn machine hears the hard reset from the engine)
        void notifyEntry(pe::request_hard_reset)
        {
            port.onLiveEngine([](auto& engine) { return engine.hardReset(); });
        }

        PdDrp& port;
    };

    using Layer = ProtocolLayer<PHY, TIMER, OBSERVERs...>;
    using Vconn = VconnMachine<TIMER, engine_directions, OBSERVERs...>;
    using SinkEngine = SinkPolicyEngine<
        Layer,
        TIMER,
        SINK_POLICY,
        SINK_POWER,
        Vconn,
        connection_directions,
        OBSERVERs...>;
    using SourceEngine = SourcePolicyEngine<
        CC,
        Layer,
        TIMER,
        SOURCE_POLICY,
        SUPPLY,
        SOURCE_POWER,
        Vconn,
        connection_directions,
        OBSERVERs...>;
    // The connection machine's observers in order: the layer holds the
    // PHY and the CC line before any engine transmits, VCONN switches
    // before the source engine's first message (switching VCONN
    // reconfigures the CC lines, which aborts a transmission in
    // flight), the engines take turns on the layer
    using Drp = TypeCDrp<
        CC,
        VBUS,
        TIMER,
        TIMING,
        PREFERENCE,
        Layer,
        Vconn,
        SinkEngine,
        SourceEngine,
        OBSERVERs...>;

    connection_directions connection_directions_;
    engine_directions engine_directions_;
    Layer prl_;
    Vconn vconn_;
    SinkEngine sink_engine_;
    SourceEngine source_engine_;
    Drp drp_;
};

} // namespace usbc
