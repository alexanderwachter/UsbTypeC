/*
 * USB Type-C source connection layer: Unattached.SRC, AttachWait.SRC,
 * Attached.SRC and UnattachedWait.SRC per the USB Type-C Cable and
 * Connector Specification. The port presents Rp with the configured
 * advertisement; the CC pins are debounced outside the machine by the
 * CcWatcher (TypeC.hpp) applying each state's cc_debounce rule, and
 * the machine hears event::cc_detected with the status that held.
 * Unattached.SRC detects at once and enters AttachWait.SRC when a sink
 * or an accessory appears - Rd on either pin, or Ra on both
 * (4.5.2.2.7.2; Ra on one pin alone is a cable without a device).
 * AttachWait.SRC hears a single Rd after tCCDebounce, with a load on
 * VBUS so it reaches vSafe0V (4.5.2.2.8.1), and requires VBUS at
 * vSafe0V before applying power (the detection asks the driver;
 * attach_wait_src_rd_detected waits for the report when it is not there
 * yet, keeping the detected status in the attachment for the attach);
 * it hears the open line at once and leaves for the table's open exit
 * - both pins open, or open and Ra (4.5.2.2.8.2, "within
 * tSRCDisconnect, as quickly as possible"). Rd on both pins or Ra on
 * both is an accessory this layer does not support: no row takes it,
 * the port keeps waiting for the line to change. Attached.SRC sources
 * VBUS and hears the Rd removed at once; UnattachedWait.SRC discharges
 * to vSafe0V without monitoring the pins - the driver's arm-report
 * exits it right away when VBUS already sits there, and Unattached.SRC
 * then detects a sink that arrived meanwhile.
 *
 * Integration mirrors TypeCSink: initialized drivers plus two
 * caller-owned timer policy instances, the connection states' and the
 * CC debounce's. Construction rests in the spec's Disabled state (open
 * terminations, nothing monitored); start() is the go-live moment - it
 * fires the started event, whose transition presents Rp, then registers
 * the callbacks and hands the watcher the line as it is. Attach results
 * reach injected observers watching the attached state's
 * attachedInfo(); their notifications may originate from the timer
 * context on debounce paths.
 *
 * DRP and Try.SRC/Try.SNK build on this layer's states in
 * TypeCDrp.hpp. Not covered yet: debug and audio accessories, and
 * VCONN (the PD layer's VCONN machine drives the switch).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
#include <usbc/TypeC.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <tuple>

namespace usbc {

namespace tc {

constexpr bool singleRd(cc_status status)
{
    return isRd(status.cc1) != isRd(status.cc2);
}

constexpr plug_orientation srcOrientationOf(cc_status status)
{
    return isRd(status.cc1) ? plug_orientation::cc1 : plug_orientation::cc2;
}

namespace state {

// The spec's Disabled state: the port is not operating, terminations
// removed, nothing monitored. start() fires the started event
struct disabled_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );
};

// The spec's ErrorRecovery state, source flavor: both terminations
// removed for at least tErrorRecovery, then connection resolution
// restarts. Entered on the PD layer's command
struct error_recovery_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );

    static constexpr auto timeout = t_error_recovery;
};

// The resting state resets the connection's attachment: the next
// attach resolves afresh. A sink is detected on CC alone, at once
struct unattached_src {
    explicit unattached_src(attachment& attached) { attached = {}; }

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::at_once, .open = debounce_time::at_once}
    );

    using contexts = fsm::contexts<attachment>;
};

// Debouncing a single Rd for tCCDebounce with a load on VBUS so it
// reaches vSafe0V within tVsafe0V (4.5.2.2.8.1); the open line is
// heard at once. The detection asks the driver whether VBUS is at
// vSafe0V
struct attach_wait_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::safe0v},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::t_cc_debounce, .open = debounce_time::at_once}
    );
};

// AttachWait.SRC with a single Rd detected, waiting for VBUS at vSafe0V
// (the sink's capacitors may hold charge: the load stays applied); the
// detected status is the attach's. Entered with the detection - the
// plain form is the one the machine asks of every state
struct attach_wait_src_rd_detected {
    attach_wait_src_rd_detected(event::cc_detected const& event, attachment& attached)
    {
        attached.detected = event.cc;
    }
    explicit attach_wait_src_rd_detected(attachment&) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::safe0v},
        vbus_level{vbus_level::safe0v},
        cc_debounce{.termination = debounce_time::t_cc_debounce, .open = debounce_time::at_once}
    );

    using contexts = fsm::contexts<attachment>;
};

struct attached_src {
    // a fresh attach on the status the attachment holds
    // (attach_wait_src_rd_detected's) - the plain form the machine asks
    // of every state
    explicit attached_src(attachment& attached_ref) : attached(attached_ref) { attachFresh(); }
    // a fresh attach on the detection itself
    attached_src(event::cc_detected const& event, attachment& attached_ref) : attached(attached_ref)
    {
        attached.detected = event.cc;
        attachFresh();
    }
    // entered on the vSafe0V report after the detection
    attached_src(event::vbus_reached_safe0v const&, attachment& attached_ref)
        : attached(attached_ref)
    {
        attachFresh();
    }
    // entered from a PD-directed role swap's standby (the DRP layer),
    // completed or aborted: the plug stays put - the attachment is
    // already resolved
    attached_src(event::swap_complete const&, attachment& attached_ref)
        : attached(attached_ref), origin_(attach_origin::completed_power_role_swap)
    {
    }
    attached_src(event::swap_abort const&, attachment& attached_ref) : attached(attached_ref) {}

    // sourcing: detach detection is CC-based, within tSRCDisconnect -
    // a dead-battery sink's Rd may gap while it hands over from its
    // port protector to its own pull-down; the comparator rests
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::safe5v},
        vbus_level{vbus_level::unwatched},
        attached_role{power_role::source},
        pd_connection{},
        cc_debounce{.termination = debounce_time::at_once, .open = debounce_time::t_src_disconnect}
    );

    // the attach report, the CC polarity and the partner, observed as
    // instance values: clients consume the orientation (a source
    // reports no more), the hw driver applies the polarity, the PD
    // layer above acts on the partner (what brought the port here,
    // data role, the cable's Ra)
    auto values() const
    {
        return fsm::annotate(
            attached.orientation,
            polarity{.orientation = attached.orientation},
            attached_partner{.origin = origin_, .data = attached.data, .cc = attached.detected}
        );
    }

    using contexts = fsm::contexts<attachment>;

    // a DR_Swap flips the data role in place (DRP only)
    void handle(event::swap_data_role const&) { attached.data = otherDataRole(attached.data); }

    plug_orientation orientation() const { return attached.orientation; }
    data_role dataRole() const { return attached.data; }
    attachment& attached;

private:
    // a fresh attach resolves the plug from the detected status and
    // takes the source's default data role
    void attachFresh()
    {
        attached.orientation = srcOrientationOf(attached.detected);
        attached.data = data_role::dfp;
        attached.resolved = true;
    }

    attach_origin origin_ = attach_origin::fresh_attach;
};

// Discharges VBUS to vSafe0V before presenting Rp for a new attach;
// the pins are not monitored meanwhile - Unattached.SRC detects a sink
// that arrived during the discharge
struct unattached_wait_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::safe0v},
        vbus_level{vbus_level::safe0v}
    );
};

} // namespace state

// Guards on the detected status
struct single_rd {
    static bool check(auto const&, event::cc_detected const& event) { return singleRd(event.cc); }
};

struct rd_removed {
    static bool check(auto const&, event::cc_detected const& event) { return !singleRd(event.cc); }
};

struct sink_present {
    static bool check(auto const&, event::cc_detected const& event)
    {
        return sinkPresent(event.cc);
    }
};

// The source attach flow, shared with the DRP layer: UNATTACHED
// anchors it (the source's resting state, or the DRP's toggling Rp
// phase), ATTACH is where a successful attach leads (Attached.SRC, or
// Try.SNK for a sink-preferring DRP) and OPEN_EXIT is where
// AttachWait.SRC leaves when the sink is gone (Unattached.SRC for a
// source, Unattached.SNK for a DRP)
template<
    fsm::concepts::state UNATTACHED,
    fsm::concepts::state ATTACH,
    fsm::concepts::state OPEN_EXIT>
using source_attach_flow = fsm::transition_table<
    fsm::transition<fsm::from<UNATTACHED>, fsm::on<event::cc_detected>, fsm::guard<sink_present>, fsm::to<state::attach_wait_src>>,
    // the sink is gone: leave at once; a single Rd attaches at vSafe0V
    // or waits for it; an accessory waits for the line to change
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<sink_present>>, fsm::to<OPEN_EXIT>>,
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<event::cc_detected>, fsm::guard<single_rd, vbus_at_safe0v>, fsm::to<ATTACH>>,
    fsm::transition<fsm::from<state::attach_wait_src>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<state::attach_wait_src_rd_detected>>,
    fsm::transition<fsm::from<state::attach_wait_src_rd_detected>, fsm::on<event::vbus_reached_safe0v>, fsm::to<ATTACH>>,
    fsm::transition<fsm::from<state::attach_wait_src_rd_detected>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<sink_present>>, fsm::to<OPEN_EXIT>>,
    fsm::transition<fsm::from<state::attach_wait_src_rd_detected>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<state::attach_wait_src_rd_detected>>,
    fsm::transition<fsm::from<state::attach_wait_src_rd_detected>, fsm::on<event::cc_detected>, fsm::to<state::attach_wait_src>>>;

using source_transitions = fsm::transition_table<
    fsm::initial<state::disabled_src>,
    fsm::transition<fsm::from<state::disabled_src>, fsm::on<event::started>, fsm::to<state::unattached_src>>,
    source_attach_flow<state::unattached_src, state::attached_src, state::unattached_src>,
    // the Rd removed: Rp waits again after UnattachedWait.SRC - whose
    // entry arms vSafe0V, and the driver's arm-report exits it right
    // away when the line is already there
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::cc_detected>, fsm::guard<rd_removed>, fsm::to<state::unattached_wait_src>>,
    fsm::transition<fsm::from<state::unattached_wait_src>, fsm::on<event::vbus_reached_safe0v>, fsm::to<state::unattached_src>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::error_recovery>, fsm::to<state::error_recovery_src>>,
    fsm::transition<fsm::from<state::error_recovery_src>, fsm::on<fsm::timeout>, fsm::to<state::unattached_src>>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol and is the machine id in
// trace lines and DOT graphs - unique across the stack
struct tc_source_table : source_transitions {};

// error_recovery_src is not part of a shared map: the DRP composes its
// map without carrying this table's private state
using source_recovery_timer_range =
    fsm::timer_ranges<fsm::timed_by<state::error_recovery_src, spec::t_error_recovery>>;

// timeout bounds and reachability checked in test/compliance.cpp

// Applies each state's src_hw annotation (suppressed while unchanged)
// with the port's configured Rp advertisement, and the attached
// state's plug orientation. Discharge belongs to the vbus driver
template<concepts::tcpc TCPC, concepts::vbus VBUS>
struct src_hw_driver : fsm::observing<src_hw_driver<TCPC, VBUS>> {
    src_hw_driver(TCPC& tcpc_ref, VBUS& vbus_ref, rp_value advertisement)
        : tcpc(tcpc_ref), vbus(vbus_ref), rp(advertisement)
    {
    }

    // A state the dispatch would silently skip is a table bug: the
    // previous state's terminations or power paths would stay applied.
    // Checked per element: every state must carry both
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(
            fsm::all_states_carry_v<TABLE, cc_termination>,
            "src_hw_driver: every state must annotate its CC termination"
        );
        static_assert(
            fsm::all_states_carry_v<TABLE, vbus_power>,
            "src_hw_driver: every state must annotate its VBUS power path"
        );
    }

    void notifyEntry(cc_termination termination) { tcpc.setCc(termination.pull, rp); }
    void notifyEntry(vbus_power power)
    {
        tcpc.sourceVbus(power.path == vbus_path::safe5v);
        vbus.discharge(power.path == vbus_path::safe0v);
    }
    void notifyEntry(polarity resolved) { tcpc.setPlugOrientation(resolved.orientation); }

    TCPC& tcpc;
    VBUS& vbus;
    rp_value rp;
};

} // namespace tc

template<
    concepts::tcpc TCPC,
    concepts::vbus VBUS,
    fsm::concepts::timer TIMER,
    typename... OBSERVERs>
class TypeCSource
    : public tc::port_frontend<TypeCSource<TCPC, VBUS, TIMER, OBSERVERs...>, TCPC, VBUS> {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts,
    // cc_timer the CC debounce. The advertisement is the Rp the port
    // presents (and must honor). The observers are injected into the
    // machine after the built-in ones (timer, hw driver, vbus watcher -
    // which also answers the tables' questions about VBUS now - and
    // cc watcher); attach results are observed on the attached state's
    // attachedInfo() (a source reports the orientation only), and an
    // observer providing onPdAlert(alert_status) receives the alert
    // bits this layer does not consume - the hook for the PD layers
    // above
    TypeCSource(
        TCPC& tcpc,
        VBUS& vbus,
        TIMER& timer,
        TIMER& cc_timer,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : tcpc_(tcpc), hw_(tcpc, vbus, advertisement), vbus_(vbus), cc_(cc_timer), timer_(timer),
          timed_(timer_), observers_(observers...), sm_(timed_, hw_, vbus_, cc_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    TypeCSource(TCPC& tcpc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : TypeCSource(tcpc, vbus, timer, cc_timer, rp_value::usb_default, observers...)
    {
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts. Call from the stack's
    // serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

private:
    friend tc::port_frontend<TypeCSource, TCPC, VBUS>;

    TCPC& tcpc_;
    tc::src_hw_driver<TCPC, VBUS> hw_;
    tc::VbusWatcher<VBUS> vbus_;
    tc::CcWatcher<TIMER> cc_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        tc::tc_source_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        tc::src_hw_driver<TCPC, VBUS>,
        tc::VbusWatcher<VBUS>,
        tc::CcWatcher<TIMER>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
