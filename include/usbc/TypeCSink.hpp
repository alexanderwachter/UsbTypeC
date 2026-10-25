/*
 * USB Type-C sink connection layer: Unattached.SNK, AttachWait.SNK and
 * Attached.SNK per the USB Type-C Cable and Connector Specification,
 * driving a CC and a VBUS driver. No PD involved - the layer detects
 * attach/detach, resolves plug orientation and the Rp current
 * advertisement, and switches the sink power path.
 *
 * The CC pins are debounced outside the machine: every state annotates
 * its cc_debounce rule, the CcWatcher (TypeC.hpp) applies it, and the
 * machine hears event::cc_detected with the status that held. The rows
 * decide on the event's payload. Unattached.SNK detects at once and
 * enters AttachWait.SNK when Rp appears on at least one pin
 * (4.5.2.2.3.2); AttachWait.SNK hears a single Rp after tCCDebounce and
 * the open line after tPDDebounce (4.5.2.2.4.2), and leaves for the
 * table's open exit on the latter. Rp on both pins is a debug accessory
 * this layer does not support: no row takes it, the port keeps waiting
 * for the line to change. A detected single Rp attaches as soon as VBUS
 * is present - at once, asking the driver, or later on the VBUS report
 * attach_wait_snk_rp_detected arms; that state keeps the detected
 * status in the attachment for the attach to resolve plug and
 * advertisement from.
 * Attached.SNK watches vSinkDisconnect instead of vSafe5V; the watched
 * level is a state annotation the vbus driver observes and re-arms on.
 *
 * Integration: hand over initialized drivers - the CC block (Cc.hpp)
 * and the VBUS block (Vbus.hpp), both observers of this machine
 * applying the states' terminations, power paths and watched levels
 * themselves - and two caller-owned timer policy instances, the
 * connection states' and the CC debounce's. Construction rests in the
 * spec's Disabled state (open terminations, nothing monitored); start()
 * is the go-live moment - it registers the drivers' callbacks, fires
 * the started event, whose transition applies the Unattached.SNK
 * terminations, and hands the watcher the line as it is. The drivers
 * must invoke their callbacks from the stack's serialized context (the
 * Zephyr adapters deliver on a workqueue), and the timers are
 * serialized with them by the integrator (mtl timer contract). Attach
 * results reach injected observers watching the attached state's
 * attachedInfo(); their notifications may originate from the timer
 * context on debounce paths.
 *
 * DRP and Try.SNK/Try.SRC build on this layer's states in
 * TypeCDrp.hpp. Not covered yet: debug and audio accessories, Rp
 * change notification while attached, and VCONN (a sink without cable
 * communication needs none).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/TypeC.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <tuple>
#include <type_traits>

namespace usbc {

namespace tc {

constexpr bool singleRp(cc_status status)
{
    return isRp(status.cc1) != isRp(status.cc2);
}

constexpr plug_orientation orientationOf(cc_status status)
{
    return isRp(status.cc1) ? plug_orientation::cc1 : plug_orientation::cc2;
}

constexpr rp_value advertisementOf(cc_status status)
{
    switch (isRp(status.cc1) ? status.cc1 : status.cc2) {
    case cc_state::snk_power_1a5: return rp_value::p_1a5;
    case cc_state::snk_power_3a0: return rp_value::p_3a0;
    default: return rp_value::usb_default;
    }
}

// Delivered to the client on attach, observed on the attached state
struct attach_info {
    plug_orientation orientation;
    rp_value advertisement;
};

namespace state {

// The spec's Disabled state: the port is not operating, terminations
// removed, nothing monitored. start() fires the started event
struct disabled_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );
};

// The spec's ErrorRecovery state: both terminations removed for at
// least tErrorRecovery, then connection resolution restarts from the
// table's unattached anchor. Entered on the PD layer's command (e.g.
// nHardResetCount exhausted)
struct error_recovery {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::open},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );

    static constexpr auto timeout = t_error_recovery;
};

// The resting state resets the connection's attachment: the next
// attach resolves afresh. A source connection is detected on CC alone,
// at once
struct unattached_snk {
    explicit unattached_snk(attachment& attached) { attached = {}; }

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::at_once, .open = debounce_time::at_once}
    );

    using contexts = fsm::contexts<attachment>;
};

// Debouncing: a single Rp for tCCDebounce, the open line for
// tPDDebounce; the detection asks the driver whether VBUS is present
struct attach_wait_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::t_cc_debounce, .open = debounce_time::t_pd_debounce}
    );
};

// AttachWait.SNK with a single Rp detected, waiting for VBUS; the
// detected status is the attach's. Entered with the detection - the
// plain form is the one the machine asks of every state
struct attach_wait_snk_rp_detected {
    attach_wait_snk_rp_detected(event::cc_detected const& event, attachment& attached)
    {
        attached.detected = event.cc;
    }
    explicit attach_wait_snk_rp_detected(attachment&) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        cc_debounce{.termination = debounce_time::t_cc_debounce, .open = debounce_time::t_pd_debounce}
    );

    using contexts = fsm::contexts<attachment>;
};

struct attached_snk {
    // a fresh attach on the status the attachment holds
    // (attach_wait_snk_rp_detected's, or TryWait.SNK's)
    explicit attached_snk(attachment& attached_ref) : attached(attached_ref) { attachFresh(); }
    // a fresh attach on the detection itself
    attached_snk(event::cc_detected const& event, attachment& attached_ref) : attached(attached_ref)
    {
        attached.detected = event.cc;
        attachFresh();
    }
    // entered on the VBUS report: a fresh attach (from
    // attach_wait_snk_rp_detected) resolves plug and data role; VBUS
    // returning after a hard reset
    // resumes the connection unchanged
    attached_snk(event::vbus_present const&, attachment& attached_ref) : attached(attached_ref)
    {
        if (attached.resolved) {
            origin_ = attach_origin::resumed_after_hard_reset;
            advertisement_ = advertisementOf(attached.detected);
        } else {
            attachFresh();
        }
    }
    // entered from a PD-directed role swap's standby (the DRP layer),
    // completed or aborted: the plug stays put - the attachment is
    // already resolved - and the current draw is governed by the PD
    // contract, not the Rp advertisement
    attached_snk(event::swap_complete const&, attachment& attached_ref)
        : attached(attached_ref), origin_(attach_origin::completed_power_role_swap)
    {
    }
    attached_snk(event::swap_abort const&, attachment& attached_ref) : attached(attached_ref) {}

    // sinking: detach detection is VBUS-based, the CC pins are not
    // monitored
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::sink},
        vbus_level{vbus_level::sink_disconnect},
        attached_role{power_role::sink},
        pd_connection{}
    );

    // the attach result, the plug orientation and the partner, observed
    // as instance values: the clients and loggers consume the attach
    // info, the PD layer above selects the PHY's CC line from the
    // orientation and acts on the partner (what brought the port here,
    // data role, the source's Rp)
    auto values() const
    {
        return fsm::annotate(
            attach_info{.orientation = attached.orientation, .advertisement = advertisement_},
            attached.orientation,
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
    // takes the sink's default data role
    void attachFresh()
    {
        attached.orientation = orientationOf(attached.detected);
        attached.data = data_role::ufp;
        attached.resolved = true;
        advertisement_ = advertisementOf(attached.detected);
    }

    rp_value advertisement_ = rp_value::usb_default;
    attach_origin origin_ = attach_origin::fresh_attach;
};

// The hard-reset window, first phase: the source legitimately drops
// VBUS to vSafe0V - not a detach. Rd stays presented, the sink path
// is off, the PD connection holds (the engine awaits the source's
// capabilities), and VBUS going away is watched at the same threshold
// Attached.SNK detects a detach with. The policy engine's
// NoResponseTimer owns the give-up; the timeout here only terminates a
// dead port
struct hard_reset_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::sink_disconnect},
        pd_connection{}
    );

    static constexpr auto timeout = t_hard_reset_window;
};

// ... second phase: VBUS is down, its return resumes Attached.SNK
struct hard_reset_recover_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        pd_connection{}
    );

    static constexpr auto timeout = t_hard_reset_window;
};

} // namespace state

// Guards on the detected status
struct source_present {
    static bool check(auto const&, event::cc_detected const& event)
    {
        return sourcePresent(event.cc);
    }
};

struct single_rp {
    static bool check(auto const&, event::cc_detected const& event) { return singleRp(event.cc); }
};

// The sink attach flow, shared with the DRP layer: UNATTACHED anchors
// it (the sink's resting state, or the DRP's toggling Rd phase) and
// ATTACH is where a successful attach leads (Attached.SNK, or Try.SRC
// for a source-preferring DRP). AttachWait.SNK's open exit leads back
// to UNATTACHED; a DRP lists its own exit (Unattached.SRC) ahead
template<fsm::concepts::state UNATTACHED, fsm::concepts::state ATTACH>
using sink_attach_flow = fsm::transition_table<
    fsm::transition<fsm::from<UNATTACHED>, fsm::on<event::cc_detected>, fsm::guard<source_present>, fsm::to<state::attach_wait_snk>>,
    // a single Rp detected attaches with VBUS present, else waits for
    // it; the open line leaves; Rp on both pins waits for the line to
    // change
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<single_rp, vbus_at_safe5v>, fsm::to<ATTACH>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<single_rp>, fsm::to<state::attach_wait_snk_rp_detected>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<UNATTACHED>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::vbus_present>, fsm::to<ATTACH>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::guard<single_rp>, fsm::to<state::attach_wait_snk_rp_detected>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<UNATTACHED>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::to<state::attach_wait_snk>>,
    // sink detach detection is VBUS-based
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::vbus_removed>, fsm::to<UNATTACHED>>>;

// Separate entry: the DRP shares the state, standalone source tables
// do not - the bidirectional map check rejects entries for absent states
using error_recovery_timer_range =
    fsm::timer_ranges<fsm::timed_by<state::error_recovery, spec::t_error_recovery>>;

using hard_reset_timer_ranges = fsm::timer_ranges<
    fsm::timed_by<state::hard_reset_snk, spec::t_hard_reset_window>,
    fsm::timed_by<state::hard_reset_recover_snk, spec::t_hard_reset_window>>;

// The hard-reset window, shared with the DRP: attach held while VBUS
// legitimately cycles through vSafe0V; a window that never completes
// falls back to the table's unattached anchor
template<fsm::concepts::state UNATTACHED>
using hard_reset_flow = fsm::transition_table<
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::hard_reset>, fsm::to<state::hard_reset_snk>>,
    fsm::transition<fsm::from<state::hard_reset_snk>, fsm::on<event::vbus_removed>, fsm::to<state::hard_reset_recover_snk>>,
    fsm::transition<fsm::from<state::hard_reset_snk>, fsm::on<fsm::timeout>, fsm::to<UNATTACHED>>,
    fsm::transition<fsm::from<state::hard_reset_recover_snk>, fsm::on<event::vbus_present>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<state::hard_reset_recover_snk>, fsm::on<fsm::timeout>, fsm::to<UNATTACHED>>>;

// ErrorRecovery is anchored per table: open terminations, then back
// to that table's unattached resting state
template<fsm::concepts::state UNATTACHED>
using error_recovery_flow = fsm::transition_table<
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::error_recovery>, fsm::to<state::error_recovery>>,
    fsm::transition<fsm::from<state::error_recovery>, fsm::on<fsm::timeout>, fsm::to<UNATTACHED>>>;

using sink_transitions = fsm::transition_table<
    fsm::initial<state::disabled_snk>,
    fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>, fsm::to<state::unattached_snk>>,
    sink_attach_flow<state::unattached_snk, state::attached_snk>,
    error_recovery_flow<state::unattached_snk>,
    hard_reset_flow<state::unattached_snk>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol and is the machine id in
// trace lines and DOT graphs - unique across the stack
struct tc_sink_table : sink_transitions {};
// timeout bounds and reachability checked in test/compliance.cpp

} // namespace tc

template<concepts::cc CC, concepts::vbus VBUS, fsm::concepts::timer TIMER, typename... OBSERVERs>
class TypeCSink : public tc::port_frontend<TypeCSink<CC, VBUS, TIMER, OBSERVERs...>, CC, VBUS> {
    static_assert(
        tc::hardware_annotations_complete_v<tc::tc_sink_table>,
        "every state must annotate its CC termination, VBUS power path and watched VBUS level"
    );

public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts,
    // cc_timer the CC debounce. The drivers observe the machine, the
    // observers are injected after the built-in ones (timer, the two
    // drivers, cc watcher); attach results are observed on the attached
    // state's attach_info
    TypeCSink(CC& cc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : cc_driver_(cc), vbus_driver_(vbus), cc_(cc_timer), timer_(timer), timed_(timer_),
          observers_(observers...), sm_(timed_, cc, vbus, cc_, observers...)
    {
    }

    // The go-live moment, provided by the shared frontend; a sink
    // presents no Rp
    void start() { this->startPort(rp_value::usb_default); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts. Call from the stack's
    // serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCSink, CC, VBUS>;

    CC& cc_driver_;
    VBUS& vbus_driver_;
    tc::CcWatcher<TIMER> cc_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        tc::tc_sink_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        CC,
        VBUS,
        tc::CcWatcher<TIMER>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
