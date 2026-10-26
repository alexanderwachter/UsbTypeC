/*
 * USB Type-C sink connection layer: Unattached.SNK, AttachWait.SNK and
 * Attached.SNK per the USB Type-C Cable and Connector Specification,
 * driving a CC and a VBUS driver. No PD involved - the layer detects
 * attach/detach, resolves plug orientation and the Rp current
 * advertisement, and switches the sink power path.
 *
 * The CC pins are debounced outside the machine: every state annotates
 * its cc_debounce rule, the CcWatcher (layer/TypeC.hpp) applies it,
 * and the machine hears event::cc_detected with the status that held. The rows
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
 * The port running this table is usbc::TypeCSink (usbc/TypeCSink.hpp).
 * DRP and Try.SNK/Try.SRC build on this layer's states in
 * TypeCDrp.hpp. Not covered yet: debug and audio accessories, Rp
 * change notification while attached, and VCONN (a sink without cable
 * communication needs none).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/spec/Parameters.hpp>
#include <usbc/state/TypeC.hpp>

#include <mtl/StateMachine.hpp>

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

    // the attach result, the plug orientation, the data role and the
    // attach itself, observed as instance values: the clients and
    // loggers consume the attach info, an application's mux the
    // orientation and the data role, the protocol layer selects the
    // PHY's CC line from the orientation, the sink engine goes live on
    // the attach (what brought the port here, the source's Rp)
    auto values() const
    {
        return fsm::annotate(
            attach_info{.orientation = attached.orientation, .advertisement = advertisement_},
            attached.orientation,
            attached_data_role{attached.data},
            attached_as_sink{.origin = origin_, .cc = attached.detected}
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

} // namespace usbc
