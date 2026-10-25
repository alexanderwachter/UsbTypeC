/*
 * USB Type-C dual-role port (DRP) connection layer per the USB Type-C
 * Cable and Connector Specification. The port toggles between
 * Unattached.SNK (presenting Rd) and Unattached.SRC (presenting Rp)
 * and follows the sink or source attach flow of whichever role it was
 * advertising when a partner appeared - the AttachWait and Attached
 * states are shared with TypeCSink.hpp/TypeCSource.hpp, and so is the
 * CcWatcher (TypeC.hpp) debouncing the CC pins by every state's
 * cc_debounce rule. A sink leaving Attached.SRC puts the DRP at Rd at
 * once while VBUS is removed (removing_vbus), then on to Unattached.SNK
 * - or TryWait.SNK for a source-preferring port (4.5.2.2.9.2).
 *
 * Toggle timing is a compile-time policy: tDRP is the full toggle
 * period and dcSRC.DRP the percentage of it spent advertising Rp, both
 * checked against the spec's Table 4-30 ranges (tDRP 50-100 ms,
 * dcSRC.DRP 30-70 %, which also bounds each role's slice to the
 * table's 15-70 ms pulse width). Derive from default_drp_timing and
 * override members to configure.
 *
 * Role preference (spec 4.5.2.2): drp_preference::source inserts
 * Try.SRC/TryWait.SNK where the sink flow would attach,
 * drp_preference::sink inserts Try.SNK/TryWait.SRC where the source
 * flow would attach. A Try state's time budget is its own timeout - a
 * wall a flapping partner cannot extend - and the wanted termination is
 * debounced by the watcher for tTryCCDebounce:
 *   Try.SRC     = try_src (tDRPTry), try_src_until_safe0v (the rest of
 *                 tTryTimeout while VBUS has not reached vSafe0V) and
 *                 try_src_timed_out (the budget spent while an Rd is
 *                 under debounce)
 *   TryWait.SNK = try_wait_snk (tCCDebounce since entry) and
 *                 try_wait_snk_debounced (VBUS attaches from then on);
 *                 the open line for tPDDebounce ends both
 *   Try.SNK     = try_snk_wait (tDRPTry, the pins not monitored),
 *                 try_snk (an Rp, or no Rp for tTryCCDebounce) and
 *                 try_snk_rp_detected (VBUS attaches)
 *   TryWait.SRC = try_wait_src (tDRPTry), try_wait_src_rd_detected (Rd
 *                 detected, vSafe0V awaited) and try_wait_src_timed_out
 * A wall expiring while the termination is under debounce (the
 * watcher's cc_state_pending) does not abort the debounce - the spec's
 * walls apply while the termination has "not yet been detected" - the
 * timed_out states take the detection's outcome.
 *
 * Role lock (port control, conformance-neutral): an injected object
 * answering bool check(usbc::sourcing_allowed) keeps the port
 * sink-only while it says no - the toggle stays at Rd (re-asking each
 * slice, so lifting the lock resumes toggling within one), a
 * source-preferring port attaches as sink instead of trying Rp, and
 * the PD layer refuses swaps to source. The question's static
 * default is yes: without such an object nothing changes.
 *
 * Open deviations from Release 2.5 are listed in the conformance
 * roadmap (CLAUDE.md).
 *
 * Integration matches TypeCSink/TypeCSource: initialized drivers -
 * the CC and VBUS blocks, observers of this machine - plus two
 * caller-owned timer policy instances; construction rests in the
 * spec's Disabled state and start() goes live in Unattached.SNK. The
 * vbus driver is armed by the states waiting for a report (vSafe5V by
 * the sink-side waits, vSafe0V by the source-side waits,
 * vSinkDisconnect while Attached.SNK), the callback's meaning is
 * mapped through the level it reports on, and a detection asks the
 * driver whether VBUS is there now. Injected observers watching the
 * attached states' instance values learn which role attached through
 * the info type: tc::attach_info (orientation and advertisement) for
 * Attached.SNK only; both roles deliver the plug_orientation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/PortControl.hpp>
#include <usbc/TypeC.hpp>
#include <usbc/TypeCSink.hpp>
#include <usbc/TypeCSource.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <chrono>
#include <concepts>
#include <tuple>

namespace usbc {

// Which role a DRP tries to resolve to when a partner attaches
enum class drp_preference : std::uint8_t { none, source, sink };

// Spec Table 4-30 DRP timing parameters, passed to the port as a
// reference to a constexpr instance (chrono durations are not
// structural types, so the object cannot be a by-value template
// argument). Configure with designated initializers over the member
// defaults; the ranges are enforced at compile time:
//
//   inline constexpr usbc::drp_timing sink_heavy{.t_drp = std::chrono::milliseconds{100},
//                                                .dc_src = 30};
struct drp_timing {
    std::chrono::milliseconds t_drp{75};          // tDRP, 50 - 100 ms
    unsigned dc_src = 50;                         // dcSRC.DRP, 30 - 70 %
    std::chrono::milliseconds t_drp_try{100};     // tDRPTry, 75 - 150 ms
    std::chrono::milliseconds t_try_timeout{800}; // tTryTimeout, 550 - 1100 ms
};

inline constexpr drp_timing default_drp_timing{};

namespace tc {

namespace drp {

template<drp_timing const& TIMING>
consteval bool timingWithinSpec()
{
    static_assert(spec::t_drp.contains(TIMING.t_drp), "tDRP outside the spec's 50-100 ms");
    static_assert(
        spec::within(TIMING.dc_src, spec::dc_src_drp_min, spec::dc_src_drp_max),
        "dcSRC.DRP outside the spec's 30-70 %"
    );
    static_assert(
        spec::t_drp_try.contains(TIMING.t_drp_try),
        "tDRPTry outside the spec's 75-150 ms"
    );
    static_assert(
        spec::t_try_timeout.contains(TIMING.t_try_timeout),
        "tTryTimeout outside the spec's 550-1100 ms"
    );
    return true;
}

// The Rp slice of the toggle period; the Rd slice is the remainder
template<drp_timing const& TIMING>
inline constexpr std::chrono::milliseconds t_src_slice = TIMING.t_drp * TIMING.dc_src / 100;

// tTryTimeout counts from Try.SRC's entry: the spec's range less this
// port's tDRPTry bounds what is left of it for try_src_until_safe0v
template<drp_timing const& TIMING>
inline constexpr spec::timeout_range t_try_timeout_remainder{
    spec::t_try_timeout.min - TIMING.t_drp_try,
    spec::t_try_timeout.max - TIMING.t_drp_try
};

// The CC rules the Try states share
inline constexpr cc_debounce try_rd_debounce{
    .termination = debounce_time::t_try_cc_debounce,
    .open = debounce_time::at_once
};
inline constexpr cc_debounce try_rp_debounce{
    .termination = debounce_time::t_try_cc_debounce,
    .open = debounce_time::t_try_cc_debounce
};
inline constexpr cc_debounce wait_snk_debounce{
    .termination = debounce_time::at_once,
    .open = debounce_time::t_pd_debounce
};

// --- toggling unattached states ---------------------------------------------

// Unattached.SNK of a DRP: Rd presented for the sink slice of tDRP;
// the toggle resets the connection's attachment, the next attach
// resolves afresh
template<drp_timing const& TIMING>
struct unattached_snk {
    explicit unattached_snk(attachment& attached) { attached = {}; }

    // toggling: a partner is detected on CC alone, at once
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::at_once, .open = debounce_time::at_once}
    );

    using contexts = fsm::contexts<attachment>;

    static constexpr auto timeout = TIMING.t_drp - t_src_slice<TIMING>;
};

// Unattached.SRC of a DRP: Rp presented for the source slice of tDRP
template<drp_timing const& TIMING>
struct unattached_src {
    explicit unattached_src(attachment& attached) { attached = {}; }

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        cc_debounce{.termination = debounce_time::at_once, .open = debounce_time::at_once}
    );

    using contexts = fsm::contexts<attachment>;

    static constexpr auto timeout = t_src_slice<TIMING>;
};

// --- Try.SRC / TryWait.SNK (drp_preference::source) --------------------------

// Try.SRC: Rp presented where the sink flow would have attached, for
// tDRPTry; a single Rd detected attaches (4.5.2.2.10.2)
template<drp_timing const& TIMING>
struct try_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        try_rd_debounce
    );

    static constexpr auto timeout = TIMING.t_drp_try;
};

// Try.SRC past tDRPTry while VBUS has not reached vSafe0V: Rp stays
// presented until it does, or until tTryTimeout since Try.SRC's entry
// is up - what is left of it after tDRPTry is this state's own timeout
// (a debounce running at the tDRPTry wall delays the handover by up to
// tTryCCDebounce). vSafe0V reached while an Rd is under debounce waits
// for the detection
template<drp_timing const& TIMING>
struct try_src_until_safe0v {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe0v},
        try_rd_debounce
    );

    static constexpr auto timeout = TIMING.t_try_timeout - TIMING.t_drp_try;

    void handle(event::vbus_reached_safe0v const&) {}
};

// tTryTimeout is up while an Rd is under debounce: its detection
// attaches, anything else leaves for TryWait.SNK
struct try_src_timed_out {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        try_rd_debounce
    );
};

// TryWait.SNK: Rd presented where Try.SRC failed, or where the sink
// left Attached.SRC. Attaches as sink after tCCDebounce once VBUS is
// detected, ends with the open line for tPDDebounce (4.5.2.2.11.2); a
// detected Rp is the attach's status, kept in the attachment
struct try_wait_snk {
    explicit try_wait_snk(attachment& attached_ref) : attached(attached_ref) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        wait_snk_debounce
    );

    using contexts = fsm::contexts<attachment>;

    static constexpr auto timeout = t_cc_debounce;

    void handle(event::cc_detected const& event) { attached.detected = event.cc; }
    // VBUS before the debounce: asked again when it is up
    void handle(event::vbus_present const&) {}

    attachment& attached;
};

// TryWait.SNK with tCCDebounce up: VBUS detected attaches - with an Rp
// to resolve the plug from
struct try_wait_snk_debounced {
    explicit try_wait_snk_debounced(attachment& attached_ref) : attached(attached_ref) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        wait_snk_debounce
    );

    using contexts = fsm::contexts<attachment>;

    void handle(event::cc_detected const& event) { attached.detected = event.cc; }
    // VBUS without an Rp: waited on
    void handle(event::vbus_present const&) {}

    attachment& attached;
};

// --- leaving Attached.SRC ----------------------------------------------------

// The sink left: Rd is presented at once - the spec's Unattached.SNK
// or TryWait.SNK (4.5.2.2.9.2) - while VBUS is removed, discharged to
// vSafe0V within tVBUSOFF. The connection's attachment is reset here
// like in the Unattached states: the next attach resolves afresh. The
// pins are not monitored meanwhile; where the port goes on once VBUS
// is down is the preference's business, and that state detects a
// source that presented Rp meanwhile
struct removing_vbus {
    explicit removing_vbus(attachment& attached) { attached = {}; }

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::safe0v},
        vbus_level{vbus_level::safe0v}
    );

    using contexts = fsm::contexts<attachment>;
};

// --- Try.SNK / TryWait.SRC (drp_preference::sink) ----------------------------

// Try.SNK: Rd presented where the source flow would have attached. The
// spec mandates waiting tDRPTry before the CC pins are even monitored
// (4.5.2.2.12.2)
template<drp_timing const& TIMING>
struct try_snk_wait {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched}
    );

    static constexpr auto timeout = TIMING.t_drp_try;
};

// Try.SNK monitoring: a single Rp for tTryCCDebounce attaches with
// VBUS present, no Rp for tTryCCDebounce leaves for TryWait.SRC
struct try_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        try_rp_debounce
    );
};

// The Rp detected, VBUS not yet present: the attach follows the report
struct try_snk_rp_detected {
    try_snk_rp_detected(event::cc_detected const& event, attachment& attached)
    {
        attached.detected = event.cc;
    }
    explicit try_snk_rp_detected(attachment&) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe5v},
        try_rp_debounce
    );

    using contexts = fsm::contexts<attachment>;
};

// TryWait.SRC: the partner did not present Rp - back to Rp for tDRPTry
// (4.5.2.2.13.2); a single Rd detected attaches at vSafe0V
template<drp_timing const& TIMING>
struct try_wait_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe0v},
        try_rd_debounce
    );

    static constexpr auto timeout = TIMING.t_drp_try;

    void handle(event::vbus_reached_safe0v const&) {}
};

// The Rd detected but VBUS not yet at vSafe0V: the attach follows the
// report; the Rd going away starts the phase over
struct try_wait_src_rd_detected {
    try_wait_src_rd_detected(event::cc_detected const& event, attachment& attached)
    {
        attached.detected = event.cc;
    }
    explicit try_wait_src_rd_detected(attachment&) {}

    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::safe0v},
        try_rd_debounce
    );

    using contexts = fsm::contexts<attachment>;
};

// tDRPTry is up while an Rd is under debounce: its detection attaches,
// anything else resumes toggling
struct try_wait_src_timed_out {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rp},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        try_rd_debounce
    );
};

// --- PR_Swap standby ---------------------------------------------------------

// Operating values for the PS_RDY waits, within the spec ranges
inline constexpr auto t_ps_source_off = std::chrono::milliseconds{835}; // tPSSourceOff
inline constexpr auto t_ps_source_on = std::chrono::milliseconds{435};  // tPSSourceOn

// The swap standbys (tc::swap_standby, TypeC.hpp): the timeout is the
// safety net for a PD layer that never completes - a swap without the
// partner's PS_RDY has failed, and connection resolution restarts from
// Unattached.SNK

// The old sink, waiting for the old source's PS_RDY before taking over
struct swap_standby_to_src {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        swap_standby{power_role::source},
        pd_connection{}
    );

    static constexpr auto timeout = t_ps_source_off;
};

// The old source, its PS_RDY sent, waiting for the new source's
struct swap_standby_to_snk {
    static constexpr auto annotations = fsm::annotate(
        cc_termination{cc_pull::rd},
        vbus_power{vbus_path::open},
        vbus_level{vbus_level::unwatched},
        swap_standby{power_role::sink},
        pd_connection{}
    );

    static constexpr auto timeout = t_ps_source_on;
};

// --- guards ------------------------------------------------------------------

// TryWait.SNK holds a detected single Rp: the attach has a plug to
// resolve from
struct rp_detected {
    static bool check(auto const& state) { return singleRp(state.attached.detected); }
};

// --- timer-range maps --------------------------------------------------------

// Composed per preference like the flows they check; the shared attach
// flows have no timed states of their own
template<drp_timing const& TIMING>
using core_timer_ranges = fsm::timer_ranges<
    error_recovery_timer_range,
    hard_reset_timer_ranges,
    fsm::timed_by<unattached_snk<TIMING>, spec::t_drp_pw>,
    fsm::timed_by<unattached_src<TIMING>, spec::t_drp_pw>,
    fsm::timed_by<swap_standby_to_src, spec::t_ps_source_off>,
    fsm::timed_by<swap_standby_to_snk, spec::t_ps_source_on>>;

template<drp_timing const& TIMING>
using try_src_timer_ranges = fsm::timer_ranges<
    fsm::timed_by<try_src<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_src_until_safe0v<TIMING>, t_try_timeout_remainder<TIMING>>,
    fsm::timed_by<try_wait_snk, spec::t_cc_debounce>>;

template<drp_timing const& TIMING>
using try_snk_timer_ranges = fsm::timer_ranges<
    fsm::timed_by<try_snk_wait<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_wait_src<TIMING>, spec::t_drp_try>>;

// --- transition table composition --------------------------------------------

// Sink-role flow: the sink layer's attach flow anchored at the
// toggling Unattached.SNK, plus the toggle to the Rp phase; SNK_ATTACH
// is where a successful attach leads - Attached.SNK, or Try.SRC for a
// source-preferring port. A DRP leaves AttachWait.SNK's open exit for
// Unattached.SRC (4.5.2.2.4.2) - listed ahead of the shared flow's
// rows, whose Unattached.SNK is the role lock's fallback
template<drp_timing const& TIMING, fsm::concepts::state SNK_ATTACH>
using sink_flow = fsm::transition_table<
    // the Rd slice is up: advertise Rp - unless the port is locked
    // to the sink role, then another Rd slice (re-asking each time)
    fsm::transition<fsm::from<unattached_snk<TIMING>>, fsm::on<fsm::timeout>, fsm::guard<sourcing_allowed>, fsm::to<unattached_src<TIMING>>>,
    fsm::transition<fsm::from<unattached_snk<TIMING>>, fsm::on<fsm::timeout>, fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>, sourcing_allowed>, fsm::to<unattached_src<TIMING>>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>, sourcing_allowed>, fsm::to<unattached_src<TIMING>>>,
    sink_attach_flow<unattached_snk<TIMING>, SNK_ATTACH>>;

// Source-role flow; SRC_ATTACH is Attached.SRC, or Try.SNK for a
// sink-preferring port. A DRP leaves AttachWait.SRC's open exit for
// Unattached.SNK (4.5.2.2.8.2), and a sink detach for Rd with VBUS
// being removed - where it goes on from there is the preference's:
// vbus_removed_flow or TryWait.SNK (4.5.2.2.9.2)
template<drp_timing const& TIMING, fsm::concepts::state SRC_ATTACH>
using source_flow = fsm::transition_table<
    fsm::transition<fsm::from<unattached_src<TIMING>>, fsm::on<fsm::timeout>, fsm::to<unattached_snk<TIMING>>>,
    source_attach_flow<unattached_src<TIMING>, SRC_ATTACH, unattached_snk<TIMING>>,
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::cc_detected>, fsm::guard<rd_removed>, fsm::to<removing_vbus>>>;

// VBUS removed after Attached.SRC, no Try: toggling resumes at Rd,
// where a source presenting Rp meanwhile is detected at once
template<drp_timing const& TIMING>
using vbus_removed_flow = fsm::transition_table<
    fsm::transition<fsm::from<removing_vbus>, fsm::on<event::vbus_reached_safe0v>, fsm::to<unattached_snk<TIMING>>>>;

template<drp_timing const& TIMING>
using try_src_flow = fsm::transition_table<
    // tDRPTry is up: an Rd under debounce keeps Try.SRC going, else
    // TryWait.SNK with VBUS at vSafe0V, or Rp stays until it is there
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<fsm::timeout>, fsm::guard<cc_state_pending>, fsm::to<try_src_until_safe0v<TIMING>>>,
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<fsm::timeout>, fsm::guard<vbus_at_safe0v>, fsm::to<try_wait_snk>>,
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<fsm::timeout>, fsm::to<try_src_until_safe0v<TIMING>>>,
    // the Rd not detected with VBUS at vSafe0V, or tTryTimeout up:
    // TryWait.SNK - an Rd under debounce has its detection decide
    fsm::transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<event::cc_detected>, fsm::guard<vbus_at_safe0v>, fsm::to<try_wait_snk>>,
    fsm::internal_transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<event::vbus_reached_safe0v>, fsm::guard<cc_state_pending>>,
    fsm::transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<event::vbus_reached_safe0v>, fsm::to<try_wait_snk>>,
    fsm::transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<fsm::timeout>, fsm::guard<cc_state_pending>, fsm::to<try_src_timed_out>>,
    fsm::transition<fsm::from<try_src_until_safe0v<TIMING>>, fsm::on<fsm::timeout>, fsm::to<try_wait_snk>>,
    fsm::transition<fsm::from<try_src_timed_out>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_src_timed_out>, fsm::on<event::cc_detected>, fsm::to<try_wait_snk>>,
    // the sink left Attached.SRC: TryWait.SNK once VBUS is down
    fsm::transition<fsm::from<removing_vbus>, fsm::on<event::vbus_reached_safe0v>, fsm::to<try_wait_snk>>,
    // the open line ends TryWait.SNK, a detected Rp is kept for the
    // attach; tCCDebounce up: attach with VBUS and an Rp, else VBUS
    // detected from now on attaches
    fsm::transition<fsm::from<try_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<unattached_snk<TIMING>>>,
    fsm::internal_transition<fsm::from<try_wait_snk>, fsm::on<event::cc_detected>>,
    fsm::internal_transition<fsm::from<try_wait_snk>, fsm::on<event::vbus_present>>,
    fsm::transition<fsm::from<try_wait_snk>, fsm::on<fsm::timeout>, fsm::guard<rp_detected, vbus_at_safe5v>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<try_wait_snk>, fsm::on<fsm::timeout>, fsm::to<try_wait_snk_debounced>>,
    fsm::transition<fsm::from<try_wait_snk_debounced>, fsm::on<event::vbus_present>, fsm::guard<rp_detected>, fsm::to<state::attached_snk>>,
    fsm::internal_transition<fsm::from<try_wait_snk_debounced>, fsm::on<event::vbus_present>>,
    fsm::transition<fsm::from<try_wait_snk_debounced>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<try_wait_snk_debounced>, fsm::on<event::cc_detected>, fsm::guard<single_rp, vbus_at_safe5v>, fsm::to<state::attached_snk>>,
    fsm::internal_transition<fsm::from<try_wait_snk_debounced>, fsm::on<event::cc_detected>>>;

template<drp_timing const& TIMING>
using try_snk_flow = fsm::transition_table<
    // the CC pins are not monitored during the tDRPTry wait; then a
    // single Rp attaches with VBUS or waits for it, no Rp for
    // tTryCCDebounce leaves for TryWait.SRC, Rp on both pins waits
    fsm::transition<fsm::from<try_snk_wait<TIMING>>, fsm::on<fsm::timeout>, fsm::to<try_snk>>,
    fsm::transition<fsm::from<try_snk>, fsm::on<event::cc_detected>, fsm::guard<single_rp, vbus_at_safe5v>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<try_snk>, fsm::on<event::cc_detected>, fsm::guard<single_rp>, fsm::to<try_snk_rp_detected>>,
    fsm::transition<fsm::from<try_snk>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_snk_rp_detected>, fsm::on<event::vbus_present>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<try_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::guard<single_rp>, fsm::to<try_snk_rp_detected>>,
    fsm::transition<fsm::from<try_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::guard<fsm::not_<source_present>>, fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_snk_rp_detected>, fsm::on<event::cc_detected>, fsm::to<try_snk>>,
    // tDRPTry is up: an Rd under debounce has its detection decide,
    // else toggling resumes
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_detected>, fsm::guard<single_rd, vbus_at_safe0v>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<try_wait_src_rd_detected>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::vbus_reached_safe0v>>,
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<fsm::timeout>, fsm::guard<cc_state_pending>, fsm::to<try_wait_src_timed_out>>,
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<fsm::timeout>, fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_rd_detected>, fsm::on<event::vbus_reached_safe0v>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_wait_src_rd_detected>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<try_wait_src_rd_detected>>,
    fsm::transition<fsm::from<try_wait_src_rd_detected>, fsm::on<event::cc_detected>, fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_timed_out>, fsm::on<event::cc_detected>, fsm::guard<single_rd, vbus_at_safe0v>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<try_wait_src_timed_out>, fsm::on<event::cc_detected>, fsm::guard<single_rd>, fsm::to<try_wait_src_rd_detected>>,
    fsm::transition<fsm::from<try_wait_src_timed_out>, fsm::on<event::cc_detected>, fsm::to<unattached_snk<TIMING>>>>;

// PD-directed role swaps (spec: Attached.SNK <-> Attached.SRC "as
// directed by USB PD"): a power swap passes through its standby while
// the roles change hands, a data role swap changes no terminations and
// only flips the context's data role. The message choreography and the
// decision to swap are the PD layer's business, gated by the injected
// policy observers
template<drp_timing const& TIMING>
using swap_flow = fsm::transition_table<
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::swap_to_source>, fsm::to<swap_standby_to_src>>,
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::swap_to_sink>, fsm::to<swap_standby_to_snk>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<event::swap_complete>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<event::swap_abort>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<fsm::timeout>, fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<event::swap_complete>, fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<event::swap_abort>, fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<fsm::timeout>, fsm::to<unattached_snk<TIMING>>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::swap_data_role>>,
    fsm::internal_transition<fsm::from<state::attached_src>, fsm::on<event::swap_data_role>>>;

// The port rests in the sink layer's Disabled state and goes live
// toggling at Rd
template<drp_timing const& TIMING>
using entry_flow = fsm::transition_table<
    fsm::initial<state::disabled_snk>,
    fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>, fsm::to<unattached_snk<TIMING>>>>;

// Named structs, not aliases: the short name replaces the fully
// spelled table type in every mangled symbol. Timeout bounds and
// reachability are checked in test/compliance.cpp
template<drp_timing const& TIMING, drp_preference PREFERENCE>
struct table_for : fsm::transition_table<
                       entry_flow<TIMING>,
                       sink_flow<TIMING, state::attached_snk>,
                       source_flow<TIMING, state::attached_src>,
                       vbus_removed_flow<TIMING>,
                       swap_flow<TIMING>,
                       error_recovery_flow<unattached_snk<TIMING>>,
                       hard_reset_flow<unattached_snk<TIMING>>> {
    static_assert(timingWithinSpec<TIMING>());
};

// Where the sink flow would attach, a source-preferring port tries Rp
// first - unless locked to the sink role. Listed ahead of the shared
// flow's attach rows: alternatives are tried in table order, so the
// lock's no falls through to the plain sink attach
template<drp_timing const& TIMING>
using try_src_entry = fsm::transition_table<
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<event::cc_detected>, fsm::guard<single_rp, vbus_at_safe5v, sourcing_allowed>, fsm::to<try_src<TIMING>>>,
    fsm::transition<fsm::from<state::attach_wait_snk_rp_detected>, fsm::on<event::vbus_present>, fsm::guard<sourcing_allowed>, fsm::to<try_src<TIMING>>>>;

// A Try.SRC DRP leaves ErrorRecovery for Unattached.SRC (4.5.2.2.2.2)
template<drp_timing const& TIMING>
struct table_for<TIMING, drp_preference::source> : fsm::transition_table<
                                                       entry_flow<TIMING>,
                                                       try_src_entry<TIMING>,
                                                       sink_flow<TIMING, state::attached_snk>,
                                                       source_flow<TIMING, state::attached_src>,
                                                       try_src_flow<TIMING>,
                                                       swap_flow<TIMING>,
                                                       error_recovery_flow<unattached_src<TIMING>>,
                                                       hard_reset_flow<unattached_snk<TIMING>>> {
    static_assert(timingWithinSpec<TIMING>());
};

template<drp_timing const& TIMING>
struct table_for<TIMING, drp_preference::sink> : fsm::transition_table<
                                                     entry_flow<TIMING>,
                                                     sink_flow<TIMING, state::attached_snk>,
                                                     source_flow<TIMING, try_snk_wait<TIMING>>,
                                                     vbus_removed_flow<TIMING>,
                                                     try_snk_flow<TIMING>,
                                                     swap_flow<TIMING>,
                                                     error_recovery_flow<unattached_snk<TIMING>>,
                                                     hard_reset_flow<unattached_snk<TIMING>>> {
    static_assert(timingWithinSpec<TIMING>());
};

template<drp_timing const& TIMING, drp_preference PREFERENCE>
using table_for_t = table_for<TIMING, PREFERENCE>;

} // namespace drp

} // namespace tc

template<
    concepts::cc CC,
    concepts::vbus VBUS,
    fsm::concepts::timer TIMER,
    drp_timing const& TIMING = default_drp_timing,
    drp_preference PREFERENCE = drp_preference::none,
    typename... OBSERVERs>
class TypeCDrp
    : public tc::
          port_frontend<TypeCDrp<CC, VBUS, TIMER, TIMING, PREFERENCE, OBSERVERs...>, CC, VBUS> {
    static_assert(
        tc::hardware_annotations_complete_v<tc::drp::table_for_t<TIMING, PREFERENCE>>,
        "every state must annotate its CC termination, VBUS power path and watched VBUS level"
    );

public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). timer runs the connection states' timeouts
    // (the toggle, the Try walls, the standbys), cc_timer the CC
    // debounce. The advertisement is the Rp presented whenever the
    // port advertises the source role. The drivers observe the machine
    // (one VBUS switch position for both roles: entering the other
    // role's states collapses this one's path by value), the observers
    // are injected after the built-in ones (timer, the two drivers, cc
    // watcher); attach results are observed on the attached states'
    // instance values with the role encoded in the info type, and an
    // observer answering check(usbc::sourcing_allowed) is the role lock
    // (sink-only while it says no)
    TypeCDrp(
        CC& cc,
        VBUS& vbus,
        TIMER& timer,
        TIMER& cc_timer,
        rp_value advertisement,
        OBSERVERs&... observers
    )
        : cc_driver_(cc), vbus_driver_(vbus), cc_(cc_timer), timer_(timer), timed_(timer_),
          advertisement_(advertisement), observers_(observers...),
          sm_(timed_, cc, vbus, cc_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    TypeCDrp(CC& cc, VBUS& vbus, TIMER& timer, TIMER& cc_timer, OBSERVERs&... observers)
        : TypeCDrp(cc, vbus, timer, cc_timer, rp_value::usb_default, observers...)
    {
    }

    // Power role swap directed by the layer above (a USB PD PR_Swap
    // both sides agreed to - the arbitration happened there), in
    // phases: begin enters the swap standby - power paths off, Rd
    // presented, detach detection suspended while VBUS is legitimately
    // absent - completeSwap() (the partner's PS_RDY) lands in the new
    // attached state, abortSwap() restores the departing role. A
    // standby left to its spec timeout (tPSSourceOff/tPSSourceOn)
    // falls back to the departing role on its own. False when not
    // attached in the departing role. Call from the stack's serialized
    // context
    bool beginSwapToSource()
    {
        if (powerRole() != power_role::sink) {
            return false;
        }
        return sm_.process(tc::event::swap_to_source{});
    }

    bool beginSwapToSink()
    {
        if (powerRole() != power_role::source) {
            return false;
        }
        return sm_.process(tc::event::swap_to_sink{});
    }

    // Both only mean something in a swap standby - the standbys'
    // swap_standby annotation answers that, where the old process()
    // return cannot (the queued machine reports acceptance, not whether
    // a transition fired)
    bool completeSwap() { return inSwapStandby() && sm_.process(tc::event::swap_complete{}); }

    bool abortSwap() { return inSwapStandby() && sm_.process(tc::event::swap_abort{}); }

    // Data role swap directed by the layer above (an agreed USB PD
    // DR_Swap): no termination changes, only the context's data role
    // flips, by an internal transition - observers hear the flip from
    // the engine's pe::data_role_changed, the role at attach from
    // tc::attached_data_role. False while not attached
    bool swapDataRole()
    {
        if (!dataRole()) {
            return false;
        }
        return applyDataRoleSwap();
    }

    // The flip without the arbitration: for a PD layer applying a
    // DR_Swap it already negotiated (the verdict was asked when the
    // message exchange began). The attach check replaces the old
    // process() return: the queued machine reports acceptance, not
    // whether the internal transition fired
    bool applyDataRoleSwap()
    {
        if (!dataRole()) {
            return false;
        }
        sm_.process(tc::event::swap_data_role{});
        return true;
    }

    // The role lock's answer (usbc::sourcing_allowed): the injected
    // object answering it decides, the question's static default (yes)
    // stands without one - the same resolution the machine applies
    bool sourcingAllowed()
    {
        return std::apply(
            [](auto&... observer) { return (allowsSourcing(observer) && ...); },
            observers_
        );
    }

    // The attached pair's power role, as the Attached states annotate
    // it; nullopt while not attached (a swap standby included)
    std::optional<power_role> powerRole() const
    {
        if (auto const attached = sm_.template annotation<tc::attached_role>()) {
            return attached->role;
        }
        return std::nullopt;
    }

    // The attached pair's data role, from the connection's attachment;
    // nullopt while not attached
    std::optional<data_role> dataRole() const
    {
        if (powerRole()) {
            return sm_.template context<tc::attachment>().data;
        }
        return std::nullopt;
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(advertisement_); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts from the preference's
    // Unattached state. Call from the stack's serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCDrp, CC, VBUS>;

    bool inSwapStandby() const { return sm_.template annotation<tc::swap_standby>().has_value(); }

    static bool allowsSourcing(auto& observer)
    {
        if constexpr (
            fsm::concepts::
                answers_stateless_guard<std::remove_cvref_t<decltype(observer)>, sourcing_allowed>
        ) {
            return observer.check(sourcing_allowed{});
        } else {
            return true;
        }
    }

    using table = tc::drp::table_for_t<TIMING, PREFERENCE>;
    using queued_timer = fsm::QueuedTimer<TIMER>;

    CC& cc_driver_;
    VBUS& vbus_driver_;
    tc::CcWatcher<TIMER> cc_;
    queued_timer timer_;
    fsm::timed<queued_timer&> timed_;
    rp_value advertisement_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::QueuedMachine<
        table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<queued_timer&>,
        CC,
        VBUS,
        tc::CcWatcher<TIMER>,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
