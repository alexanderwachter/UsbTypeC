/*
 * USB Type-C dual-role port (DRP) connection layer per the USB Type-C
 * Cable and Connector Specification. The port toggles between
 * Unattached.SNK (presenting Rd) and Unattached.SRC (presenting Rp)
 * and follows the sink or source attach flow of whichever role it was
 * advertising when a partner appeared - the AttachWait, Attached and
 * UnattachedWait states are shared with TypeCSink.hpp/TypeCSource.hpp.
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
 * flow would attach. The debounce phases of the Try states are
 * modelled as sub-states (a state machine state has one timeout), and
 * each spec state's time budget is a phase deadline (fsm::deadlined)
 * shared by its sub-states - a hard wall a flapping partner cannot
 * extend by bouncing between them:
 *   Try.SRC        = try_src + try_src_debounce (tTryCCDebounce),
 *                    under the tDRPTry deadline
 *   TryWait.SNK    = try_wait_snk (tDRPTryWait)
 *   Try.SNK        = try_snk (tDRPTry, CC ignored) + try_snk_monitor
 *                    + try_snk_debounce (tPDDebounce), under the
 *                    tTryTimeout deadline
 *   TryWait.SRC    = try_wait_src + try_wait_src_debounce
 *                    (tTryCCDebounce), under the tDRPTryWait deadline,
 *                    + try_wait_src_safe0v (vSafe0V wait, Rd detected -
 *                    the wall no longer applies)
 * A deadline expiring during a debounce does not abort it (the spec's
 * walls apply while the wanted termination has "not yet been
 * detected"): the debounce state records the expiry in the port
 * context, an attach may still complete, and a failed debounce leaves
 * the phase instead of re-arming it (the try_expired guard).
 *
 * Role lock (port control, conformance-neutral): an injected object
 * answering bool check(tc::drp::sourcing_allowed) keeps the port
 * sink-only while it says no - the toggle stays at Rd (re-asking each
 * slice, so lifting the lock resumes toggling within one), a
 * source-preferring port attaches as sink instead of trying Rp, and
 * swaps to source are vetoed ahead of the swap policies. The
 * question's static default is yes: without such an object nothing
 * changes.
 *
 * Deviations from the spec, accepted knowingly: TryWait.SNK attaches
 * on VBUS with a single Rp in the context rather than debouncing the
 * Rp separately; AttachWait exit back to toggling uses tCCDebounce
 * (the spec's shorter tPDDebounce path is not modelled, matching the
 * sink/source layers).
 *
 * Integration matches TypeCSink/TypeCSource: initialized drivers plus
 * a caller-owned timer policy instance; construction rests in the
 * spec's Disabled state and start() goes live in Unattached.SNK. The
 * vbus driver is re-armed per state (vSafe5V in sink-role states,
 * vSafe0V in source-role states, vSinkDisconnect while Attached.SNK)
 * and the callback's meaning is mapped through the armed level.
 * Injected observers watching the attached states' instance values
 * learn which role attached through the info type: tc::attach_info
 * (orientation and advertisement) for Attached.SNK, plug_orientation
 * for Attached.SRC; the hw drivers apply the tc::polarity element.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>
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
    std::chrono::milliseconds t_drp{75};             // tDRP, 50 - 100 ms
    unsigned dc_src = 50;                            // dcSRC.DRP, 30 - 70 %
    std::chrono::milliseconds t_drp_try{100};        // tDRPTry, 75 - 150 ms
    std::chrono::milliseconds t_drp_try_wait{600};   // tDRPTryWait, 400 - 800 ms
    std::chrono::milliseconds t_try_cc_debounce{15}; // tTryCCDebounce, 10 - 20 ms
    std::chrono::milliseconds t_try_timeout{800};    // tTryTimeout, 550 - 1100 ms
    std::chrono::milliseconds t_pd_debounce{15};     // tPDDebounce, 10 - 20 ms
};

inline constexpr drp_timing default_drp_timing{};

namespace concepts {

// An injected observer with the runtime say over PD-directed role
// swaps of the given kind (power_role or data_role), consulted with
// the role the port would swap to. Every such observer may veto; with
// none injected, swaps of that kind are refused
template<typename T, typename ROLE>
concept drp_swap_policy = requires(T policy, ROLE role) {
    { policy.allowSwap(role) } -> std::convertible_to<bool>;
};

} // namespace concepts

namespace tc {

namespace drp {

template<drp_timing const& TIMING>
consteval bool timingWithinSpec()
{
    static_assert(spec::t_drp.contains(TIMING.t_drp), "tDRP outside the spec's 50-100 ms");
    static_assert(spec::within(TIMING.dc_src, spec::dc_src_drp_min, spec::dc_src_drp_max),
                  "dcSRC.DRP outside the spec's 30-70 %");
    static_assert(spec::t_drp_try.contains(TIMING.t_drp_try),
                  "tDRPTry outside the spec's 75-150 ms");
    static_assert(spec::t_drp_try_wait.contains(TIMING.t_drp_try_wait),
                  "tDRPTryWait outside the spec's 400-800 ms");
    static_assert(spec::t_try_cc_debounce.contains(TIMING.t_try_cc_debounce),
                  "tTryCCDebounce outside the spec's 10-20 ms");
    static_assert(spec::t_try_timeout.contains(TIMING.t_try_timeout),
                  "tTryTimeout outside the spec's 550-1100 ms");
    static_assert(spec::t_pd_debounce.contains(TIMING.t_pd_debounce),
                  "tPDDebounce outside the spec's 10-20 ms");
    return true;
}

// The Rp slice of the toggle period; the Rd slice is the remainder
template<drp_timing const& TIMING>
inline constexpr std::chrono::milliseconds t_src_slice = TIMING.t_drp * TIMING.dc_src / 100;

// --- toggling unattached states ---------------------------------------------

// Unattached.SNK of a DRP: Rd presented for the sink slice of tDRP
template<drp_timing const& TIMING>
struct unattached_snk : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_drp - t_src_slice<TIMING>;

    using state::sink_state::sink_state;
};

// Unattached.SRC of a DRP: Rp presented for the source slice of tDRP
template<drp_timing const& TIMING>
struct unattached_src : state::source_state {
    // toggling: nothing measured - AttachWait re-arms vSafe0V on entry
    static constexpr vbus_level vbus_watch = vbus_level::unwatched;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = t_src_slice<TIMING>;

    // entering on the discharge-complete event records what it means
    unattached_src(event::vbus_reached_safe0v const&, port_context& ctx) : source_state(ctx)
    {
        context.vbus_safe0v = true;
    }
    using state::source_state::source_state;
};

// --- Try.SRC / TryWait.SNK (drp_preference::source) --------------------------

// Rp presented where the sink flow would have attached; the tDRPTry
// budget is a phase deadline shared with the debounce sub-state, so a
// flapping partner cannot extend it
template<drp_timing const& TIMING>
struct try_src : state::source_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe0v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto deadline    = TIMING.t_drp_try;

    // entered only with the phase fresh (an expired phase leaves
    // through its debounce guard): open a new budget
    explicit try_src(port_context& ctx) : source_state(ctx) { context.try_expired = false; }
};

// A single Rd appeared in Try.SRC: stable for tTryCCDebounce attaches.
// The phase deadline keeps running; expiring mid-debounce does not
// abort it - the failure exit leaves the phase instead of re-arming
template<drp_timing const& TIMING>
struct try_src_debounce : state::source_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe0v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_try_cc_debounce;
    static constexpr auto deadline    = TIMING.t_drp_try;

    try_src_debounce(event::cc_changed const& event, port_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::source_state::source_state;
};

// The partner did not present Rd: back to Rd for tDRPTryWait, attaching
// as sink when the partner sources VBUS
template<drp_timing const& TIMING>
struct try_wait_snk : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_drp_try_wait;

    // a fresh-attach gateway: a stale hard-reset window flag must not
    // leak into the attach it resolves
    explicit try_wait_snk(port_context& ctx) : sink_state(ctx) { context.resuming = false; }
};

// --- Try.SNK / TryWait.SRC (drp_preference::sink) ----------------------------

// Rd presented where the source flow would have attached; the spec
// mandates waiting tDRPTry before the CC pins are even monitored. The
// whole Try.SNK phase - wait, monitor, debounce - runs under one
// tTryTimeout deadline
template<drp_timing const& TIMING>
struct try_snk : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_drp_try;
    static constexpr auto deadline    = TIMING.t_try_timeout;

    explicit try_snk(port_context& ctx) : sink_state(ctx) { context.try_expired = false; }
};

// Monitoring phase of Try.SNK: the phase deadline is the only clock
template<drp_timing const& TIMING>
struct try_snk_monitor : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto deadline    = TIMING.t_try_timeout;

    using state::sink_state::sink_state;
};

// A single Rp appeared in Try.SNK: stable for tPDDebounce with VBUS
// present attaches; a failure after the phase deadline expired exits
template<drp_timing const& TIMING>
struct try_snk_debounce : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_pd_debounce;
    static constexpr auto deadline    = TIMING.t_try_timeout;

    try_snk_debounce(event::cc_changed const& event, port_context& ctx) : sink_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::sink_state::sink_state;
};

// The partner did not present Rp: back to Rp under the tDRPTryWait
// phase deadline
template<drp_timing const& TIMING>
struct try_wait_src : state::source_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe0v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto deadline    = TIMING.t_drp_try_wait;

    explicit try_wait_src(port_context& ctx) : source_state(ctx)
    {
        context.try_expired = false;
    }
};

// A single Rd appeared in TryWait.SRC: stable for tTryCCDebounce
// attaches once VBUS is at vSafe0V; a failure after the phase
// deadline expired exits
template<drp_timing const& TIMING>
struct try_wait_src_debounce : state::source_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe0v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);
    static constexpr auto timeout     = TIMING.t_try_cc_debounce;
    static constexpr auto deadline    = TIMING.t_drp_try_wait;

    try_wait_src_debounce(event::cc_changed const& event, port_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::source_state::source_state;
};

// Rd debounced but VBUS not yet at vSafe0V: attach follows the report
template<drp_timing const& TIMING>
struct try_wait_src_safe0v : state::source_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe0v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rp}, vbus_power{vbus_path::open}, vbus_watch);

    using state::source_state::source_state;
};

// --- PR_Swap standby ---------------------------------------------------------

// Operating values for the PS_RDY waits, within the spec ranges
inline constexpr auto t_ps_source_off = std::chrono::milliseconds{835}; // tPSSourceOff
inline constexpr auto t_ps_source_on  = std::chrono::milliseconds{435}; // tPSSourceOn

// The window of a PR_Swap where the roles change hands: power paths
// off, Rd presented, and CC/VBUS reports absorbed - VBUS is
// legitimately absent and the partner's terminations flap, neither is
// a detach. The timeout is the safety net for a PD layer that never
// completes: a swap without the partner's PS_RDY has failed, and
// connection resolution restarts from Unattached.SNK

// Annotation tag marking that window: completeSwap()/abortSwap() only
// mean something inside it, tracked by the hw driver
struct swap_standby {
    constexpr bool operator==(swap_standby const&) const = default;
};

// The old sink, waiting for the old source's PS_RDY before taking over
struct swap_standby_to_src : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch, swap_standby{});
    static constexpr auto timeout     = t_ps_source_off;

    using state::sink_state::sink_state;
};

// The old source, its PS_RDY sent, waiting for the new source's
struct swap_standby_to_snk : state::sink_state {
    static constexpr vbus_level vbus_watch = vbus_level::safe5v;
    static constexpr auto annotations =
        fsm::annotate(cc_termination{cc_pull::rd}, vbus_power{vbus_path::open}, vbus_watch, swap_standby{});
    static constexpr auto timeout     = t_ps_source_on;

    using state::sink_state::sink_state;
};

// --- guards ------------------------------------------------------------------

// Guards deciding on the event's CC payload (the context still holds
// the pre-event status when a guard runs) or on the context

struct rd_on_event {
    static bool check(auto const&, event::cc_changed const& event) { return singleRd(event.cc); }
};

struct rp_on_event {
    static bool check(auto const&, event::cc_changed const& event) { return singleRp(event.cc); }
};

struct rp_on_event_with_vbus {
    static bool check(auto const& state, event::cc_changed const& event)
    {
        return singleRp(event.cc) && state.context.vbus_present;
    }
};

struct rd_in_context {
    static bool check(auto const& state) { return singleRd(state.context.cc); }
};

struct rd_in_context_at_safe0v {
    static bool check(auto const& state)
    {
        return singleRd(state.context.cc) && state.context.vbus_safe0v;
    }
};

struct rp_in_context {
    static bool check(auto const& state) { return singleRp(state.context.cc); }
};

struct rp_in_context_with_vbus {
    static bool check(auto const& state)
    {
        return singleRp(state.context.cc) && state.context.vbus_present;
    }
};

// The phase deadline expired while this debounce ran (the role bases'
// fsm::deadline handler recorded it): its failure exits the phase
// instead of re-arming it
struct try_expired {
    static bool check(auto const& state) { return state.context.try_expired; }
};

// --- the role lock -----------------------------------------------------------

// May the port take the source role now? The table's question at
// every decision towards Rp - the toggle's Rp slice, Try.SRC where a
// source-preferring port's sink flow would attach - and the facade's
// for a swap to source. Its static default says yes; an application
// object injected into the port answering bool check(sourcing_allowed)
// overrides it, and while it says no the DRP behaves as a sink: it
// rests at Rd re-asking every slice, attaches as sink, vetoes swaps
// to source. A phase already running completes (a lock engaged
// mid-debounce may still resolve to Attached.SRC once), a port
// already sourcing keeps its contract - the application requests the
// swap. Exactly one answering object per port (mtl's rule)
struct sourcing_allowed {
    static constexpr bool check() { return true; }
};

// Try.SRC combines the sink flow's attach conditions with that
// answer; the facade's try_gate answers it, so the application object
// never sees a state
struct try_src_allowed {};

// --- timer-range maps --------------------------------------------------------

// Composed per preference like the flows they check; the shared attach
// flows bring the sink and source maps along
template<drp_timing const& TIMING>
using core_timer_ranges = mtl::linearize_t<mtl::typelist<
    sink_timer_ranges, source_timer_ranges, error_recovery_timer_range,
    hard_reset_timer_ranges,
    fsm::timed_by<unattached_snk<TIMING>, spec::t_drp_pw>,
    fsm::timed_by<unattached_src<TIMING>, spec::t_drp_pw>,
    fsm::timed_by<swap_standby_to_src, spec::t_ps_source_off>,
    fsm::timed_by<swap_standby_to_snk, spec::t_ps_source_on>>>;

template<drp_timing const& TIMING>
using try_src_timer_ranges = mtl::typelist<
    fsm::timed_by<try_src_debounce<TIMING>, spec::t_try_cc_debounce>,
    fsm::timed_by<try_wait_snk<TIMING>, spec::t_drp_try_wait>>;

template<drp_timing const& TIMING>
using try_snk_timer_ranges = mtl::typelist<
    fsm::timed_by<try_snk<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_snk_debounce<TIMING>, spec::t_pd_debounce>,
    fsm::timed_by<try_wait_src_debounce<TIMING>, spec::t_try_cc_debounce>>;

// The phase deadlines against the spec ranges, checked with
// fsm::deadlines_within_bounds alongside the timeout maps
template<drp_timing const& TIMING>
using try_src_deadline_ranges = mtl::typelist<
    fsm::timed_by<try_src<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_src_debounce<TIMING>, spec::t_drp_try>>;

template<drp_timing const& TIMING>
using try_snk_deadline_ranges = mtl::typelist<
    fsm::timed_by<try_snk<TIMING>, spec::t_try_timeout>,
    fsm::timed_by<try_snk_monitor<TIMING>, spec::t_try_timeout>,
    fsm::timed_by<try_snk_debounce<TIMING>, spec::t_try_timeout>,
    fsm::timed_by<try_wait_src<TIMING>, spec::t_drp_try_wait>,
    fsm::timed_by<try_wait_src_debounce<TIMING>, spec::t_drp_try_wait>>;

// --- transition table composition --------------------------------------------

// Sink-role flow: the sink layer's attach flow anchored at the
// toggling Unattached.SNK, plus the toggle to the Rp phase; SNK_ATTACH
// is where a successful attach leads - Attached.SNK, or Try.SRC for a
// source-preferring port
template<drp_timing const& TIMING, typename SNK_ATTACH>
using sink_flow = mtl::concat_t<
    mtl::typelist<
        // the Rd slice is up: advertise Rp - unless the port is locked
        // to the sink role, then another Rd slice (re-asking each time)
        fsm::transition<fsm::from<unattached_snk<TIMING>>, fsm::on<fsm::timeout>,
                        fsm::to<unattached_src<TIMING>>, fsm::guard<sourcing_allowed>>,
        fsm::transition<fsm::from<unattached_snk<TIMING>>, fsm::on<fsm::timeout>,
                        fsm::to<unattached_snk<TIMING>>>>,
    sink_attach_flow<unattached_snk<TIMING>, SNK_ATTACH>>;

// Source-role flow; SRC_ATTACH is Attached.SRC, or Try.SNK for a
// sink-preferring port
template<drp_timing const& TIMING, typename SRC_ATTACH>
using source_flow = mtl::concat_t<
    mtl::typelist<fsm::transition<fsm::from<unattached_src<TIMING>>, fsm::on<fsm::timeout>,
                                  fsm::to<unattached_snk<TIMING>>>>,
    source_attach_flow<unattached_src<TIMING>, SRC_ATTACH>>;

template<drp_timing const& TIMING>
using try_src_flow = mtl::typelist<
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_src_debounce<TIMING>>, fsm::guard<rd_on_event>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::vbus_left_safe0v>>,
    // tDRPTry is up with no Rd under debounce: give up trying
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<fsm::deadline>,
                    fsm::to<try_wait_snk<TIMING>>>,
    // a CC change restarts the Rd debounce (the phase deadline keeps
    // running); expiry mid-debounce is only recorded - the debounce
    // may still attach, its failure leaves the phase
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_src_debounce<TIMING>>>,
    fsm::internal_transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<fsm::deadline>>,
    fsm::internal_transition<fsm::from<try_src_debounce<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_src_debounce<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_src>, fsm::guard<rd_in_context>>,
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_snk<TIMING>>, fsm::guard<try_expired>>,
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_src<TIMING>>>,
    // TryWait.SNK: the partner sourcing VBUS is the attach signal
    fsm::transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<state::attached_snk>, fsm::guard<rp_on_event_with_vbus>>,
    fsm::internal_transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<event::vbus_present>,
                    fsm::to<state::attached_snk>, fsm::guard<rp_in_context>>,
    fsm::internal_transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_wait_snk<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<unattached_snk<TIMING>>>>;

template<drp_timing const& TIMING>
using try_snk_flow = mtl::typelist<
    // the CC pins are not monitored during the initial tDRPTry wait
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_snk<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_in_context>>,
    fsm::transition<fsm::from<try_snk<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    // unreachable while tTryTimeout > tDRPTry (the spec ranges
    // guarantee it), but the armed deadline must be handled
    fsm::transition<fsm::from<try_snk<TIMING>>, fsm::on<fsm::deadline>,
                    fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_on_event>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_present>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_in_context>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_removed>>,
    // tTryTimeout is up with no Rp under debounce: stop trying
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<fsm::deadline>,
                    fsm::to<try_wait_src<TIMING>>>,
    // a CC change keeping the Rp restarts the debounce (the phase
    // deadline keeps running), losing it resumes monitoring - or
    // leaves the phase once the deadline expired
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_on_event>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src<TIMING>>, fsm::guard<try_expired>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    fsm::internal_transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::deadline>>,
    fsm::internal_transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_snk>, fsm::guard<rp_in_context_with_vbus>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src<TIMING>>, fsm::guard<try_expired>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    // TryWait.SRC
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src_debounce<TIMING>>, fsm::guard<rd_on_event>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::vbus_left_safe0v>>,
    // tDRPTryWait is up with no Rd under debounce: resume toggling
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<fsm::deadline>,
                    fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src_debounce<TIMING>>>,
    fsm::internal_transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::deadline>>,
    fsm::internal_transition<fsm::from<try_wait_src_debounce<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_wait_src_debounce<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_src>, fsm::guard<rd_in_context_at_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src_safe0v<TIMING>>, fsm::guard<rd_in_context>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<unattached_snk<TIMING>>, fsm::guard<try_expired>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_safe0v<TIMING>>, fsm::on<event::vbus_reached_safe0v>,
                    fsm::to<state::attached_src>>,
    fsm::internal_transition<fsm::from<try_wait_src_safe0v<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_safe0v<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src<TIMING>>>>;

// PD-directed role swaps (spec: Attached.SNK <-> Attached.SRC "as
// directed by USB PD"): a power swap passes through its standby while
// the roles change hands, a data role swap changes no terminations and
// only flips the context's data role. The message choreography and the
// decision to swap are the PD layer's business, gated by the injected
// policy observers
template<drp_timing const& TIMING>
using swap_flow = mtl::typelist<
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::swap_to_source>,
                    fsm::to<swap_standby_to_src>>,
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::swap_to_sink>,
                    fsm::to<swap_standby_to_snk>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<event::swap_complete>,
                    fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<event::swap_abort>,
                    fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<swap_standby_to_src>, fsm::on<fsm::timeout>,
                    fsm::to<unattached_snk<TIMING>>>,
    fsm::internal_transition<fsm::from<swap_standby_to_src>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<swap_standby_to_src>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<swap_standby_to_src>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<event::swap_complete>,
                    fsm::to<state::attached_snk>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<event::swap_abort>,
                    fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<swap_standby_to_snk>, fsm::on<fsm::timeout>,
                    fsm::to<unattached_snk<TIMING>>>,
    fsm::internal_transition<fsm::from<swap_standby_to_snk>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<swap_standby_to_snk>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<swap_standby_to_snk>, fsm::on<event::vbus_removed>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::swap_data_role>>,
    fsm::internal_transition<fsm::from<state::attached_src>, fsm::on<event::swap_data_role>>>;

// The port rests in the sink layer's Disabled state and goes live
// toggling at Rd
template<drp_timing const& TIMING>
using entry_flow = mtl::typelist<
    fsm::initial<state::disabled_snk>,
    fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>,
                    fsm::to<unattached_snk<TIMING>>>>;

// Named structs, not aliases: the short name replaces the fully
// spelled table type in every mangled symbol. Timeout bounds and
// reachability are checked in test/compliance.cpp
template<drp_timing const& TIMING, drp_preference PREFERENCE>
struct table_for
    : mtl::rebind_t<
          mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                         sink_flow<TIMING, state::attached_snk>,
                                         source_flow<TIMING, state::attached_src>,
                                         swap_flow<TIMING>,
                                         error_recovery_flow<unattached_snk<TIMING>>,
                                         hard_reset_flow<unattached_snk<TIMING>>>>,
          fsm::transition_table> {
    static_assert(timingWithinSpec<TIMING>());
};

// Where the sink flow would attach, a source-preferring port tries Rp
// first - unless locked to the sink role. Listed ahead of the shared
// flow's attach rows: alternatives are tried in table order, so the
// lock's no falls through to the plain sink attach
template<drp_timing const& TIMING>
using try_src_entry = mtl::typelist<
    fsm::transition<fsm::from<state::attach_wait_snk>, fsm::on<fsm::timeout>,
                    fsm::to<try_src<TIMING>>, fsm::guard<try_src_allowed>>,
    fsm::transition<fsm::from<state::attach_wait_snk_debounced>, fsm::on<event::vbus_present>,
                    fsm::to<try_src<TIMING>>, fsm::guard<sourcing_allowed>>>;

template<drp_timing const& TIMING>
struct table_for<TIMING, drp_preference::source>
    : mtl::rebind_t<
          mtl::linearize_t<mtl::typelist<entry_flow<TIMING>, try_src_entry<TIMING>,
                                         sink_flow<TIMING, state::attached_snk>,
                                         source_flow<TIMING, state::attached_src>,
                                         try_src_flow<TIMING>, swap_flow<TIMING>,
                                         error_recovery_flow<unattached_snk<TIMING>>,
                                         hard_reset_flow<unattached_snk<TIMING>>>>,
          fsm::transition_table> {
    static_assert(timingWithinSpec<TIMING>());
};

template<drp_timing const& TIMING>
struct table_for<TIMING, drp_preference::sink>
    : mtl::rebind_t<
          mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                         sink_flow<TIMING, state::attached_snk>,
                                         source_flow<TIMING, try_snk<TIMING>>,
                                         try_snk_flow<TIMING>, swap_flow<TIMING>,
                                         error_recovery_flow<unattached_snk<TIMING>>,
                                         hard_reset_flow<unattached_snk<TIMING>>>>,
          fsm::transition_table> {
    static_assert(timingWithinSpec<TIMING>());
};

template<drp_timing const& TIMING, drp_preference PREFERENCE>
using table_for_t = table_for<TIMING, PREFERENCE>;

} // namespace drp

// Applies both roles' hw annotations. Entering a state of one role
// shuts the other role's paths off first: the annotations carry only
// their own role's switches, and a role change must never leave the
// old role sourcing or sinking
template<concepts::tcpc TCPC, concepts::vbus VBUS>
struct drp_hw_driver : fsm::observing<drp_hw_driver<TCPC, VBUS>> {
    drp_hw_driver(TCPC& tcpc_ref, VBUS& vbus_ref, rp_value advertisement)
        : tcpc(tcpc_ref), vbus(vbus_ref), rp(advertisement)
    {
    }

    // A state the dispatch would silently skip is a table bug: the
    // previous state's terminations or power paths would stay applied.
    // Checked per element: every state must carry both
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(fsm::all_states_notified_v<annotation_probe<cc_termination>, TABLE>,
                      "drp_hw_driver: every state must annotate its CC termination");
        static_assert(fsm::all_states_notified_v<annotation_probe<vbus_power>, TABLE>,
                      "drp_hw_driver: every state must annotate its VBUS power path");
    }

    void notifyEntry(cc_termination termination) { tcpc.setCc(termination.pull, rp); }

    // One position for both roles: entering the other role's states
    // collapses this one's path by value
    void notifyEntry(vbus_power power)
    {
        tcpc.sinkVbus(power.path == vbus_path::sink);
        tcpc.sourceVbus(power.path == vbus_path::safe5v);
        vbus.discharge(power.path == vbus_path::safe0v);
    }

    void notifyEntry(polarity resolved) { tcpc.setPlugOrientation(resolved.orientation); }

    // the PR_Swap window, tracked for completeSwap()/abortSwap()
    void notifyEntry(drp::swap_standby) { in_swap_standby = true; }
    void notifyExit(drp::swap_standby) { in_swap_standby = false; }

    TCPC& tcpc;
    VBUS& vbus;
    rp_value rp;
    bool in_swap_standby = false;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         drp_timing const& TIMING = default_drp_timing,
         drp_preference PREFERENCE = drp_preference::none, typename... OBSERVERs>
class TypeCDrp
    : public tc::port_frontend<
          TypeCDrp<TCPC, VBUS, TIMER, TIMING, PREFERENCE, OBSERVERs...>, TCPC, VBUS> {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The advertisement is the Rp presented
    // whenever the port advertises the source role. The observers are
    // injected into the machine after the built-in ones (timer, hw
    // driver, vbus watcher); attach results are observed on the
    // attached states' attachedInfo() with the role encoded in the
    // info type, an observer providing onPdAlert(alert_status)
    // receives the alert bits this layer does not consume, one
    // providing allowSwap(power_role) is a swap policy, and one
    // answering check(tc::drp::sourcing_allowed) is the role lock
    // (sink-only while it says no). deadline_timer drives the Try
    // phases' hard walls (tDRPTry, tTryTimeout, tDRPTryWait)
    // alongside the per-state timer; a preference-none port never
    // arms it
    TypeCDrp(TCPC& tcpc, VBUS& vbus, TIMER& timer, TIMER& deadline_timer,
             rp_value advertisement, OBSERVERs&... observers)
        : tcpc_(tcpc), hw_(tcpc, vbus, advertisement), vbus_(vbus), timer_(timer),
          deadline_timer_(deadline_timer), timed_(timer_), deadlined_(deadline_timer_),
          observers_(observers...), sm_(timed_, deadlined_, hw_, vbus_, try_gate_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    TypeCDrp(TCPC& tcpc, VBUS& vbus, TIMER& timer, TIMER& deadline_timer,
             OBSERVERs&... observers)
        : TypeCDrp(tcpc, vbus, timer, deadline_timer, rp_value::usb_default, observers...)
    {
    }

    // Power role swap directed by the layer above (a USB PD PR_Swap),
    // in phases: begin enters the swap standby - power paths off, Rd
    // presented, detach detection suspended while VBUS is legitimately
    // absent - completeSwap() (the partner's PS_RDY) lands in the new
    // attached state, abortSwap() restores the departing role. A
    // standby left to its spec timeout (tPSSourceOff/tPSSourceOn)
    // falls back to the departing role on its own. Every injected
    // observer providing allowSwap(power_role) is consulted at begin
    // and may veto; with no such observer, swaps are refused. False
    // when vetoed or not attached in the departing role. Call from the
    // stack's serialized context
    bool beginSwapToSource()
    {
        if (!sm_.template is<tc::state::attached_snk>() || !swapAllowed(power_role::source)) {
            return false;
        }
        return sm_.process(tc::event::swap_to_source{});
    }

    bool beginSwapToSink()
    {
        if (!sm_.template is<tc::state::attached_src>() || !swapAllowed(power_role::sink)) {
            return false;
        }
        return sm_.process(tc::event::swap_to_sink{});
    }

    // Both only mean something in a swap standby - the hw driver's
    // swap_standby annotation tracking replaces the old process()
    // return (the queued machine reports acceptance, not whether a
    // transition fired)
    bool completeSwap()
    {
        return hw_.in_swap_standby && sm_.process(tc::event::swap_complete{});
    }

    bool abortSwap() { return hw_.in_swap_standby && sm_.process(tc::event::swap_abort{}); }

    // Data role swap directed by the layer above (a USB PD DR_Swap):
    // no termination changes, only the context's data role flips. Same
    // arbitration, asked with the data role the port would take. The
    // flip is an internal transition without machine hooks, so the new
    // role is forwarded to the observers providing onDataRole(data_role)
    bool swapDataRole()
    {
        auto const current = dataRole();
        if (!current ||
            !swapAllowed(*current == data_role::ufp ? data_role::dfp : data_role::ufp)) {
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
        auto const swapped = *dataRole();
        std::apply([&](auto&... observer) { (forwardDataRole(observer, swapped), ...); },
                   observers_);
        return true;
    }

    // The arbitration alone, for a PD layer answering the partner's
    // swap request: every injected policy observer for the role kind
    // is consulted with the role the port would take - a swap to
    // source first passes the role lock
    template<typename ROLE>
    bool swapAllowed(ROLE role)
    {
        if constexpr (std::is_same_v<ROLE, power_role>) {
            if (role == power_role::source && !sourcingAllowed()) {
                return false;
            }
        }
        constexpr bool any_policy =
            (concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, ROLE> || ...);
        return any_policy && std::apply(
                                 [role](auto&... observer) {
                                     return (allowsSwap(observer, role) && ...);
                                 },
                                 observers_);
    }

    // The role lock's answer (tc::drp::sourcing_allowed): the injected
    // object answering it decides, the question's static default (yes)
    // stands without one - the same resolution the machine applies
    bool sourcingAllowed()
    {
        return std::apply([](auto&... observer) { return (allowsSourcing(observer) && ...); },
                          observers_);
    }

    // The attached pair's power role; nullopt while not attached (a
    // swap standby included)
    std::optional<power_role> powerRole() const
    {
        if (sm_.template is<tc::state::attached_snk>()) {
            return power_role::sink;
        }
        if (sm_.template is<tc::state::attached_src>()) {
            return power_role::source;
        }
        return std::nullopt;
    }

    // The attached pair's data role; nullopt while not attached
    std::optional<data_role> dataRole() const
    {
        if (auto const* attached = sm_.template getIf<tc::state::attached_snk>()) {
            return attached->dataRole();
        }
        if (auto const* attached = sm_.template getIf<tc::state::attached_src>()) {
            return attached->dataRole();
        }
        return std::nullopt;
    }

    // The go-live moment, provided by the shared frontend
    void start() { this->startPort(); }

    // PD-directed Type-C Error Recovery: both terminations removed for
    // tErrorRecovery, then resolution restarts from Unattached.SNK.
    // Call from the stack's serialized context
    bool errorRecovery() { return sm_.process(tc::event::error_recovery{}); }

    // PD-directed hard-reset window: the attach is held while VBUS
    // legitimately cycles through vSafe0V and back
    bool hardResetWindow() { return sm_.process(tc::event::hard_reset{}); }

private:
    friend tc::port_frontend<TypeCDrp, TCPC, VBUS>;

    template<typename ROLE>
    static bool allowsSwap(auto& observer, ROLE role)
    {
        if constexpr (concepts::drp_swap_policy<std::remove_cvref_t<decltype(observer)>,
                                                ROLE>) {
            return observer.allowSwap(role);
        } else {
            return true;
        }
    }

    static void forwardDataRole(auto& observer, data_role role)
    {
        if constexpr (requires { observer.onDataRole(role); }) {
            observer.onDataRole(role);
        }
    }

    static bool allowsSourcing(auto& observer)
    {
        if constexpr (fsm::concepts::answers_stateless_guard<std::remove_cvref_t<decltype(observer)>,
                                                             tc::drp::sourcing_allowed>) {
            return observer.check(tc::drp::sourcing_allowed{});
        } else {
            return true;
        }
    }

    // Answers the table's try_src_allowed (source preference): Try.SRC
    // where the sink flow would attach, unless the port is locked to
    // the sink role - the sink flow's attach conditions and the lock's
    // answer combined here, so the application object never sees a
    // state
    struct try_gate {
        TypeCDrp& port;

        bool check(tc::drp::try_src_allowed, tc::state::attach_wait_snk const& state)
        {
            return tc::attach_conditions_met::check(state) && port.sourcingAllowed();
        }
    };

    TCPC& tcpc_;
    tc::drp_hw_driver<TCPC, VBUS> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    fsm::QueuedTimer<TIMER> timer_;
    fsm::QueuedTimer<TIMER> deadline_timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
    fsm::deadlined<fsm::QueuedTimer<TIMER>&> deadlined_;
    std::tuple<OBSERVERs&...> observers_;
    try_gate try_gate_{*this};
    fsm::QueuedMachine<tc::drp::table_for_t<TIMING, PREFERENCE>, 4, fsm::inline_work,
                       fsm::no_lock, fsm::timed<fsm::QueuedTimer<TIMER>&>,
                       fsm::deadlined<fsm::QueuedTimer<TIMER>&>,
                       tc::drp_hw_driver<TCPC, VBUS>, tc::vbus_watcher<VBUS>, try_gate,
                       OBSERVERs...>
        sm_;
};

} // namespace usbc
