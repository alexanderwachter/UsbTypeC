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
 * modelled as sub-states (a state machine state has one timeout):
 *   Try.SRC        = try_src (tDRPTry) + try_src_debounce (tTryCCDebounce)
 *   TryWait.SNK    = try_wait_snk (tDRPTryWait)
 *   Try.SNK        = try_snk (tDRPTry, CC ignored) + try_snk_monitor
 *                    (tTryTimeout - tDRPTry) + try_snk_debounce (tPDDebounce)
 *   TryWait.SRC    = try_wait_src (tDRPTryWait) + try_wait_src_debounce
 *                    (tTryCCDebounce) + try_wait_src_safe0v (vSafe0V wait)
 *
 * Deviations from the spec, accepted for the single-timeout state
 * model: a CC change during a Try debounce restarts the enclosing
 * phase's timer, so a flapping partner extends tDRPTry/tTryTimeout/
 * tDRPTryWait instead of hitting them as hard deadlines; TryWait.SNK
 * attaches on VBUS with a single Rp in the context rather than
 * debouncing the Rp separately; AttachWait exit back to toggling uses
 * tCCDebounce (the spec's shorter tPDDebounce path is not modelled,
 * matching the sink/source layers).
 *
 * Integration matches TypeCSink/TypeCSource: initialized drivers plus
 * a caller-owned timer policy instance; construction rests in the
 * spec's Disabled state and start() goes live in Unattached.SNK. The
 * vbus driver is re-armed per state (vSafe5V in sink-role states,
 * vSafe0V in source-role states, vSinkDisconnect while Attached.SNK)
 * and the callback's meaning is mapped through the armed level.
 * Injected observers watching the attached states' attachedInfo()
 * learn which role attached through the info type: tc::attach_info
 * (orientation and advertisement) for Attached.SNK, plug_orientation
 * for Attached.SRC.
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

// Spec Table 4-30 DRP timing parameters; derive and override members
// to configure, the ranges are enforced at compile time
struct default_drp_timing {
    static constexpr auto t_drp   = std::chrono::milliseconds{75}; // 50 - 100 ms
    static constexpr unsigned dc_src = 50;                         // dcSRC.DRP, 30 - 70 %
    static constexpr auto t_drp_try        = std::chrono::milliseconds{100}; // 75 - 150 ms
    static constexpr auto t_drp_try_wait   = std::chrono::milliseconds{600}; // 400 - 800 ms
    static constexpr auto t_try_cc_debounce = std::chrono::milliseconds{15}; // 10 - 20 ms
    static constexpr auto t_try_timeout    = std::chrono::milliseconds{800}; // 550 - 1100 ms
    static constexpr auto t_pd_debounce    = std::chrono::milliseconds{15};  // 10 - 20 ms
};

namespace concepts {

// An injected observer with the runtime say over PD-directed role
// swaps of the given kind (power_role or data_role), consulted with
// the role the port would swap to. Every such observer may veto; with
// none injected, swaps of that kind are refused
template<typename T, typename ROLE>
concept drp_swap_policy = requires(T policy, ROLE role) {
    { policy.allowSwap(role) } -> std::convertible_to<bool>;
};

template<typename T>
concept drp_timing = requires {
    { T::t_drp } -> std::convertible_to<std::chrono::milliseconds>;
    { T::dc_src } -> std::convertible_to<unsigned>;
    { T::t_drp_try } -> std::convertible_to<std::chrono::milliseconds>;
    { T::t_drp_try_wait } -> std::convertible_to<std::chrono::milliseconds>;
    { T::t_try_cc_debounce } -> std::convertible_to<std::chrono::milliseconds>;
    { T::t_try_timeout } -> std::convertible_to<std::chrono::milliseconds>;
    { T::t_pd_debounce } -> std::convertible_to<std::chrono::milliseconds>;
};

} // namespace concepts

namespace tc {

namespace drp {

template<concepts::drp_timing TIMING>
consteval bool timingWithinSpec()
{
    static_assert(spec::t_drp.contains(TIMING::t_drp), "tDRP outside the spec's 50-100 ms");
    static_assert(spec::within(TIMING::dc_src, spec::dc_src_drp_min, spec::dc_src_drp_max),
                  "dcSRC.DRP outside the spec's 30-70 %");
    static_assert(spec::t_drp_try.contains(TIMING::t_drp_try),
                  "tDRPTry outside the spec's 75-150 ms");
    static_assert(spec::t_drp_try_wait.contains(TIMING::t_drp_try_wait),
                  "tDRPTryWait outside the spec's 400-800 ms");
    static_assert(spec::t_try_cc_debounce.contains(TIMING::t_try_cc_debounce),
                  "tTryCCDebounce outside the spec's 10-20 ms");
    static_assert(spec::t_try_timeout.contains(TIMING::t_try_timeout),
                  "tTryTimeout outside the spec's 550-1100 ms");
    static_assert(spec::t_pd_debounce.contains(TIMING::t_pd_debounce),
                  "tPDDebounce outside the spec's 10-20 ms");
    return true;
}

// The Rp slice of the toggle period; the Rd slice is the remainder
template<typename TIMING>
inline constexpr std::chrono::milliseconds t_src_slice = TIMING::t_drp * TIMING::dc_src / 100;

// --- toggling unattached states ---------------------------------------------

// Unattached.SNK of a DRP: Rd presented for the sink slice of tDRP
template<typename TIMING>
struct unattached_snk : state::sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = TIMING::t_drp - t_src_slice<TIMING>;

    using state::sink_state::sink_state;
};

// Unattached.SRC of a DRP: Rp presented for the source slice of tDRP
template<typename TIMING>
struct unattached_src : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = t_src_slice<TIMING>;

    // entering on the discharge-complete event records what it means
    unattached_src(event::vbus_reached_safe0v const&, port_context& ctx) : source_state(ctx)
    {
        context.vbus_safe0v = true;
    }
    using state::source_state::source_state;
};

// --- Try.SRC / TryWait.SNK (drp_preference::source) --------------------------

// Rp presented where the sink flow would have attached; waits tDRPTry
// for the partner's Rd
template<typename TIMING>
struct try_src : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = TIMING::t_drp_try;

    using state::source_state::source_state;
};

// A single Rd appeared in Try.SRC: stable for tTryCCDebounce attaches
template<typename TIMING>
struct try_src_debounce : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = TIMING::t_try_cc_debounce;

    try_src_debounce(event::cc_changed const& event, port_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::source_state::source_state;
};

// The partner did not present Rd: back to Rd for tDRPTryWait, attaching
// as sink when the partner sources VBUS
template<typename TIMING>
struct try_wait_snk : state::sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = TIMING::t_drp_try_wait;

    using state::sink_state::sink_state;
};

// --- Try.SNK / TryWait.SRC (drp_preference::sink) ----------------------------

// Rd presented where the source flow would have attached; the spec
// mandates waiting tDRPTry before the CC pins are even monitored
template<typename TIMING>
struct try_snk : state::sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = TIMING::t_drp_try;

    using state::sink_state::sink_state;
};

// Monitoring phase of Try.SNK: the tTryTimeout budget minus the wait
template<typename TIMING>
struct try_snk_monitor : state::sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = TIMING::t_try_timeout - TIMING::t_drp_try;

    using state::sink_state::sink_state;
};

// A single Rp appeared in Try.SNK: stable for tPDDebounce with VBUS
// present attaches
template<typename TIMING>
struct try_snk_debounce : state::sink_state {
    static constexpr hw_config hw{cc_pull::rd, false};
    static constexpr vbus_level watch = vbus_level::safe5v;
    static constexpr auto timeout     = TIMING::t_pd_debounce;

    try_snk_debounce(event::cc_changed const& event, port_context& ctx) : sink_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::sink_state::sink_state;
};

// The partner did not present Rp: back to Rp for tDRPTryWait
template<typename TIMING>
struct try_wait_src : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = TIMING::t_drp_try_wait;

    using state::source_state::source_state;
};

// A single Rd appeared in TryWait.SRC: stable for tTryCCDebounce
// attaches once VBUS is at vSafe0V
template<typename TIMING>
struct try_wait_src_debounce : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;
    static constexpr auto timeout     = TIMING::t_try_cc_debounce;

    try_wait_src_debounce(event::cc_changed const& event, port_context& ctx) : source_state(ctx)
    {
        context.cc = event.cc;
    }
    using state::source_state::source_state;
};

// Rd debounced but VBUS not yet at vSafe0V: attach follows the report
template<typename TIMING>
struct try_wait_src_safe0v : state::source_state {
    static constexpr src_hw_config hw{.pull = cc_pull::rp, .source = false, .discharge = false};
    static constexpr vbus_level watch = vbus_level::safe0v;

    using state::source_state::source_state;
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

// --- timer-range maps --------------------------------------------------------

// Budget left for Try.SNK's monitoring phase after the blind tDRPTry wait
inline constexpr fsm::timeout_range t_try_monitor{
    spec::t_try_timeout.min - spec::t_drp_try.max,
    spec::t_try_timeout.max - spec::t_drp_try.min};

// Composed per preference like the flows they check; the shared attach
// flows bring the sink and source maps along
template<typename TIMING>
using core_timer_ranges = mtl::linearize_t<mtl::typelist<
    sink_timer_ranges, source_timer_ranges,
    fsm::timed_by<unattached_snk<TIMING>, spec::t_drp_pw>,
    fsm::timed_by<unattached_src<TIMING>, spec::t_drp_pw>>>;

template<typename TIMING>
using try_src_timer_ranges = mtl::typelist<
    fsm::timed_by<try_src<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_src_debounce<TIMING>, spec::t_try_cc_debounce>,
    fsm::timed_by<try_wait_snk<TIMING>, spec::t_drp_try_wait>>;

template<typename TIMING>
using try_snk_timer_ranges = mtl::typelist<
    fsm::timed_by<try_snk<TIMING>, spec::t_drp_try>,
    fsm::timed_by<try_snk_monitor<TIMING>, t_try_monitor>,
    fsm::timed_by<try_snk_debounce<TIMING>, spec::t_pd_debounce>,
    fsm::timed_by<try_wait_src<TIMING>, spec::t_drp_try_wait>,
    fsm::timed_by<try_wait_src_debounce<TIMING>, spec::t_try_cc_debounce>>;

// --- transition table composition --------------------------------------------

// Sink-role flow: the sink layer's attach flow anchored at the
// toggling Unattached.SNK, plus the toggle to the Rp phase; SNK_ATTACH
// is where a successful attach leads - Attached.SNK, or Try.SRC for a
// source-preferring port
template<typename TIMING, typename SNK_ATTACH>
using sink_flow = mtl::concat_t<
    mtl::typelist<fsm::transition<fsm::from<unattached_snk<TIMING>>, fsm::on<fsm::timeout>,
                                  fsm::to<unattached_src<TIMING>>>>,
    sink_attach_flow<unattached_snk<TIMING>, SNK_ATTACH>>;

// Source-role flow; SRC_ATTACH is Attached.SRC, or Try.SNK for a
// sink-preferring port
template<typename TIMING, typename SRC_ATTACH>
using source_flow = mtl::concat_t<
    mtl::typelist<fsm::transition<fsm::from<unattached_src<TIMING>>, fsm::on<fsm::timeout>,
                                  fsm::to<unattached_snk<TIMING>>>>,
    source_attach_flow<unattached_src<TIMING>, SRC_ATTACH>>;

template<typename TIMING>
using try_src_flow = mtl::typelist<
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_src_debounce<TIMING>>, fsm::guard<rd_on_event>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_src<TIMING>>, fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_src<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_snk<TIMING>>>,
    // a CC change restarts the Rd debounce; Rd gone at the timeout
    // falls back to Try.SRC
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_src_debounce<TIMING>>>,
    fsm::internal_transition<fsm::from<try_src_debounce<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_src_debounce<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_src>, fsm::guard<rd_in_context>>,
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

template<typename TIMING>
using try_snk_flow = mtl::typelist<
    // the CC pins are not monitored during the initial tDRPTry wait
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_snk<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_in_context>>,
    fsm::transition<fsm::from<try_snk<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_on_event>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_present>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_in_context>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_snk_monitor<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src<TIMING>>>,
    // a CC change keeping the Rp restarts the debounce, losing it
    // resumes monitoring
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_debounce<TIMING>>, fsm::guard<rp_on_event>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    fsm::internal_transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::vbus_present>>,
    fsm::internal_transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<event::vbus_removed>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_snk>, fsm::guard<rp_in_context_with_vbus>>,
    fsm::transition<fsm::from<try_snk_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_snk_monitor<TIMING>>>,
    // TryWait.SRC
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src_debounce<TIMING>>, fsm::guard<rd_on_event>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::cc_changed>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_wait_src<TIMING>>, fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_wait_src<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<unattached_snk<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src_debounce<TIMING>>>,
    fsm::internal_transition<fsm::from<try_wait_src_debounce<TIMING>>,
                             fsm::on<event::vbus_reached_safe0v>>,
    fsm::internal_transition<fsm::from<try_wait_src_debounce<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<state::attached_src>, fsm::guard<rd_in_context_at_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src_safe0v<TIMING>>, fsm::guard<rd_in_context>>,
    fsm::transition<fsm::from<try_wait_src_debounce<TIMING>>, fsm::on<fsm::timeout>,
                    fsm::to<try_wait_src<TIMING>>>,
    fsm::transition<fsm::from<try_wait_src_safe0v<TIMING>>, fsm::on<event::vbus_reached_safe0v>,
                    fsm::to<state::attached_src>>,
    fsm::internal_transition<fsm::from<try_wait_src_safe0v<TIMING>>,
                             fsm::on<event::vbus_left_safe0v>>,
    fsm::transition<fsm::from<try_wait_src_safe0v<TIMING>>, fsm::on<event::cc_changed>,
                    fsm::to<try_wait_src<TIMING>>>>;

// PD-directed role swaps (spec: Attached.SNK <-> Attached.SRC "as
// directed by USB PD"): a power swap flips the terminations while the
// pair stays attached, a data role swap changes no terminations and
// only flips the context's data role. VBUS sequencing and the decision
// to swap are the PD layer's business, gated by the injected policy
// observers
using swap_flow = mtl::typelist<
    fsm::transition<fsm::from<state::attached_snk>, fsm::on<event::swap_to_source>,
                    fsm::to<state::attached_src>>,
    fsm::transition<fsm::from<state::attached_src>, fsm::on<event::swap_to_sink>,
                    fsm::to<state::attached_snk>>,
    fsm::internal_transition<fsm::from<state::attached_snk>, fsm::on<event::swap_data_role>>,
    fsm::internal_transition<fsm::from<state::attached_src>, fsm::on<event::swap_data_role>>>;

// The port rests in the sink layer's Disabled state and goes live
// toggling at Rd
template<typename TIMING>
using entry_flow = mtl::typelist<
    fsm::initial<state::disabled_snk>,
    fsm::transition<fsm::from<state::disabled_snk>, fsm::on<event::started>,
                    fsm::to<unattached_snk<TIMING>>>>;

template<concepts::drp_timing TIMING, drp_preference PREFERENCE>
struct table_for {
    static_assert(timingWithinSpec<TIMING>());
    using type = mtl::rebind_t<
        mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                       sink_flow<TIMING, state::attached_snk>,
                                       source_flow<TIMING, state::attached_src>, swap_flow>>,
        fsm::transition_table>;
    static_assert(fsm::timeouts_within_bounds_v<type, core_timer_ranges<TIMING>>);
    static_assert(fsm::all_states_reachable_v<type>);
};

template<concepts::drp_timing TIMING>
struct table_for<TIMING, drp_preference::source> {
    static_assert(timingWithinSpec<TIMING>());
    using type = mtl::rebind_t<
        mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                       sink_flow<TIMING, try_src<TIMING>>,
                                       source_flow<TIMING, state::attached_src>,
                                       try_src_flow<TIMING>, swap_flow>>,
        fsm::transition_table>;
    static_assert(fsm::timeouts_within_bounds_v<
                  type, mtl::linearize_t<mtl::typelist<core_timer_ranges<TIMING>,
                                                       try_src_timer_ranges<TIMING>>>>);
    static_assert(fsm::all_states_reachable_v<type>);
};

template<concepts::drp_timing TIMING>
struct table_for<TIMING, drp_preference::sink> {
    static_assert(timingWithinSpec<TIMING>());
    using type = mtl::rebind_t<
        mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                       sink_flow<TIMING, state::attached_snk>,
                                       source_flow<TIMING, try_snk<TIMING>>,
                                       try_snk_flow<TIMING>, swap_flow>>,
        fsm::transition_table>;
    static_assert(fsm::timeouts_within_bounds_v<
                  type, mtl::linearize_t<mtl::typelist<core_timer_ranges<TIMING>,
                                                       try_snk_timer_ranges<TIMING>>>>);
    static_assert(fsm::all_states_reachable_v<type>);
};

template<concepts::drp_timing TIMING, drp_preference PREFERENCE>
using table_for_t = typename table_for<TIMING, PREFERENCE>::type;

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
    // previous state's terminations would stay applied. notified_of
    // also proves each annotation is one of the two role configs the
    // entry hooks accept
    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(fsm::all_states_notified_v<drp_hw_driver, TABLE>,
                      "drp_hw_driver: every state must annotate a role hw config");
    }

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::hw)
    {
        return STATE::hw;
    }
    void notifyEntry(hw_config const& config) // a sink-role state
    {
        tcpc.setCc(config.pull, rp);
        tcpc.sourceVbus(false);
        vbus.discharge(false);
        tcpc.sinkVbus(config.sink);
    }
    void notifyEntry(src_hw_config const& config) // a source-role state
    {
        tcpc.setCc(config.pull, rp);
        tcpc.sinkVbus(false);
        tcpc.sourceVbus(config.source);
        vbus.discharge(config.discharge);
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.orientation()))
    {
        return state.orientation();
    }
    void notifyEntry(plug_orientation orientation) { tcpc.setPlugOrientation(orientation); }

    TCPC& tcpc;
    VBUS& vbus;
    rp_value rp;
};

} // namespace tc

template<concepts::tcpc TCPC, concepts::vbus VBUS, fsm::concepts::timer TIMER,
         concepts::drp_timing TIMING = default_drp_timing,
         drp_preference PREFERENCE = drp_preference::none, typename... OBSERVERs>
class TypeCDrp {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The advertisement is the Rp presented
    // whenever the port advertises the source role. The observers are
    // injected into the machine after the built-in ones (timer, hw
    // driver, vbus watcher); attach results are observed on the
    // attached states' attachedInfo() with the role encoded in the
    // info type, an observer providing onPdAlert(alert_status)
    // receives the alert bits this layer does not consume, and one
    // providing allowSwap(power_role) is a swap policy
    TypeCDrp(TCPC& tcpc, VBUS& vbus, TIMER& timer, rp_value advertisement,
             OBSERVERs&... observers)
        : tcpc_(tcpc), hw_(tcpc, vbus, advertisement), vbus_(vbus), timed_(timer),
          observers_(observers...), sm_(timed_, hw_, vbus_, observers...)
    {
    }
    // Default-Rp convenience: a trailing pack cannot follow a defaulted
    // advertisement
    TypeCDrp(TCPC& tcpc, VBUS& vbus, TIMER& timer, OBSERVERs&... observers)
        : TypeCDrp(tcpc, vbus, timer, rp_value::usb_default, observers...)
    {
    }

    // Power role swap directed by the layer above (a USB PD PR_Swap):
    // the pair stays attached and the terminations flip - VBUS
    // sequencing is the caller's business. Every injected observer
    // providing allowSwap(power_role) is consulted and may veto; with
    // no such observer, swaps are refused. False when vetoed or not
    // attached in the departing role. Call from the stack's serialized
    // context
    bool swapToSource()
    {
        if (!sm_.template is<tc::state::attached_snk>() || !swapAllowed(power_role::source)) {
            return false;
        }
        return sm_.process(tc::event::swap_to_source{});
    }

    bool swapToSink()
    {
        if (!sm_.template is<tc::state::attached_src>() || !swapAllowed(power_role::sink)) {
            return false;
        }
        return sm_.process(tc::event::swap_to_sink{});
    }

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
        if (!sm_.process(tc::event::swap_data_role{})) {
            return false;
        }
        auto const swapped = *dataRole();
        std::apply([&](auto&... observer) { (forwardDataRole(observer, swapped), ...); },
                   observers_);
        return true;
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

    // Leaves Disabled toggling at Rd: terminations and monitoring apply
    // through the machine, the callbacks register, and a present
    // partner is seeded from the CC status. A second start() finds no
    // started transition and does nothing
    void start()
    {
        if (!sm_.process(tc::event::started{})) {
            return;
        }
        vbus_.vbus.setCallback(
            [](void* self, bool met) { static_cast<TypeCDrp*>(self)->vbusEvent(met); }, this);
        tcpc_.setAlertHandler([](void* self) { static_cast<TypeCDrp*>(self)->alert(); }, this);
        vbus_.vbus.monitor(vbus_.monitored); // deliver the initial condition
        seedCcState();
    }

private:
    // Drains the TCPC's pending alerts; the bits this layer does not
    // consume go to the observers providing onPdAlert(alert_status)
    void alert()
    {
        if (auto const alerts = tcpc_.readAlert()) {
            if (any(*alerts & alert_status::cc_status_changed)) {
                ccAlert();
            }
            auto const residual = *alerts & ~alert_status::cc_status_changed;
            if (any(residual)) {
                std::apply([&](auto&... observer) { (forwardPdAlert(observer, residual), ...); },
                           observers_);
            }
        }
    }

    static void forwardPdAlert(auto& observer, alert_status alerts)
    {
        if constexpr (requires { observer.onPdAlert(alerts); }) {
            observer.onPdAlert(alerts);
        }
    }

    void ccAlert()
    {
        if (auto const cc = tcpc_.readCcStatus()) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // The vbus driver reports the level the active state watches; the
    // event family follows the armed level's role
    void vbusEvent(bool met)
    {
        switch (vbus_.monitored) {
        case vbus_level::safe0v:
            if (met) {
                sm_.process(tc::event::vbus_reached_safe0v{});
            } else {
                sm_.process(tc::event::vbus_left_safe0v{});
            }
            break;
        case vbus_level::sink_disconnect:
        case vbus_level::sink_disconnect_pd:
            if (met) {
                sm_.process(tc::event::vbus_removed{});
            } else {
                sm_.process(tc::event::vbus_present{});
            }
            break;
        case vbus_level::safe5v:
            if (met) {
                sm_.process(tc::event::vbus_present{});
            } else {
                sm_.process(tc::event::vbus_removed{});
            }
            break;
        }
    }

    // A partner plugged in before construction has no alert to announce it
    void seedCcState()
    {
        auto const cc = tcpc_.readCcStatus();
        if (cc && (tc::isRp(cc->cc1) || tc::isRp(cc->cc2) || tc::isRd(cc->cc1) ||
                   tc::isRd(cc->cc2))) {
            sm_.process(tc::event::cc_changed{*cc});
        }
    }

    // Every policy observer for the role kind is consulted and each
    // may veto; with no such observer among the injected ones there is
    // nobody to say yes, and swaps of that kind are refused
    template<typename ROLE>
    bool swapAllowed(ROLE role)
    {
        constexpr bool any_policy =
            (concepts::drp_swap_policy<std::remove_cvref_t<OBSERVERs>, ROLE> || ...);
        return any_policy && std::apply(
                                 [role](auto&... observer) {
                                     return (allowsSwap(observer, role) && ...);
                                 },
                                 observers_);
    }

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

    TCPC& tcpc_;
    tc::drp_hw_driver<TCPC, VBUS> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    fsm::timed<TIMER&> timed_;
    std::tuple<OBSERVERs&...> observers_;
    fsm::state_machine<tc::drp::table_for_t<TIMING, PREFERENCE>, fsm::timed<TIMER&>,
                       tc::drp_hw_driver<TCPC, VBUS>, tc::vbus_watcher<VBUS>, OBSERVERs...>
        sm_;
};

} // namespace usbc
