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
 * and the callback's meaning is mapped through the armed level. The
 * client is told which role attached: onAttachedSnk(orientation,
 * advertisement) or onAttachedSrc(orientation), and onDetached().
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

// A DRP client learns which role the port attached in
template<typename T>
concept tc_drp_client = requires(T client, plug_orientation orientation, rp_value advertisement) {
    client.onAttachedSnk(orientation, advertisement);
    client.onAttachedSrc(orientation);
    client.onDetached();
};

} // namespace concepts

namespace tc {

namespace drp {

template<concepts::drp_timing TIMING>
consteval bool timingWithinSpec()
{
    using std::chrono::milliseconds;
    static_assert(milliseconds{50} <= TIMING::t_drp && TIMING::t_drp <= milliseconds{100},
                  "tDRP outside the spec's 50-100 ms");
    static_assert(30u <= TIMING::dc_src && TIMING::dc_src <= 70u,
                  "dcSRC.DRP outside the spec's 30-70 %");
    static_assert(milliseconds{75} <= TIMING::t_drp_try &&
                      TIMING::t_drp_try <= milliseconds{150},
                  "tDRPTry outside the spec's 75-150 ms");
    static_assert(milliseconds{400} <= TIMING::t_drp_try_wait &&
                      TIMING::t_drp_try_wait <= milliseconds{800},
                  "tDRPTryWait outside the spec's 400-800 ms");
    static_assert(milliseconds{10} <= TIMING::t_try_cc_debounce &&
                      TIMING::t_try_cc_debounce <= milliseconds{20},
                  "tTryCCDebounce outside the spec's 10-20 ms");
    static_assert(milliseconds{550} <= TIMING::t_try_timeout &&
                      TIMING::t_try_timeout <= milliseconds{1100},
                  "tTryTimeout outside the spec's 550-1100 ms");
    static_assert(milliseconds{10} <= TIMING::t_pd_debounce &&
                      TIMING::t_pd_debounce <= milliseconds{20},
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
    unattached_src(event::vbus_reached_safe0v const&, src_context& ctx) : source_state(ctx)
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

    try_src_debounce(event::cc_changed const& event, src_context& ctx) : source_state(ctx)
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

    try_wait_src_debounce(event::cc_changed const& event, src_context& ctx) : source_state(ctx)
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
                                       source_flow<TIMING, state::attached_src>>>,
        fsm::transition_table>;
};

template<concepts::drp_timing TIMING>
struct table_for<TIMING, drp_preference::source> {
    static_assert(timingWithinSpec<TIMING>());
    using type = mtl::rebind_t<
        mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                       sink_flow<TIMING, try_src<TIMING>>,
                                       source_flow<TIMING, state::attached_src>,
                                       try_src_flow<TIMING>>>,
        fsm::transition_table>;
};

template<concepts::drp_timing TIMING>
struct table_for<TIMING, drp_preference::sink> {
    static_assert(timingWithinSpec<TIMING>());
    using type = mtl::rebind_t<
        mtl::linearize_t<mtl::typelist<entry_flow<TIMING>,
                                       sink_flow<TIMING, state::attached_snk>,
                                       source_flow<TIMING, try_snk<TIMING>>,
                                       try_snk_flow<TIMING>>>,
        fsm::transition_table>;
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
         concepts::tc_drp_client CLIENT, concepts::drp_timing TIMING = default_drp_timing,
         drp_preference PREFERENCE = drp_preference::none>
class TypeCDrp {
public:
    // Construction rests in Disabled with open terminations; the port
    // goes live on start(). The advertisement is the Rp presented
    // whenever the port advertises the source role
    TypeCDrp(TCPC& tcpc, VBUS& vbus, TIMER& timer, CLIENT& client,
             rp_value advertisement = rp_value::usb_default)
        : tcpc_(tcpc), hw_(tcpc, vbus, advertisement), vbus_(vbus), client_(client), timed_(timer)
    {
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
    // Drains the TCPC's pending alerts; a client providing
    // onPdAlert(alert_status) receives the bits this layer does not
    // consume - the hook for the PD layers above
    void alert()
    {
        if (auto const alerts = tcpc_.readAlert()) {
            if (any(*alerts & alert_status::cc_status_changed)) {
                ccAlert();
            }
            if constexpr (requires { client_.onPdAlert(*alerts); }) {
                auto const residual = *alerts & ~alert_status::cc_status_changed;
                if (any(residual)) {
                    client_.onPdAlert(residual);
                }
            }
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

    // Tells the client about attach results with the attached role,
    // dispatched on the attached state's info type
    struct attach_reporter : fsm::observing<attach_reporter> {
        explicit attach_reporter(CLIENT& client_ref) : client(client_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.attachedInfo()))
        {
            return state.attachedInfo();
        }
        void notifyEntry(tc::attach_info info)
        {
            client.onAttachedSnk(info.orientation, info.advertisement);
        }
        void notifyExit(tc::attach_info) { client.onDetached(); }
        void notifyEntry(plug_orientation orientation) { client.onAttachedSrc(orientation); }
        void notifyExit(plug_orientation) { client.onDetached(); }

        CLIENT& client;
    };

    TCPC& tcpc_;
    tc::drp_hw_driver<TCPC, VBUS> hw_;
    tc::vbus_watcher<VBUS> vbus_;
    CLIENT& client_;
    attach_reporter reporter_{client_};
    fsm::timed<TIMER&> timed_;
    fsm::state_machine<tc::drp::table_for_t<TIMING, PREFERENCE>, fsm::timed<TIMER&>,
                       tc::drp_hw_driver<TCPC, VBUS>, tc::vbus_watcher<VBUS>, attach_reporter>
        sm_{timed_, hw_, vbus_, reporter_};
};

} // namespace usbc
