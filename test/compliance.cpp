/*
 * The stack's compile-time compliance checks, gathered in one
 * dedicated translation unit: every transition table's timeout bounds
 * (against the spec ranges in usbc/Spec.hpp) and reachability, for
 * every variant the library can instantiate. Keeping them here means
 * ordinary translation units pay only for the machines they build,
 * while a clean compile of this file IS the passing verification.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/PdDrp.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCDrp.hpp>
#include <usbc/TypeCSink.hpp>
#include <usbc/TypeCSource.hpp>
#include <usbc/Vconn.hpp>

#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>

namespace compliance {

using namespace usbc;

// --- protocol layer ----------------------------------------------------------

static_assert(fsm::timeouts_within_bounds_v<prl::tx_table, prl::prl_timer_ranges>);
static_assert(fsm::all_states_reachable_v<prl::tx_table>);

// --- Type-C connection layers ------------------------------------------------

static_assert(fsm::timeouts_within_bounds_v<
              tc::sink_table,
              mtl::linearize_t<mtl::typelist<
                  tc::sink_timer_ranges,
                  tc::error_recovery_timer_range,
                  tc::hard_reset_timer_ranges>>>);
static_assert(fsm::all_states_reachable_v<tc::sink_table>);

static_assert(
    fsm::timeouts_within_bounds_v<
        tc::source_table,
        mtl::linearize_t<mtl::typelist<tc::source_timer_ranges, tc::source_recovery_timer_range>>>
);
static_assert(fsm::all_states_reachable_v<tc::source_table>);

// The Try phases are composite states: the proofs cover their
// submachines' states too
template<drp_preference PREFERENCE, typename RANGES>
constexpr bool drpTableChecked()
{
    using table = tc::drp::table_for_t<default_drp_timing, PREFERENCE>;
    static_assert(fsm::timeouts_within_bounds_v<table, RANGES>);
    static_assert(fsm::all_states_reachable_v<table>);
    static_assert(tc::watch_events_consistent_v<table>);
    return true;
}
static_assert(
    drpTableChecked<drp_preference::none, tc::drp::core_timer_ranges<default_drp_timing>>()
);
static_assert(drpTableChecked<
              drp_preference::source,
              mtl::linearize_t<mtl::typelist<
                  tc::drp::core_timer_ranges<default_drp_timing>,
                  tc::drp::try_src_timer_ranges<default_drp_timing>>>>());
static_assert(drpTableChecked<
              drp_preference::sink,
              mtl::linearize_t<mtl::typelist<
                  tc::drp::core_timer_ranges<default_drp_timing>,
                  tc::drp::try_snk_timer_ranges<default_drp_timing>>>>());
static_assert(fsm::levels_v<tc::drp::table_for_t<default_drp_timing, drp_preference::none>> == 1);
static_assert(fsm::levels_v<tc::drp::table_for_t<default_drp_timing, drp_preference::source>> == 2);

static_assert(tc::watch_events_consistent_v<tc::sink_table>);
static_assert(tc::watch_events_consistent_v<tc::source_table>);

// --- policy engines: the everything-on and everything-off corners of the
// --- optional features (PR_Swap, DR_Swap, VCONN), plus each feature alone
// --- so no single filter breaks reachability -----------------------------

// A variant is the table the engine's machine runs with a policy
// enabling exactly these features: a switch observer stands in for it
template<bool PR, bool DR, bool VCONN>
using features = fsm::feature_switch<
    fsm::enabled<pe::pr_swap_feature, PR>,
    fsm::enabled<pe::dr_swap_feature, DR>,
    fsm::enabled<pe::vconn_feature, VCONN>>;

template<typename TABLE, typename TIMER_RANGES, bool PR, bool DR, bool VCONN>
inline constexpr bool variant_ok =
    fsm::timeouts_within_bounds_v<
        fsm::enabled_table_t<TABLE, mtl::typelist<features<PR, DR, VCONN>>>,
        fsm::remove_disabled_features_t<TIMER_RANGES, features<PR, DR, VCONN>>> &&
    fsm::all_states_reachable_v<
        fsm::enabled_table_t<TABLE, mtl::typelist<features<PR, DR, VCONN>>>>;

template<bool PR, bool DR, bool VCONN>
inline constexpr bool sink_variant_ok =
    variant_ok<pe::sink_table, pe::sink_timer_ranges, PR, DR, VCONN>;

template<bool PR, bool DR, bool VCONN>
inline constexpr bool source_variant_ok =
    variant_ok<pe::source_table, pe::source_timer_ranges, PR, DR, VCONN>;

static_assert(sink_variant_ok<true, true, true>);
static_assert(sink_variant_ok<false, false, false>);
static_assert(sink_variant_ok<true, false, false>);
static_assert(sink_variant_ok<false, true, false>);
static_assert(sink_variant_ok<false, false, true>);

static_assert(source_variant_ok<true, true, true>);
static_assert(source_variant_ok<false, false, false>);
static_assert(source_variant_ok<true, false, false>);
static_assert(source_variant_ok<false, true, false>);
static_assert(source_variant_ok<false, false, true>);

// --- vconn machine -----------------------------------------------------------

static_assert(fsm::timeouts_within_bounds_v<vconn::vconn_table, vconn::vconn_timer_ranges>);
static_assert(fsm::all_states_reachable_v<vconn::vconn_table>);

} // namespace compliance
