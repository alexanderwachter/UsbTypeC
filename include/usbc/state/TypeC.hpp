/*
 * Common ground of the USB Type-C connection tables: the spec's
 * debounce times, the events the connection machines take, the
 * attachment context, the CC debounce rule the states annotate (the
 * CcWatcher of layer/TypeC.hpp applies it), and the check that every
 * state declares what the hardware blocks observe. The facts the
 * states publish live in annotation/TypeC.hpp; the hardware itself -
 * the CC block (interface/Cc.hpp) and the VBUS block
 * (interface/Vbus.hpp) - observes the states' annotations directly.
 * The sink, source and DRP tables live in TypeCSink.hpp,
 * TypeCSource.hpp and TypeCDrp.hpp next to this file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Cc.hpp>
#include <usbc/interface/Vbus.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Parameters.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <cstdint>
#include <type_traits>

namespace usbc {

namespace tc {

// One debounce time of a state's CC rule: how long a status has to
// hold before it counts as detected - a spec time (Table 4-33) as its
// count of milliseconds, or at_once. An enum: a byte per time on every
// monitoring state, and the DOT graphs show the name
enum class debounce_time : std::uint8_t {
    at_once = 0,
    t_pd_debounce = 15,
    t_try_cc_debounce = 15,
    t_src_disconnect = 15,
    t_cc_debounce = 150,
};

constexpr std::chrono::milliseconds durationOf(debounce_time time)
{
    return std::chrono::milliseconds{static_cast<std::underlying_type_t<debounce_time>>(time)};
}

inline constexpr auto t_cc_debounce = durationOf(debounce_time::t_cc_debounce);
inline constexpr auto t_pd_debounce = durationOf(debounce_time::t_pd_debounce);
inline constexpr auto t_try_cc_debounce = durationOf(debounce_time::t_try_cc_debounce);
inline constexpr auto t_src_disconnect = durationOf(debounce_time::t_src_disconnect);
inline constexpr auto t_error_recovery = std::chrono::milliseconds{50};      // tErrorRecovery
inline constexpr auto t_hard_reset_window = std::chrono::milliseconds{6000}; // > tNoResponse

namespace event {

// The CC status that held for the state's debounce (CcWatcher's
// report; the spec's "detected"), the only form in which the CC pins
// reach the connection machines
struct cc_detected {
    cc_status cc;
};
struct started {}; // start(): leave Disabled, apply the terminations

// The vbus driver's report, mapped through the level the watcher
// armed: presence for the sink-role levels (vSafe5V, vSinkDisconnect),
// the discharge condition for vSafe0V
struct vbus_present {};
struct vbus_removed {};
struct vbus_reached_safe0v {};
struct vbus_left_safe0v {};

// Role swaps directed by the layer above (USB PD PR_Swap/DR_Swap),
// DRP only: the pair stays attached and the plug orientation carries
// over in the shared context. A power swap runs in two phases - the
// swap_to_* events enter a standby with the power paths off and
// detach detection suspended (VBUS is legitimately absent while the
// roles change hands), swap_complete lands in the new attached state
// on the partner's PS_RDY, and swap_abort restores the departing
// role. A data role swap changes no terminations at all - it only
// flips the context's data role
struct swap_to_source {};
struct swap_to_sink {};
struct swap_complete {};
struct swap_abort {};
struct swap_data_role {};
struct error_recovery {}; // PD-directed: remove both terminations
struct hard_reset {};     // PD-directed: open the hard-reset window

} // namespace event

// The one machine-owned context of the connection layer: one
// connection's attachment. The detected CC status the attach rests on
// (the states waiting for a VBUS report after a detection keep it here
// for the Attached state to resolve plug orientation and the Rp
// advertisement from, and the attach fact reports it), the plug
// orientation and data role a fresh attach resolves, kept across a
// hard-reset window and a power role swap (a power swap leaves the
// data role alone, per the PD spec). The Unattached anchors reset it;
// `resolved` tells Attached.SNK whether VBUS returning resumes the
// connection or attaches afresh. The CC status itself is no context:
// the decision and the status arrive together in event::cc_detected
struct attachment {
    cc_status detected = {.cc1 = cc_state::snk_open, .cc2 = cc_state::snk_open};
    plug_orientation orientation = plug_orientation::cc1;
    data_role data = data_role::ufp;
    bool resolved = false;
};

// The one place tying a VBUS level to the events its reports become:
// the sink-role levels report presence, vSafe0V reports the discharge
// condition - and on the below-threshold levels a met report means the
// bus is gone. The ports' report mapping and the verification below
// both consume this
enum class vbus_family : std::uint8_t { presence, discharge };

constexpr vbus_family familyOf(vbus_level level)
{
    return level == vbus_level::safe0v ? vbus_family::discharge : vbus_family::presence;
}

constexpr bool metMeansPresent(vbus_level level)
{
    return level == vbus_level::safe5v;
}

// The report the driver delivers when the armed condition is met: the
// event a state arms the level for
template<vbus_level LEVEL>
using met_event_t = std::conditional_t<
    familyOf(LEVEL) == vbus_family::discharge,
    fsm::events<event::vbus_reached_safe0v>,
    std::conditional_t<
        metMeansPresent(LEVEL),
        fsm::events<event::vbus_present>,
        fsm::events<event::vbus_removed>>>;

// The event a state owes for the VBUS level it watches: the met
// report of its level - a state arming a condition without a
// transition on its report waits for nothing (a lost detach at worst).
// A state without the annotation, or with monitoring off, owes
// nothing; fsm::all_states_handle is the table-wide proof
template<fsm::concepts::state STATE>
struct required_vbus_events : std::type_identity<fsm::events<>> {};

template<fsm::concepts::state STATE>
    requires requires { STATE::annotations.template get<vbus_level>(); } &&
             (STATE::annotations.template get<vbus_level>() != vbus_level::unwatched)
struct required_vbus_events<STATE>
    : std::type_identity<met_event_t<STATE::annotations.template get<vbus_level>()>> {};

template<fsm::concepts::transition_table TABLE>
inline constexpr bool watch_events_consistent_v =
    fsm::all_states_handle_v<TABLE, required_vbus_events>;

// What the hardware blocks observe, declared by every connection state:
// the CC termination and the VBUS switch position (a state the
// dispatch would silently skip would leave the previous state's
// terminations or power paths applied) and the watched VBUS level
// (vbus_level::unwatched is the deliberate "monitoring off", never an
// omission), with a transition on the level's met report. The ports
// assert it on their tables
template<fsm::concepts::transition_table TABLE>
inline constexpr bool hardware_annotations_complete_v =
    fsm::all_states_carry_v<TABLE, cc_termination> && fsm::all_states_carry_v<TABLE, vbus_power> &&
    fsm::all_states_carry_v<TABLE, vbus_level> && watch_events_consistent_v<TABLE>;

// --- CC debounce -------------------------------------------------------------

// Annotation of a connection state monitoring the CC pins - its rule:
// the debounce time of a termination (a source's Rp seen through Rd, a
// sink's Rd or Ra seen through Rp) and of the open line. CcWatcher
// applies it and reports the status that held as event::cc_detected; a
// state without the annotation does not monitor the pins
struct cc_debounce {
    debounce_time termination;
    debounce_time open;
    constexpr bool operator==(cc_debounce const&) const = default;
};

// Guard question answered by CcWatcher: a CC status is under debounce
// right now. The DRP's Try walls apply only while the wanted
// termination "has not yet been detected"
struct cc_state_pending {};

// The debounce times applied, within the spec's ranges
static_assert(spec::t_pd_debounce.contains(t_pd_debounce));
static_assert(spec::t_try_cc_debounce.contains(t_try_cc_debounce));
static_assert(spec::t_src_disconnect.contains(t_src_disconnect));
static_assert(spec::t_cc_debounce.contains(t_cc_debounce));

} // namespace tc

} // namespace usbc
