/*
 * Common ground of the USB Type-C connection layers: the CC debounce
 * time, the CC change event, and the vbus_watcher observer arming the
 * vbus driver with each state's watched level. The sink and source
 * layers live in TypeCSink.hpp and TypeCSource.hpp.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Spec.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>

namespace usbc {

namespace tc {

inline constexpr auto t_cc_debounce = std::chrono::milliseconds{150}; // 100 ms - 200 ms

namespace event {

struct cc_changed {
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

} // namespace event

// Machine-owned context shared by every connection-layer state, across
// both roles: the latest CC status (interpreted through the presented
// pull), the vbus conditions, and the resolved plug orientation and
// data role - kept here so they survive a role swap (a power swap
// leaves the data role alone, per the PD spec)
struct port_context {
    cc_status cc{cc_state::snk_open, cc_state::snk_open};
    bool vbus_present = false;
    bool vbus_safe0v  = false;
    plug_orientation orientation = plug_orientation::cc1;
    data_role data               = data_role::ufp;
};

// A watching state must consume the event family its armed level
// makes the driver deliver - a missing transition would silently drop
// a report (a lost detach at worst)
template<typename TABLE>
struct watch_family_handled {
    template<typename STATE, bool WATCHING = requires { STATE::watch; }>
    struct pred
        : std::bool_constant<
              STATE::watch == vbus_level::safe0v
                  ? (fsm::handles_event_v<TABLE, STATE, event::vbus_reached_safe0v> &&
                     fsm::handles_event_v<TABLE, STATE, event::vbus_left_safe0v>)
                  : (fsm::handles_event_v<TABLE, STATE, event::vbus_present> &&
                     fsm::handles_event_v<TABLE, STATE, event::vbus_removed>)> {};

    // an unwatched state gets no reports and implies nothing
    template<typename STATE>
    struct pred<STATE, false> : std::true_type {};
};

template<typename TABLE>
struct watch_events_consistent
    : std::bool_constant<mtl::all_of_v<typename TABLE::states,
                                       watch_family_handled<TABLE>::template pred>> {};

template<typename TABLE>
inline constexpr bool watch_events_consistent_v = watch_events_consistent<TABLE>::value;

// Arms the vbus driver with each state's watched level; the class maps
// the callback's meaning through the level it armed last
template<concepts::vbus VBUS>
struct vbus_watcher : fsm::observing<vbus_watcher<VBUS>> {
    explicit vbus_watcher(VBUS& vbus_ref) : vbus(vbus_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::watch)
    {
        return STATE::watch;
    }
    void notifyEntry(vbus_level level)
    {
        monitored = level;
        vbus.monitor(level);
    }

    // Only a state presenting no terminations (Disabled, ErrorRecovery)
    // may go unwatched: nothing can attach while the pull is open
    template<typename STATE, bool HAS_HW = requires { STATE::hw; }>
    struct idle_without_watch : std::bool_constant<STATE::hw.pull == cc_pull::open> {};

    template<typename STATE>
    struct idle_without_watch<STATE, false> : std::false_type {};

    template<typename STATE>
    struct watched_or_idle : std::bool_constant<fsm::is_notified_of_v<vbus_watcher, STATE> ||
                                                idle_without_watch<STATE>::value> {};

    template<fsm::concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states, watched_or_idle>,
                      "vbus_watcher: a state presenting terminations must watch a VBUS level");
        static_assert(watch_events_consistent_v<TABLE>,
                      "vbus_watcher: a watching state must handle its level's event family");
    }

    VBUS& vbus;
    vbus_level monitored = vbus_level::safe5v;
};

} // namespace tc

} // namespace usbc
