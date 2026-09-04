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

} // namespace event

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
    }

    VBUS& vbus;
    vbus_level monitored = vbus_level::safe5v;
};

} // namespace tc

} // namespace usbc
