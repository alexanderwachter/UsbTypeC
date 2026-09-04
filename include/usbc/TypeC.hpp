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

    VBUS& vbus;
    vbus_level monitored = vbus_level::safe5v;
};

} // namespace tc

} // namespace usbc
