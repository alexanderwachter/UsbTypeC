/*
 * Observer logging every state change of the machine it is injected
 * into to the Zephyr logging system (module usbc_fsm, debug level -
 * transitions are frequent, a toggling DRP fires several per tDRP).
 * Inject after the built-in observers so the log line follows the
 * effects of the change:
 *
 *   usbc::zephyr::StateLogger state_logger;
 *   Sink sink{tcpc, vbus, timer, logger, state_logger};
 *
 * The state names are compile-time null-terminated strings in static
 * storage, so deferred logging needs no string duplication.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/TypeName.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

namespace usbc::zephyr {

// Out of line: the logging module lives in StateLogger.cpp
void logInitialState(char const* state);
void logStateChange(char const* from, char const* to);

struct StateLogger {
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterState(MACHINE&)
    {
        if constexpr (std::is_same_v<OLD_STATE, mtl::nil_type>) {
            logInitialState(mtl::short_name_of<NEW_STATE>);
        } else {
            logStateChange(mtl::short_name_of<OLD_STATE>, mtl::short_name_of<NEW_STATE>);
        }
    }
};

} // namespace usbc::zephyr
