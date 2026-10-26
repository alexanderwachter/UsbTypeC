/*
 * The commands to the USB PD protocol layer, as the policy engines'
 * states annotate them: the engine's layer port, an observer of the
 * engine's machine, executes them on the layer on entry - for the
 * layer's client only. The note strings feed the states' dot_action
 * diagram labels.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <string_view>

namespace usbc {

namespace prl {

struct reset_action {
    static constexpr std::string_view note = "resets the protocol layer";
    // re-resetting an already reset layer changes nothing: a wildcard's
    // entry may re-notify it
    constexpr bool operator==(reset_action const&) const = default;
};
struct hard_reset_action { // not repeatable: a notification transmits
    static constexpr std::string_view note = "requests a hard reset";
    constexpr bool operator==(hard_reset_action const&) const = default;
};

} // namespace prl

} // namespace usbc
