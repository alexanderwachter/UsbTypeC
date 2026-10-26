/*
 * The facts the VCONN machine publishes: the switch position the
 * injected vconn_port applies, and the VCONN Source role held.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

namespace usbc {

namespace vconn {

// The switch annotation, a strong type: the set's element type is
// the key
struct vconn_switch {
    bool on;
    constexpr bool operator==(vconn_switch const&) const = default;
};

// The VCONN Source role (spec term) held: annotated on the states
// holding it - an accepted swap away still counts, VCONN is on until
// the hand-off - and read off the machine by the facade's
// isVconnSource(). Turning the switch on is not holding the role yet
struct source_role {
    constexpr bool operator==(source_role const&) const = default;
};

} // namespace vconn

} // namespace usbc
