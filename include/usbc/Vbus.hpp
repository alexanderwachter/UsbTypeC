/*
 * The VBUS block of a USB Type-C port as an integration interface:
 * sensing, the power path switches and the discharge. Hardware
 * frequently senses VBUS with an ADC or comparator outside the port
 * controller and switches it in a power path controller; Zephyr models
 * both as distinct device drivers (usbc_vbus.h, usbc_ppc.h). A TCPI
 * TCPC doing all of it implements this interface alongside the others.
 *
 * The driver is an observer of the connection machine. Every
 * connection state annotates the switch position (vbus_power) and the
 * one condition it waits for (vbus_level) - the level VBUS should (or
 * should no longer) be at in that state; the driver applies the
 * position, arms the condition and reports through the registered
 * callback, typically from a measurement in interrupt context: once
 * with the current condition state as soon as it is known after
 * arming, then on every change, naming the level it reports on. A new
 * level replaces the previous condition; vbus_level::unwatched turns
 * monitoring off. How the driver detects a crossing (comparator, ADC
 * sampling, TCPC voltage alarm) is its own business. Where a decision
 * depends on VBUS right now - a debounce ending with the question
 * whether VBUS is already there - the connection tables ask the
 * questions vbus_at_safe0v / vbus_at_safe5v, and the driver answers
 * them from one measurement, independent of what is monitored.
 *
 * Execution contract: the callback arrives in the stack's serialized
 * context; the hooks and questions are the stack's calls.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>

#include <concepts>
#include <cstdint>

namespace usbc {

// Detection conditions the standard defines on VBUS. "Met" means the
// condition associated with the level holds. unwatched turns
// monitoring off: the driver stops measuring and reports nothing
// until a condition is armed again
enum class vbus_level : std::uint8_t {
    unwatched,          // nothing armed: no measurement, no reports
    safe0v,             // at vSafe0V (max 0.8 V): required before a source
                        // applies VBUS, reached after removal/discharge
    safe5v,             // at vSafe5V (4.75 V - 5.5 V): default VBUS present,
                        // sink-side connection detection
    sink_disconnect,    // below vSinkDisconnect (max 3.67 V): sink-side
                        // disconnection detection at default VBUS
    sink_disconnect_pd, // below vSinkDisconnectPD: disconnection detection
                        // under a PD contract above vSafe5V; the threshold
                        // tracks the negotiated voltage
};

// The switch positions of the VBUS power circuitry, mutually
// exclusive by construction and named for the specification's VBUS
// conditions where one is driven
enum class vbus_path : std::uint8_t {
    open,   // both paths open, the line floats
    sink,   // the sink path draws from VBUS
    safe5v, // the source path applies vSafe5V
    safe0v, // paths open, actively discharging to vSafe0V
};

// The VBUS power annotation - the element that switches power. Every
// connection state carries one, so a role change collapses the old
// role's path, suppressed while unchanged
struct vbus_power {
    vbus_path path = vbus_path::open;
    constexpr bool operator==(vbus_power const&) const = default;
};

// Questions about VBUS right now, asked by the debounce outcomes and
// answered by the driver: the spec requires vSafe0V when a source
// applies power and VBUS present when a sink attaches, at the moment
// of the decision - there is nothing to debounce or remember
struct vbus_at_safe0v {};
struct vbus_at_safe5v {};

using vbus_callback = void (*)(void* context, vbus_level level, bool met);

namespace concepts {

// The driver arrives already initialized. It derives from
// fsm::observing, which is what delivers the annotations to its
// notifyEntry hooks: a class with the hooks alone is silently never
// called
template<typename T>
concept vbus =
    std::derived_from<T, fsm::observing<T>> &&
    requires(T driver, vbus_power power, vbus_level level, vbus_callback callback, void* context) {
        driver.notifyEntry(power);
        driver.notifyEntry(level);
        { driver.check(vbus_at_safe0v{}) } -> std::convertible_to<bool>;
        { driver.check(vbus_at_safe5v{}) } -> std::convertible_to<bool>;
        driver.setCallback(callback, context);
    };

} // namespace concepts

} // namespace usbc
