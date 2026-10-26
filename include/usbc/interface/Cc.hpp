/*
 * The CC block of a USB Type-C port as an integration interface: the
 * termination presented on the CC pins and what the pins read. The
 * driver is an observer of the connection machine - every connection
 * state annotates the termination it presents (cc_termination) and the
 * driver applies it, pulling up with the Rp advertisement the port set
 * through setRp(). What the pins read reaches the stack through the
 * registered callback on every change, undebounced: the CcWatcher
 * (layer/TypeC.hpp) debounces by the state's rule, which is the connection
 * layer's business, not the hardware's. readCcStatus() is one
 * measurement, taken at start() for a partner plugged in before - that
 * line changes nothing, so nothing announces it.
 *
 * Execution contract: the callback arrives in the stack's serialized
 * context (the Zephyr adapter delivers on the stack's workqueue); the
 * hooks and functions are called from the stack's context only.
 *
 * Failure contract: bool functions report whether the driver accepted
 * the operation (e.g. the bus transfer succeeded); readCcStatus()
 * returns std::nullopt on failure.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>

#include <concepts>
#include <cstdint>
#include <optional>

namespace usbc {

// Termination presented on the CC lines; one value for both lines,
// per-line asymmetry is not required of drivers
enum class cc_pull : std::uint8_t { open, ra, rp, rd };

// Current advertisement while presenting Rp
enum class rp_value : std::uint8_t { usb_default, p_1a5, p_3a0 };

// Voltage state of one CC line; src_* apply while the line presents Rp,
// snk_* while it presents Rd
enum class cc_state : std::uint8_t {
    src_open,
    src_ra,
    src_rd,
    snk_open,
    snk_default,
    snk_power_1a5,
    snk_power_3a0,
};

struct cc_status {
    cc_state cc1;
    cc_state cc2;
    constexpr bool operator==(cc_status const&) const = default;
};

// The CC line carrying the CC wire of the attached cable; VCONN, when
// enabled, is applied to the other line
enum class plug_orientation : std::uint8_t { cc1, cc2 };

// The presented CC termination: the connection states' annotation, the
// driver applies it (a pull-up with the port's Rp advertisement)
struct cc_termination {
    cc_pull pull;
    constexpr bool operator==(cc_termination const&) const = default;
};

using cc_callback = void (*)(void* context, cc_status cc);

namespace concepts {

// The driver arrives already initialized - bring-up is the
// integrator's job. It derives from fsm::observing, which is what
// delivers the annotation to its notifyEntry: a class with the hook
// alone is silently never called
template<typename T>
concept cc = requires(
                 T driver,
                 cc_termination termination,
                 rp_value rp,
                 cc_callback callback,
                 void* context
             ) {
    driver.notifyEntry(termination);
    { driver.setRp(rp) } -> std::same_as<bool>;
    driver.setCallback(callback, context);
    { driver.readCcStatus() } -> std::same_as<std::optional<cc_status>>;
} && std::derived_from<T, fsm::observing<T>>;

} // namespace concepts

} // namespace usbc
