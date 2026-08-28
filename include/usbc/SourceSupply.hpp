/*
 * The source's VBUS power supply as an integration interface. Programs
 * the voltage/current the port sources; the TCPC merely switches this
 * supply onto VBUS (source_vbus()). Only ports that source power need
 * to provide it.
 *
 * The interface is event-driven like the others: set_output() starts
 * the transition to a new operating point and returns once the driver
 * accepted it; the registered callback (typically interrupt context)
 * reports with at_target = true when the output has settled within
 * tolerance at the target - the policy engine's trigger to send
 * PS_RDY. A later at_target = false reports lost regulation, which the
 * policy engine answers with error recovery. A new set_output()
 * replaces the previous target.
 *
 * On a Request the sequence is: send Accept, wait tSrcTransition,
 * set_output(), await the settled callback, send PS_RDY.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Units.hpp>

#include <concepts>

namespace usbc {

using supply_callback = void (*)(void* context, bool at_target);

namespace concepts {

template<typename T>
concept source_supply = requires(T s, millivolt voltage, milliamp current_limit,
                                 supply_callback callback, void* context) {
    s.set_callback(callback, context);
    { s.set_output(voltage, current_limit) } -> std::same_as<bool>;
};

} // namespace concepts

} // namespace usbc
