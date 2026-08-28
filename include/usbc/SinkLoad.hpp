/*
 * The sink's power consumer as an integration interface - typically
 * the input limit of a charger IC or load switch. Only ports that sink
 * power need to provide it.
 *
 * The stack programs what the load may draw and what input voltage to
 * expect: from the Rp advertisement before an explicit contract
 * (vSafe5V at default/1.5 A/3.0 A), reduced to iSnkStdby while a
 * contract transition is in flight, and the negotiated operating
 * current once PS_RDY arrives. Unlike source_supply there is no
 * completion event - the standard bounds the reduction by time
 * (tSnkStdby), not by handshake.
 *
 * Failure contract as in Tcpc.hpp: set_limit() reports whether the
 * driver accepted the operation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Units.hpp>

#include <concepts>

namespace usbc {

namespace concepts {

template<typename T>
concept sink_load = requires(T s, millivolt expected_voltage, milliamp max_current) {
    { s.set_limit(expected_voltage, max_current) } -> std::same_as<bool>;
};

} // namespace concepts

} // namespace usbc
