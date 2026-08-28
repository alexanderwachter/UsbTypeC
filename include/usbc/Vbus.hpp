/*
 * VBUS monitoring and discharge as its own integration interface,
 * separate from the TCPC: hardware frequently senses VBUS with an ADC
 * or comparator outside the port controller, and Zephyr models it as a
 * distinct device driver (zephyr/drivers/usb_c/usbc_vbus.h). A TCPI
 * TCPC that senses VBUS itself implements both interfaces in one
 * driver.
 *
 * The interface is event-driven; the stack never polls. It selects the
 * one condition it currently cares about with monitor() - the level
 * VBUS should (or should no longer) be at in the present state - and
 * the driver reports through the registered callback, typically from
 * interrupt context: once with the current condition state as soon as
 * it is known after monitor(), then on every change. A new monitor()
 * replaces the previous condition. How the driver detects a crossing
 * (comparator, ADC sampling, TCPC voltage alarm) is its own business.
 *
 * Failure contract as in Tcpc.hpp: bool functions report whether the
 * driver accepted the operation.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <concepts>
#include <cstdint>

namespace usbc {

// Detection conditions the standard defines on VBUS. "Met" means the
// condition associated with the level holds.
enum class vbus_level : std::uint8_t {
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

using vbus_callback = void (*)(void* context, bool met);

namespace concepts {

template<typename T>
concept vbus = requires(T v, vbus_level level, vbus_callback callback, void* context,
                        bool enable) {
    { v.enable(enable) } -> std::same_as<bool>;
    v.set_callback(callback, context);
    { v.monitor(level) } -> std::same_as<bool>;
    { v.discharge(enable) } -> std::same_as<bool>;
};

} // namespace concepts

} // namespace usbc
