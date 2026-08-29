/*
 * The port-controller side of the stack's integration interface:
 * a duck-typed abstraction of a USB Type-C Port Controller (TCPC).
 * Its functional blocks follow the USB Type-C Port Controller Interface
 * (TCPI) specification, but the contract is kept implementable by
 * drivers that are not register-level TCPI - in particular a shim over
 * a Zephyr TCPC device driver (zephyr/drivers/usb_c/usbc_tcpc.h). The
 * state machines play the TCPM role and drive a user-provided driver
 * satisfying concepts::tcpc. VBUS sensing is a separate interface
 * (concepts::vbus in Vbus.hpp): hardware, and Zephyr's driver model,
 * often measure VBUS outside the TCPC. Connection detection (including
 * DRP toggling) is done by the stack's state machines; a TCPC's
 * autonomous Look4Connection mechanism is deliberately not part of
 * the interface.
 *
 * Execution contract: the driver signals pending work by invoking the
 * registered alert callback (typically from interrupt context). The
 * stack later calls read_alert() from its own context - reading clears
 * the pending flags, like the write-to-clear ALERT register. Received
 * messages are fetched with receive() until it returns false. All other
 * functions are called from the stack's context only; the driver never
 * calls into the stack apart from the alert callback.
 *
 * Failure contract: functions returning bool report whether the driver
 * accepted the operation (e.g. the bus transfer succeeded); read_*
 * functions return std::nullopt on failure. transmit() is a single
 * transmission attempt whose outcome arrives as a transmit_* alert
 * after the driver's GoodCRC handling; retransmission is the protocol
 * layer's job (PD spec PRL_Tx), so a TCPI-based driver shall configure
 * zero hardware retries.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Flags.hpp>
#include <usbc/Message.hpp>

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
};

// The CC line carrying the CC wire; VCONN, when enabled, is applied to
// the other line
enum class plug_orientation : std::uint8_t { cc1, cc2 };

// What the driver shall detect and forward to the stack; a driver
// without this granularity enables at least the requested classes
enum class receive_detect : std::uint8_t {
    none                   = 0,
    sop                    = 1u << 0,
    sop_prime              = 1u << 1,
    sop_double_prime       = 1u << 2,
    sop_prime_debug        = 1u << 3,
    sop_double_prime_debug = 1u << 4,
    hard_reset             = 1u << 5,
    cable_reset            = 1u << 6,
};

template<>
struct is_flags<receive_detect> : std::true_type {};

// What the driver needs to build GoodCRC answers
struct message_header_info {
    power_role power;
    data_role data;
    pd_revision revision;
};

// Transmissions without message content
enum class transmit_signal : std::uint8_t {
    hard_reset,
    cable_reset,
    bist_carrier_mode_2,
};

// Pending events, cleared by read_alert()
enum class alert_status : std::uint8_t {
    none                = 0,
    cc_status_changed   = 1u << 0,
    message_received    = 1u << 1, // fetch with receive() until it returns false
    transmit_success    = 1u << 2, // GoodCRC received
    transmit_discarded  = 1u << 3, // dropped in favor of an incoming message
    transmit_failed     = 1u << 4, // no GoodCRC within the retries
    hard_reset_received = 1u << 5,
    fault               = 1u << 6,
};

template<>
struct is_flags<alert_status> : std::true_type {};

using alert_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept tcpc = requires(T t, cc_pull pull, rp_value rp, plug_orientation orientation,
                        receive_detect detect, message_header_info header_info,
                        pd_message const& message, pd_message& receive_buffer,
                        transmit_signal signal, alert_callback callback, void* context,
                        bool enable) {
    { t.init() } -> std::same_as<bool>;
    t.set_alert_handler(callback, context);
    { t.read_alert() } -> std::same_as<std::optional<alert_status>>;

    // connection detection
    { t.set_cc(pull, rp) } -> std::same_as<bool>;
    { t.read_cc_status() } -> std::same_as<std::optional<cc_status>>;
    { t.set_plug_orientation(orientation) } -> std::same_as<bool>;

    // power path switches; voltage programming is the source_supply
    // interface (SourceSupply.hpp)
    { t.source_vbus(enable) } -> std::same_as<bool>;
    { t.sink_vbus(enable) } -> std::same_as<bool>;
    { t.set_vconn(enable) } -> std::same_as<bool>;

    // PD messaging
    { t.set_message_header_info(header_info) } -> std::same_as<bool>;
    { t.set_receive_detect(detect) } -> std::same_as<bool>;
    { t.transmit(message) } -> std::same_as<bool>; // one attempt, outcome via alert
    { t.transmit(signal) } -> std::same_as<bool>;
    { t.receive(receive_buffer) } -> std::same_as<bool>;
};

} // namespace concepts

} // namespace usbc
