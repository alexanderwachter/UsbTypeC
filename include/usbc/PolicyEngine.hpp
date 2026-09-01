/*
 * Common ground of the USB PD policy engines: the specification's
 * per-state Power/PD notation (rendered into the DOT diagrams and
 * driving the contract-apply edges), the shared action and observation
 * types, the timers and levels both roles use, the events the protocol
 * layer feeds into either engine, and the control message builder. The
 * sink and source engines live in SinkPolicyEngine.hpp and
 * SourcePolicyEngine.hpp.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Pdo.hpp>
#include <usbc/Units.hpp>

#include <chrono>
#include <cstdint>
#include <string_view>

namespace usbc {

namespace pe {

inline constexpr auto t_sender_response = std::chrono::milliseconds{27}; // 24 ms - 30 ms
inline constexpr auto t_chunking_not_supported = std::chrono::milliseconds{45}; // 40 ms - 50 ms

inline constexpr millivolt v_safe_5v        = 5000; // vSafe5V
inline constexpr milliamp i_default_current = 500;  // implicit vSafe5V contract

// The specification's per-state notation: every state is annotated
// with its power level and whether PD communication is connected
enum class power_level : std::uint8_t {
    default_power,
    contract_or_default, // "Default/implicit or explicit contract"
    transition,
    explicit_contract,
};
enum class pd_status : std::uint8_t { not_connected, connected, connected_or_not_connected };

constexpr std::string_view specNote(power_level power, pd_status pd)
{
    switch (power) {
    case power_level::contract_or_default:
        switch (pd) {
        case pd_status::connected:
            return "Power: Default/implicit or explicit contract | PD: Connected";
        case pd_status::connected_or_not_connected:
            return "Power: Default/implicit or explicit contract | PD: Connected/not Connected";
        default: return "Power: Default/implicit or explicit contract | PD: not Connected";
        }
    case power_level::transition:
        switch (pd) {
        case pd_status::connected: return "Power: Transition | PD: Connected";
        case pd_status::connected_or_not_connected: return "Power: Transition | PD: Connected/not Connected";
        default: return "Power: Transition | PD: not Connected";
        }
    case power_level::explicit_contract:
        switch (pd) {
        case pd_status::connected: return "Power: Explicit Contract | PD: Connected";
        case pd_status::connected_or_not_connected:
            return "Power: Explicit Contract | PD: Connected/not Connected";
        default: return "Power: Explicit Contract | PD: not Connected";
        }
    default:
        switch (pd) {
        case pd_status::connected: return "Power: Default | PD: Connected";
        case pd_status::connected_or_not_connected: return "Power: Default | PD: Connected/not Connected";
        default: return "Power: Default | PD: not Connected";
        }
    }
}

// Observations; each type selects its notify hook. The PRL-directed
// commands (prl::reset_action, prl::hard_reset_action) live with the
// protocol layer; the power-directed one is defined here. Action types
// carry their diagram note, so the rendered state machine shows
// exactly what entering the state does
struct restore_default_action {
    static constexpr std::string_view note = "restores default power";
    constexpr bool operator==(restore_default_action const&) const = default;
};
struct active_contract {
    millivolt voltage = v_safe_5v;
    milliamp current  = i_default_current;
};

namespace event {

// The protocol layer's reports, common to both engine roles
struct message_sent {};   // PRL: transmission confirmed
struct protocol_error {}; // PRL: transmission failed
struct accept {};
struct soft_reset_received {
    pd_message accept;
};
struct unsupported {
    pd_message reply;
};
struct chunked_message {
    pd_message reply;
};
struct hard_reset_complete {};
struct hard_reset_received {};

} // namespace event

inline pd_message makeControlMessage(control_message_type type, power_role power, data_role data)
{
    return {.sop    = sop_type::sop,
            .header = pd_header{.message_type    = static_cast<std::uint8_t>(type),
                                .port_data_role  = data,
                                .revision        = pd_revision::rev_3_x,
                                .port_power_role = power}
                          .encode()};
}

} // namespace pe

} // namespace usbc
