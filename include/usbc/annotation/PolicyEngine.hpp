/*
 * The facts the USB PD policy engines publish, as the observers hear
 * them: the specification's per-state Power/PD notation, the active
 * contract and the power-directed actions (what the power classes of
 * interface/Power.hpp apply), the data role flip, and the VCONN_Swap's
 * progress for the vconn machine. The PRL-directed actions live in
 * annotation/ProtocolLayer.hpp, the directions to the connection layer
 * in annotation/TypeC.hpp. No state, no machine.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/spec/Message.hpp>
#include <usbc/spec/Parameters.hpp>
#include <usbc/spec/Units.hpp>

#include <cstdint>
#include <string_view>

namespace usbc {

namespace pe {

inline constexpr millivolt v_safe_5v = spec::v_safe_5v_nom;
inline constexpr milliamp i_default_current = spec::i_usb_default; // implicit vSafe5V contract
inline constexpr milliamp i_snk_stdby = spec::i_snk_stdby;         // at any voltage

// The specification's per-state notation, carried as annotations: the
// power level on every root state, the PD status - whether the port
// partners are actively communicating, lost with a Hard Reset - only on
// the states the spec marks definitively. A state the spec marks
// "Connected/not Connected" carries none, so an observer of pd_status
// sees it change exactly where the spec says it does, and the machine's
// annotation query answers nothing there
enum class power_level : std::uint8_t {
    default_power,
    contract_or_default, // "Default/implicit or explicit contract"
    transition,
    explicit_contract,
};
enum class pd_status : std::uint8_t { not_connected, connected };

// The power-directed action: restoring vSafe5V defaults. Action types
// carry their diagram note, so the rendered state machine shows
// exactly what entering the state does
struct restore_default_action {
    static constexpr std::string_view note = "restores default power";
    // restoring already restored defaults changes nothing (the power
    // observers guard on an active contract): a wildcard's entry may
    // re-notify it
    constexpr bool operator==(restore_default_action const&) const = default;
};

// The explicit contract in place, an instance value of Ready
struct active_contract {
    millivolt voltage = v_safe_5v;
    milliamp current = i_default_current;
};

// The sink's standby draw during a contract transition: the voltage
// to expect, the current is iSnkStdby
struct standby_limit {
    millivolt voltage;
    constexpr bool operator==(standby_limit const&) const = default;
};

// The engine's facts for the port: the data role flip and the
// VCONN_Swap's progress. data_role_changed carries the runtime role as
// an instance value, the rest are compile-time facts of their states
// in the static annotation sets (comparable for the sets' change
// suppression)
struct data_role_changed { // an agreed DR_Swap: both sides flipped
    data_role role;
};
struct request_hard_reset { // e.g. a VCONN_Swap hand-off timed out
    constexpr bool operator==(request_hard_reset const&) const = default;
};
struct vconn_swap_agreed { // VCONN_Swap accepted: the vconn machine takes over
    constexpr bool operator==(vconn_swap_agreed const&) const = default;
};
struct vconn_partner_on { // the new VCONN source's PS_RDY arrived
    constexpr bool operator==(vconn_partner_on const&) const = default;
};
struct vconn_ps_rdy_sent { // our VCONN PS_RDY is on the wire
    constexpr bool operator==(vconn_ps_rdy_sent const&) const = default;
};
struct announce_vconn_on { // vconn machine: transmit our PS_RDY
    constexpr bool operator==(announce_vconn_on const&) const = default;
};

} // namespace pe

} // namespace usbc
