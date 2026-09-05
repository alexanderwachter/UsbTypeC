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
#include <usbc/Spec.hpp>
#include <usbc/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace usbc {

// One contract the sink side can accept, and the Sink_Capabilities
// content (also answered by a DRP asked while sourcing)
struct sink_capability {
    millivolt voltage;
    milliamp current;
};

// The tag the swap arbitration is asked with: allowSwap(vconn_source_role)
// says whether this port may take over sourcing VCONN
struct vconn_source_role {};

namespace pe {

inline constexpr auto t_sender_response        = std::chrono::milliseconds{27}; // tSenderResponse
inline constexpr auto t_chunking_not_supported = std::chrono::milliseconds{45}; // tChunkingNotSupported
inline constexpr auto t_bist_cont_mode         = std::chrono::milliseconds{45}; // tBISTContMode

inline constexpr millivolt v_safe_5v        = spec::v_safe_5v_nom;
inline constexpr milliamp i_default_current = spec::i_usb_default; // implicit vSafe5V contract

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

// Port observations: an engine state reporting one of these through
// portReport() tells the port-level integration (the PdDrp facade)
// what it needs from the Type-C layer
struct data_role_changed { // an agreed DR_Swap: both sides flipped
    data_role role;
};
struct assert_new_role { // a PR_Swap reached the termination change
    power_role role;
};
struct enter_swap_standby { // agreed PR_Swap: hold the connection
    power_role role;        // layer's swap standby toward this role
};
struct swap_completed {}; // the new source's PS_RDY: the swap is done
struct request_error_recovery {}; // nHardResetCount exhausted
struct hard_reset_window {}; // hold the attach while VBUS cycles
struct request_hard_reset {}; // e.g. a VCONN_Swap hand-off timed out
struct vconn_swap_agreed {};  // VCONN_Swap accepted: the vconn machine takes over
struct vconn_partner_on {};   // the new VCONN source's PS_RDY arrived
struct vconn_ps_rdy_sent {};  // our VCONN PS_RDY is on the wire
struct announce_vconn_on {};  // vconn machine: transmit our PS_RDY

namespace event {

// The protocol layer's reports, common to both engine roles
struct message_sent {};   // PRL: transmission confirmed
struct protocol_error {}; // PRL: transmission failed
struct accept {};
struct reject {};
struct wait {};
struct ps_rdy {};
struct soft_reset_received {
    pd_message accept;
};

// Role swap messaging, shared by both engine roles
struct send_pr_swap { // our PR_Swap goes out
    pd_message message;
};
struct send_dr_swap { // our DR_Swap goes out
    pd_message message;
};
struct pr_swap_accepted { // the partner's PR_Swap passed arbitration
    pd_message accept;
};
struct dr_swap_accepted { // the partner's DR_Swap passed arbitration
    pd_message accept;
};
struct swap_wait_source_on { // the port asserted Rd mid PR_Swap: this
    data_role role;          // engine announces the supply is off
};
struct attached_swap { // activation continuing a PR_Swap (new source)
    data_role role;
};
struct swap_done {};     // advances the transient swap states
struct bist_carrier {};  // BIST Carrier Mode 2 requested (vSafe5V)

// VCONN_Swap messaging, shared by both engine roles; the role and
// switch choreography lives in the vconn machine (Vconn.hpp)
struct send_vconn_swap { // our VCONN_Swap goes out
    pd_message message;
};
struct vconn_swap_accepted { // the partner's VCONN_Swap passed arbitration
    pd_message accept;
};
struct send_vconn_ps_rdy { // the vconn machine turned the switch on
    pd_message message;
};
struct hard_reset_request {}; // port-level escalation (vconn timeout)
struct unsupported {
    pd_message reply;
};
struct chunked_message {
    pd_message reply;
};
struct hard_reset_complete {};
struct hard_reset_received {};

} // namespace event

// The optional VCONN feature: states carrying a vconn_feature marker
// belong to it, and an engine whose injected policy has no
// allowSwap(vconn_source_role) removes them - and every transition
// touching them - from its table at compile time
template<typename STATE>
struct is_vconn_state : std::bool_constant<requires { STATE::vconn_feature; }> {};

template<typename ENTRY>
struct touches_vconn : std::false_type {}; // fsm::initial<> and friends

template<typename ENTRY>
    requires requires {
        typename ENTRY::from;
        typename ENTRY::to;
    }
struct touches_vconn<ENTRY>
    : std::bool_constant<is_vconn_state<typename ENTRY::from>::value ||
                         is_vconn_state<typename ENTRY::to>::value> {};

// ... and its timer-range map counterpart (the map check is
// bidirectional: entries for filtered states must go too)
template<typename ENTRY>
struct times_vconn_state : std::false_type {};

template<typename STATE, auto const& BOUND>
struct times_vconn_state<fsm::timed_by<STATE, BOUND>> : is_vconn_state<STATE> {};

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
