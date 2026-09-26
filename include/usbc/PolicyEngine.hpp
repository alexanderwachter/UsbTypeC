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
    // restoring already restored defaults changes nothing (the power
    // observers guard on an active contract): a wildcard's entry may
    // re-notify it
    constexpr bool operator==(restore_default_action const&) const = default;
};
struct active_contract {
    millivolt voltage = v_safe_5v;
    milliamp current  = i_default_current;
};

// Port observations: an engine state reporting one of these through
// portReport() tells the port-level integration (the PdDrp facade)
// what it needs from the Type-C layer
// All but data_role_changed are compile-time facts of their states
// and sit in the static annotation sets (comparable for the sets'
// change suppression); data_role_changed carries the runtime role
struct data_role_changed { // an agreed DR_Swap: both sides flipped
    data_role role;
};
struct assert_new_role { // a PR_Swap reached the termination change
    power_role role;
    constexpr bool operator==(assert_new_role const&) const = default;
};
struct enter_swap_standby { // agreed PR_Swap: hold the connection
    power_role role;        // layer's swap standby toward this role
    constexpr bool operator==(enter_swap_standby const&) const = default;
};
struct swap_completed { // the new source's PS_RDY: the swap is done
    constexpr bool operator==(swap_completed const&) const = default;
};
struct request_error_recovery { // nHardResetCount exhausted
    constexpr bool operator==(request_error_recovery const&) const = default;
};
struct hard_reset_window { // hold the attach while VBUS cycles
    constexpr bool operator==(hard_reset_window const&) const = default;
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
// The partner's swap request, delivered as the table's question: the
// guarded Ready row accepts when the injected policy answers yes, the
// catch-all sends the refusal - Reject, or Not_Supported without the
// feature (the Accept is built by the accepting state)
struct swap_request {
    control_message_type refusal;
};
struct pr_swap_received : swap_request {};
struct dr_swap_received : swap_request {};
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
struct vconn_swap_received : swap_request {}; // the partner's VCONN_Swap
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

// The optional features, as tags - the fsm library's feature mechanism:
// a state declares the feature it belongs to (`using feature =
// pe::..._feature;`), and an injected observer declaring the same tag
// (`using enables = ...;` - one tag, or an mtl::typelist of them)
// switches the feature on (fsm::observer_enables_v). A disabled
// feature's states - and every table entry touching them, timer-range
// map entries included - are removed at compile time
// (fsm::remove_features_t); the enabling observer must satisfy the
// feature's contract (checked where it is detected)
struct pr_swap_feature {};  // contract: allowSwap(power_role)
struct dr_swap_feature {};  // contract: allowSwap(data_role)
struct vconn_feature {};    // contract: concepts::vconn_port

// The tables' arbitration questions - fsm::guard tags without a static
// check, answered by the policy injected into the engine's machine:
// bool check(pe::pr_swap_allowed) - may this port take the other
// power role? - and likewise the other data role and the VCONN source
// role. A table asking a question nobody answers does not compile, so
// a policy answering is exactly what brings the feature's states in
struct pr_swap_allowed {};
struct dr_swap_allowed {};
struct vconn_swap_allowed {};

// The disabled features of a configuration as one tag list, so
// disabling costs a single filter pass over a table or map (chained
// per-feature passes measured multiples of the instantiations)
template<bool PR_SWAP, bool DR_SWAP, bool VCONN>
using disabled_features_t = mtl::concat_t<
    mtl::concat_t<std::conditional_t<PR_SWAP, mtl::typelist<>, mtl::typelist<pr_swap_feature>>,
                  std::conditional_t<DR_SWAP, mtl::typelist<>, mtl::typelist<dr_swap_feature>>>,
    std::conditional_t<VCONN, mtl::typelist<>, mtl::typelist<vconn_feature>>>;

// Lazy by design: the everything-enabled specialization returns the
// list untouched without ever naming the filter (a conditional_t alias
// would evaluate the filter branch either way). One trait for the
// transition lists and the timer-range maps: the filter knows both
template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
struct without_disabled
    : std::type_identity<
          fsm::remove_features_t<LIST, disabled_features_t<PR_SWAP, DR_SWAP, VCONN>>> {};
template<typename LIST>
struct without_disabled<LIST, true, true, true> : std::type_identity<LIST> {};

template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
using without_disabled_t = typename without_disabled<LIST, PR_SWAP, DR_SWAP, VCONN>::type;

// Annotation tag: a due request parked by PD3 collision avoidance -
// the engine re-initiates on SinkTxOk, and a gate entered while Ok
// already holds fires right away (the queued machine delivers the
// retry after the gate's entry completes)
struct retry_gated {
    constexpr bool operator==(retry_gated const&) const = default;
};

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
