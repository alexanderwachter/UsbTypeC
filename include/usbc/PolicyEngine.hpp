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

// The optional features, as tags: a state declares the feature it
// belongs to (`using feature = pe::..._feature;`), and an injected
// observer declaring the same tag (`using enables = ...;` - one tag,
// or an mtl::typelist of them) switches the feature on. A disabled
// feature's states - and every transition touching them - are
// removed from the tables at compile time; the enabling observer
// must satisfy the feature's contract (checked where it is detected)
struct pr_swap_feature {};  // contract: allowSwap(power_role)
struct dr_swap_feature {};  // contract: allowSwap(data_role)
struct vconn_feature {};    // contract: concepts::vconn_port

// Whether ENABLES (a tag, or a typelist of tags) names TAG
template<typename ENABLES, typename TAG>
struct enables_lists : std::is_same<ENABLES, TAG> {};

template<typename... TAGs, typename TAG>
struct enables_lists<mtl::typelist<TAGs...>, TAG>
    : std::bool_constant<(std::is_same_v<TAGs, TAG> || ...)> {};

template<typename OBSERVER, typename TAG>
struct observer_enables : std::false_type {};

template<typename OBSERVER, typename TAG>
    requires requires { typename OBSERVER::enables; }
struct observer_enables<OBSERVER, TAG> : enables_lists<typename OBSERVER::enables, TAG> {};

template<typename OBSERVER, typename TAG>
inline constexpr bool observer_enables_v = observer_enables<OBSERVER, TAG>::value;

// Whether STATE belongs to the tagged feature
template<typename STATE, typename TAG>
struct state_in_feature : std::false_type {};

template<typename STATE, typename TAG>
    requires requires { typename STATE::feature; }
struct state_in_feature<STATE, TAG> : std::is_same<typename STATE::feature, TAG> {};

// The disabled features as ONE predicate set, so disabling costs a
// single filter pass over the table (chained per-feature passes and
// eager conditional_t branches measured multiples of the
// instantiations). entry_pred drops every transition whose source or
// target belongs to a disabled feature (fsm::initial<> and friends
// never match); timer_pred is the timer-range map counterpart (the
// map check is bidirectional: entries for filtered states must go too)
template<bool PR_SWAP, bool DR_SWAP, bool VCONN>
struct in_disabled_feature {
    template<typename STATE>
    static constexpr bool matches =
        (!PR_SWAP && state_in_feature<STATE, pr_swap_feature>::value) ||
        (!DR_SWAP && state_in_feature<STATE, dr_swap_feature>::value) ||
        (!VCONN && state_in_feature<STATE, vconn_feature>::value);

    template<typename ENTRY>
    struct entry_pred : std::false_type {};

    template<typename ENTRY>
        requires requires {
            typename ENTRY::from;
            typename ENTRY::to;
        }
    struct entry_pred<ENTRY> : std::bool_constant<matches<typename ENTRY::from> ||
                                                  matches<typename ENTRY::to>> {};

    template<typename ENTRY>
    struct timer_pred : std::false_type {};

    template<typename STATE, auto const& BOUND>
    struct timer_pred<fsm::timed_by<STATE, BOUND>> : std::bool_constant<matches<STATE>> {};
};

// Lazy by design: the everything-enabled specialization returns the
// list untouched without ever naming remove_if (a conditional_t
// alias would evaluate the filter branch either way)
template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
struct table_without_disabled
    : std::type_identity<mtl::remove_if_t<
          LIST, in_disabled_feature<PR_SWAP, DR_SWAP, VCONN>::template entry_pred>> {};
template<typename LIST>
struct table_without_disabled<LIST, true, true, true> : std::type_identity<LIST> {};

template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
struct map_without_disabled
    : std::type_identity<mtl::remove_if_t<
          LIST, in_disabled_feature<PR_SWAP, DR_SWAP, VCONN>::template timer_pred>> {};
template<typename LIST>
struct map_without_disabled<LIST, true, true, true> : std::type_identity<LIST> {};

template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
using table_without_disabled_t =
    typename table_without_disabled<LIST, PR_SWAP, DR_SWAP, VCONN>::type;
template<typename LIST, bool PR_SWAP, bool DR_SWAP, bool VCONN>
using map_without_disabled_t =
    typename map_without_disabled<LIST, PR_SWAP, DR_SWAP, VCONN>::type;

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
