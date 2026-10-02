/*
 * Common ground of the USB PD policy engines: the specification's
 * per-state Power/PD notation (rendered into the DOT diagrams and
 * driving the contract-apply edges), the shared action and observation
 * types, the timers and levels both roles use, the events the protocol
 * layer feeds into either engine, the message builders, the connection
 * context, the states both engines share (the spec's role-neutral
 * PE_DRS_/PE_PRS_/PE_VCS_ states and the resets and service states
 * whose PE_SNK_/PE_SRC_ twins differ in nothing but the role in the
 * messages they build), the power observers, and the engine base
 * carrying the facade machinery both roles run (protocol-layer port,
 * message dispatch, BIST, capability answers). The sink and source
 * engines live in SinkPolicyEngine.hpp and SourcePolicyEngine.hpp.
 *
 * Events carry facts, states build messages: an event names the
 * message type or the contract terms, and the entered state builds the
 * pd_message from them and its connection context. Only the
 * capability lists - owned by the engine facade - travel pre-built.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Pdo.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/Spec.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
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

inline constexpr auto t_sender_response = std::chrono::milliseconds{27}; // tSenderResponse
inline constexpr auto t_chunking_not_supported =
    std::chrono::milliseconds{45};                                      // tChunkingNotSupported
inline constexpr auto t_bist_cont_mode = std::chrono::milliseconds{45}; // tBISTContMode
inline constexpr auto t_pr_swap_wait = std::chrono::milliseconds{150};  // tPRSwapWait
inline constexpr auto t_dr_swap_wait = std::chrono::milliseconds{150};  // tDRSwapWait

inline constexpr millivolt v_safe_5v = spec::v_safe_5v_nom;
inline constexpr milliamp i_default_current = spec::i_usb_default; // implicit vSafe5V contract

// The data role a port takes on attach: a sink attaches as UFP, a
// source as DFP
constexpr data_role defaultDataRole(power_role role)
{
    return role == power_role::sink ? data_role::ufp : data_role::dfp;
}

// Machine-owned context, the part living from attach to detach - both
// engines keep the same one: a hard reset changes neither the data
// role nor its own counter, and a power role swap is no reset at all
// (6.7.1). The negotiation's worth (gone with every reset within the
// connection) differs per role and lives with each engine
struct pe_connection {
    data_role data = data_role::ufp; // flipped by an agreed DR_Swap
    std::uint8_t hard_resets = 0;    // HardResetCounter
    bool attached = false;           // the source's restore asks whether the sink is still there
};

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
        case pd_status::connected_or_not_connected:
            return "Power: Transition | PD: Connected/not Connected";
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
        case pd_status::connected_or_not_connected:
            return "Power: Default | PD: Connected/not Connected";
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
    milliamp current = i_default_current;
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
struct soft_reset_received {}; // the entered state builds the Accept

// Role swap messaging, shared by both engine roles; the entered state
// builds the request from its connection context
struct send_pr_swap {}; // our PR_Swap goes out
struct send_dr_swap {}; // our DR_Swap goes out
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
struct swap_done {};    // advances the transient swap states
struct bist_carrier {}; // BIST Carrier Mode 2 requested (vSafe5V)

// VCONN_Swap messaging, shared by both engine roles; the role and
// switch choreography lives in the vconn machine (Vconn.hpp)
struct send_vconn_swap {};                    // our VCONN_Swap goes out
struct vconn_swap_received : swap_request {}; // the partner's VCONN_Swap
struct send_vconn_ps_rdy {};                  // the vconn machine turned the switch on
struct hard_reset_request {};                 // port-level escalation (vconn timeout)
struct unsupported {};                        // answered with Not_Supported from Ready
struct chunked_message {};                    // a chunked extended message arrived
struct hard_reset_complete {};
struct hard_reset_received {};
// The capability lists live with the engine facade, so their messages
// travel pre-built: Get_Sink_Cap answered in either role
struct send_sink_capabilities {
    pd_message message;
};

} // namespace event

// The tables' arbitration questions - fsm::guard tags without a static
// check, answered by the policy injected into the engine's machine:
// bool check(pe::pr_swap_allowed) - may this port take the other
// power role? - and likewise the other data role and the VCONN source
// role
struct pr_swap_allowed {};
struct dr_swap_allowed {};
struct vconn_swap_allowed {};

// The optional features, as tags - the fsm library's feature mechanism:
// a state declares the feature it belongs to (`using feature =
// pe::..._feature;`), and the tag names the question that enables it:
// a policy answering it is exactly what brings the feature's states
// in, and the machine leaves a disabled feature's states - and every
// entry touching them - out at compile time. The answering policy must
// satisfy the feature's contract (checked where it is detected)
struct pr_swap_feature { // contract: allowSwap(power_role)
    using enabled_by = pr_swap_allowed;
};
struct dr_swap_feature { // contract: allowSwap(data_role)
    using enabled_by = dr_swap_allowed;
};
struct vconn_feature { // contract: concepts::vconn_port
    using enabled_by = vconn_swap_allowed;
};

// The Atomic Message Sequences (the spec's AMS: a request and its
// replies, during which nothing else may start) an engine initiates
// and may have to hold back under PD3 collision avoidance
enum class atomic_message_sequence : std::uint8_t { request, power_role_swap, data_role_swap };

// Annotation: a due request parked by PD3 collision avoidance - the
// engine re-initiates on SinkTxOk, and a gate entered while Ok
// already holds fires right away (the queued machine delivers the
// retry after the gate's entry completes). The element says which
// sequence is waiting, so the engine fires the retry off the annotation
struct retry_gated {
    atomic_message_sequence sequence;
    constexpr bool operator==(retry_gated const&) const = default;
};

// Annotation tags the engines' facades query off the machine
// (fsm::annotation<T>) instead of enumerating states: Ready - no
// Atomic Message Sequence running, one may start (the spec allows
// swaps only there, under the explicit contract) - and the transients
// of the swap choreography that swap_done advances once their trigger
// was processed (the spec chains them without further input)
struct ready_for_atomic_message_sequence {
    constexpr bool operator==(ready_for_atomic_message_sequence const&) const = default;
};
struct swap_transient {
    constexpr bool operator==(swap_transient const&) const = default;
};

inline pd_message makeControlMessage(control_message_type type, power_role power, data_role data)
{
    return {
        .sop = sop_type::sop,
        .header =
            pd_header{
                .message_type = static_cast<std::uint8_t>(type),
                .port_data_role = data,
                .revision = pd_revision::rev_3_x,
                .port_power_role = power
            }
                .encode()
    };
}

// A data message of up to seven objects (the excess is dropped)
inline pd_message makeDataMessage(
    data_message_type type,
    power_role power,
    data_role data,
    std::span<std::uint32_t const> objects
)
{
    auto const count = std::min<std::size_t>(objects.size(), 7);
    pd_message message{
        .sop = sop_type::sop,
        .header =
            pd_header{
                .message_type = static_cast<std::uint8_t>(type),
                .port_data_role = data,
                .revision = pd_revision::rev_3_x,
                .port_power_role = power,
                .num_data_objects = static_cast<std::uint8_t>(count)
            }
                .encode()
    };
    for (std::size_t index = 0; index < count; ++index) {
        appendDataObject(message, objects[index]);
    }
    return message;
}

inline pd_message
makeSourceCapabilitiesMessage(power_role power, data_role data, std::span<std::uint32_t const> pdos)
{
    return makeDataMessage(data_message_type::source_capabilities, power, data, pdos);
}

inline pd_message makeSinkCapabilitiesMessage(
    power_role power,
    data_role data,
    std::span<sink_capability const> capabilities
)
{
    std::array<std::uint32_t, 7> objects{};
    auto const count = std::min<std::size_t>(capabilities.size(), objects.size());
    for (std::size_t index = 0; index < count; ++index) {
        objects[index] =
            pdo::makeFixedSink(capabilities[index].voltage, capabilities[index].current);
    }
    return makeDataMessage(
        data_message_type::sink_capabilities,
        power,
        data,
        std::span<std::uint32_t const>{objects.data(), count}
    );
}

// --- the states both engines share --------------------------------------------
//
// Named as the spec names them without a role (PE_DRS_*, PE_PRS_*,
// PE_VCS_*, and the resets and service states whose PE_SNK_/PE_SRC_
// twins are one state). A state that builds a message is a template on
// the port's power role; the rest are plain types both tables list.
// Each engine aliases them under the spec's role-specific names where
// the spec has them, so its table reads against the spec
namespace state {

// PE_SNK_/PE_SRC_Send_Not_Supported: answers a message the port does
// not support - or a swap request the table's question refused - then
// returns to Ready
template<power_role ROLE>
struct pe_send_not_supported {
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    pe_send_not_supported(event::unsupported const&, pe_connection& connection_ref)
        : pe_send_not_supported(connection_ref)
    {
    }
    // a swap request the table's question refused (or nobody answers)
    pe_send_not_supported(event::swap_request const& event, pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(event.refusal, ROLE, connection.data))
    {
    }
    // the chunking timeout ran out: Not_Supported as well
    explicit pe_send_not_supported(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::not_supported, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Chunk_Received: a non-chunking device lets the sender
// run into its chunking timeout before answering Not_Supported
struct pe_chunk_received {
    static constexpr auto timeout = t_chunking_not_supported; // ChunkingNotSupportedTimer
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

// PE_SNK_/PE_SRC_BIST_Carrier_Mode: the tester's carrier runs for
// tBISTContMode (the engine commanded the TCPC on entry), then normal
// operation resumes
struct pe_bist_carrier {
    static constexpr auto timeout = t_bist_cont_mode; // BISTContModeTimer
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "transmits the BIST carrier";
};

// PE_SNK_Give_Sink_Cap / PE_DR_SRC_Give_Sink_Cap: Get_Sink_Cap answered
// with the sink-role capabilities, then back to Ready
struct pe_give_sink_cap {
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    pe_give_sink_cap() = default;
    explicit pe_give_sink_cap(event::send_sink_capabilities const& event) : message_(event.message)
    {
    }

    pd_message const& values() const { return message_; }

private:
    pd_message message_{};
};

// PE_DRS_*_Send_Swap: our DR_Swap is out; no answer within
// tSenderResponse means the partner ignored it - stay Ready
template<power_role ROLE>
struct pe_drs_send_swap {
    using feature = dr_swap_feature;
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends DR_Swap";

    pe_drs_send_swap(event::send_dr_swap const&, pe_connection& connection_ref)
        : pe_drs_send_swap(connection_ref)
    {
    }
    // also the re-entry from the tDRSwapWait retry
    explicit pe_drs_send_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::dr_swap, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_DRS_*_Accept_Swap: the partner's DR_Swap passed the arbitration
template<power_role ROLE>
struct pe_drs_accept_swap {
    using feature = dr_swap_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_drs_accept_swap(event::dr_swap_received const&, pe_connection& connection_ref)
        : pe_drs_accept_swap(connection_ref)
    {
    }
    explicit pe_drs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_DRS_*_Change_to_*: the agreed swap flips the data role; the
// report lets the port update the TCPC header and the Type-C context
struct pe_drs_change_data_role {
    using feature = dr_swap_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power,
        swap_transient{}
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "flips the data role";

    pe_drs_change_data_role(event::accept const&, pe_connection& connection_ref)
        : pe_drs_change_data_role(connection_ref)
    {
    }
    pe_drs_change_data_role(event::message_sent const&, pe_connection& connection_ref)
        : pe_drs_change_data_role(connection_ref)
    {
    }
    explicit pe_drs_change_data_role(pe_connection& connection_ref) : connection(connection_ref)
    {
        connection.data = otherDataRole(connection.data);
    }

    data_role_changed values() const { return {.role = connection.data}; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;
};

// PE_VCS_Send_Swap: our VCONN_Swap is out
template<power_role ROLE>
struct pe_vcs_send_swap {
    using feature = vconn_feature;
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends VCONN_Swap";

    pe_vcs_send_swap(event::send_vconn_swap const&, pe_connection& connection_ref)
        : pe_vcs_send_swap(connection_ref)
    {
    }
    explicit pe_vcs_send_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::vconn_swap, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_VCS_Accept_Swap: the partner's VCONN_Swap passed the arbitration
template<power_role ROLE>
struct pe_vcs_accept_swap {
    using feature = vconn_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_vcs_accept_swap(event::vconn_swap_received const&, pe_connection& connection_ref)
        : pe_vcs_accept_swap(connection_ref)
    {
    }
    explicit pe_vcs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// The agreed swap's message anchor (glue, not a spec state): the vconn
// machine choreographs the hand-off; the engine relays the PS_RDY
// traffic and stays here until its side is done
struct pe_vcs_active {
    using feature = vconn_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power,
        vconn_swap_agreed{}
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

// PE_VCS_Send_PS_RDY: the vconn machine turned the switch on
template<power_role ROLE>
struct pe_vcs_send_ps_rdy {
    using feature = vconn_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY";

    pe_vcs_send_ps_rdy(event::send_vconn_ps_rdy const&, pe_connection& connection_ref)
        : pe_vcs_send_ps_rdy(connection_ref)
    {
    }
    explicit pe_vcs_send_ps_rdy(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::ps_rdy, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// Transients reporting the hand-off progress to the vconn machine
struct pe_vcs_partner_on {
    using feature = vconn_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power,
        vconn_partner_on{},
        swap_transient{}
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

struct pe_vcs_ps_rdy_sent {
    using feature = vconn_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power,
        vconn_ps_rdy_sent{},
        swap_transient{}
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

// PE_PRS_*_Send_Swap: our PR_Swap is out
template<power_role ROLE>
struct pe_prs_send_swap {
    using feature = pr_swap_feature;
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PR_Swap";

    pe_prs_send_swap(event::send_pr_swap const&, pe_connection& connection_ref)
        : pe_prs_send_swap(connection_ref)
    {
    }
    // also the re-entry from the tPRSwapWait retry
    explicit pe_prs_send_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::pr_swap, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_PRS_*_Accept_Swap: the partner's PR_Swap passed arbitration
template<power_role ROLE>
struct pe_prs_accept_swap {
    using feature = pr_swap_feature;
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_prs_accept_swap(event::pr_swap_received const&, pe_connection& connection_ref)
        : pe_prs_accept_swap(connection_ref)
    {
    }
    explicit pe_prs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// The partner answered Wait: the swap request is retried after the
// spec's pause (still Ready, spec-wise)
struct pe_dr_swap_wait {
    using feature = dr_swap_feature;
    static constexpr auto timeout = t_dr_swap_wait; // tDRSwapWait
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

struct pe_pr_swap_wait {
    using feature = pr_swap_feature;
    static constexpr auto timeout = t_pr_swap_wait; // tPRSwapWait
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr auto annotations = fsm::annotate(
        power
    );
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

// PE_SNK_/PE_SRC_Soft_Reset: accepts a received Soft_Reset; the
// protocol layer resets before the Accept goes out (guaranteed hook
// order)
template<power_role ROLE>
struct pe_soft_reset {
    static constexpr auto annotations = fsm::annotate(
        prl::reset_action{}
    );
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    pe_soft_reset(event::soft_reset_received const&, pe_connection& connection_ref)
        : pe_soft_reset(connection_ref)
    {
    }
    explicit pe_soft_reset(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Send_Soft_Reset: protocol errors first try a soft
// reset; the protocol layer is reset before the Soft_Reset goes out
template<power_role ROLE>
struct pe_send_soft_reset {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr auto annotations = fsm::annotate(
        prl::reset_action{}
    );
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    explicit pe_send_soft_reset(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::soft_reset, ROLE, connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Hard_Reset: the signal goes out, HardResetCounter counts
struct pe_hard_reset {
    static constexpr auto annotations = fsm::annotate(
        prl::hard_reset_action{}
    );
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::hard_reset_action::note;

    explicit pe_hard_reset(pe_connection& connection_ref) : connection(connection_ref)
    {
        ++connection.hard_resets; // HardResetCounter
    }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;
};

// nHardResetCount exhausted with no response: the port-level
// integration commands Type-C Error Recovery, whose teardown resets
// the engine
struct pe_error_recovery {
    static constexpr auto annotations = fsm::annotate(
        request_error_recovery{}
    );
    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
};

} // namespace state

// --- guards both tables ask ---------------------------------------------------

// HardResetCounter still allows another hard reset
struct hard_resets_left {
    static bool check(auto const& state)
    {
        return state.connection.hard_resets <= spec::n_hard_reset_count;
    }
};

// An explicit contract is in place (the negotiation context says so)
struct explicit_contract_holds {
    static bool check(auto const& state) { return state.negotiation.explicit_contract; }
};

// --- the power side's observers -----------------------------------------------
//
// The member observers behind SinkPower / SourcePower (POWER is that
// CRTP class), injected together as one fsm::ObserverGroup

// Stores the contract terms the Ready state reports; the power side
// acts on them when the power annotation edge fires
template<typename POWER>
struct contract_store : fsm::observing<contract_store<POWER>> {
    explicit contract_store(POWER& power_ref) : power(power_ref) {}

    // the derived class's interface is checked once the machine is
    // built - the CRTP class is complete by then
    template<typename TABLE>
    static constexpr void validate()
    {
        POWER::validateClient();
    }

    void notifyEntry(active_contract contract) { power.contract_ = contract; }

    POWER& power;
};

// The power effects, delivered from the states' annotation sets by
// overload: the stored contract is applied exactly when the power
// element appears or changes (only Explicit Contract states carry it,
// so bounces between Ready and its service states stay suppressed and
// the engines' wildcard transitions stay shareable), and vSafe5V
// defaults are restored on the states carrying the restore action
template<typename POWER>
struct power_effects : fsm::observing<power_effects<POWER>> {
    explicit power_effects(POWER& power_ref) : power(power_ref) {}

    void notifyEntry(power_level) { power.applyContract(); }
    void notifyEntry(restore_default_action) { power.restoreDefaults(); }

    POWER& power;
};

// --- the engine base ----------------------------------------------------------

// What both engine facades run alike, behind the role-specific
// SinkPolicyEngine / SourcePolicyEngine (CRTP): the protocol layer
// and its client port, the message dispatch with the role's own
// messages left to the derived engine (dispatchRole), the BIST entry,
// the Get_Sink_Cap answer, the swap and VCONN requests, and the
// transient advance. The derived engine owns the machine (its
// observers are the role's) and befriends this base to let it drive
// the machine
template<
    typename DERIVED,
    power_role ROLE,
    concepts::pd_transport TCPC,
    fsm::concepts::timer TIMER,
    typename POLICY>
class PolicyEngineBase {
public:
    // Feed the TCPC's PD alerts (message/transmit/hard reset bits)
    void onAlert(alert_status alerts) { prl_.onAlert(alerts); }

    // VCONN_Swap, driven by the vconn machine through the facade:
    // request the swap, transmit our PS_RDY once the switch is on,
    // and escalate a failed hand-off
    bool requestVconnSwap() { return derived().sm_.process(event::send_vconn_swap{}); }
    bool sendVconnPsRdy() { return derived().sm_.process(event::send_vconn_ps_rdy{}); }
    bool hardReset() { return derived().sm_.process(event::hard_reset_request{}); }

    // The protocol layer's per-partner state, read at a PR_Swap's
    // engine handover
    pd_revision negotiatedRevision() const { return prl_.revision(); }
    prl::message_id_state messageIds() const { return prl_.messageIds(); }

protected:
    PolicyEngineBase(TCPC& tcpc, TIMER& prl_timer, TIMER& pe_timer)
        : tcpc_(tcpc), prl_(tcpc, prl_timer, port_), pe_timer_(pe_timer), timed_(pe_timer_)
    {
        tcpc_.setMessageHeaderInfo(
            {.power = ROLE, .data = defaultDataRole(ROLE), .revision = pd_revision::rev_3_x}
        );
        tcpc_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
    }

    // The optional features follow the injected policy: the machine
    // keeps a feature's states exactly when the policy answers the
    // feature's question (the facade's proxies answer exactly when an
    // injected observer enables the feature by tag); the engine's own
    // branches ask the library the same thing
    static constexpr bool pr_swap_capable = fsm::feature_enabled_v<pr_swap_feature, POLICY>;
    static constexpr bool dr_swap_capable = fsm::feature_enabled_v<dr_swap_feature, POLICY>;
    static constexpr bool vconn_capable = fsm::feature_enabled_v<vconn_feature, POLICY>;

    // The partner's swap request is the table's question (the Ready
    // rows on the *_swap_received events): the injected policy's
    // answer accepts, the catch-all refuses - Reject where the
    // feature exists, the non-DRP Not_Supported where it does not. A
    // request outside Ready is discarded (another Atomic Message
    // Sequence is running)
    template<bool CAPABLE>
    static constexpr control_message_type refusal =
        CAPABLE ? control_message_type::reject : control_message_type::not_supported;

    DERIVED& derived() { return static_cast<DERIVED&>(*this); }
    DERIVED const& derived() const { return static_cast<DERIVED const&>(*this); }

    data_role dataRole() const { return derived().sm_.template context<pe_connection>().data; }

    // The protocol layer's client, forwarding into the engine
    struct PrlPort {
        PolicyEngineBase& engine;

        void onMessage(pd_message const& message) { engine.dispatch(message); }
        // adopted revision: the TCPC's GoodCRC header must follow
        void onRevision(pd_revision revision)
        {
            engine.tcpc_.setMessageHeaderInfo(
                {.power = ROLE, .data = engine.dataRole(), .revision = revision}
            );
        }
        void onTxDone()
        {
            engine.derived().sm_.process(event::message_sent{});
            engine.advanceTransients();
        }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { engine.derived().sm_.process(event::protocol_error{}); }
        void onHardReset()
        {
            engine.setBistTestData(false); // a hard reset ends the test mode
            engine.derived().sm_.process(event::hard_reset_received{});
            engine.derived().afterHardReset();
        }
        void onHardResetSent()
        {
            engine.setBistTestData(false);
            engine.derived().sm_.process(event::hard_reset_complete{});
            engine.derived().afterHardReset();
        }
    };

    // Hook: what follows a hard reset in the machine, if anything (the
    // sink pumps its transients, the source's supply drives them)
    void afterHardReset() {}

    // A transient state left standing after its trigger was processed
    // is advanced here (the spec chains them without further input)
    void advanceTransients()
    {
        if constexpr (dr_swap_capable || vconn_capable) { // else no state is a transient
            if (derived().sm_.template annotation<swap_transient>()) {
                derived().sm_.process(event::swap_done{});
            }
        }
    }

    // A received message becomes the engine's event: the role's own
    // messages (capabilities, Request, Get_Source_Cap) go to the
    // derived engine first, everything else is common to both roles
    void dispatch(pd_message const& message)
    {
        if (bist_test_data_) {
            return; // BIST test data mode: deaf until hard reset/detach
        }
        auto const header = pd_header::decode(message.header);
        if (header.extended) {
            auto const extended = extended_header::decode(
                static_cast<std::uint16_t>(message.payload[0]) |
                (static_cast<std::uint16_t>(message.payload[1]) << 8u)
            );
            if (extended.chunked) {
                derived().sm_.process(event::chunked_message{});
            } else {
                derived().sm_.process(event::unsupported{});
            }
            return;
        }
        if (derived().dispatchRole(header, message)) {
            return;
        }
        if (isData(header, data_message_type::bist)) {
            if (header.num_data_objects >= 1) {
                enterBist(dataObjectAt(message, 0));
            }
        } else if (isControl(header, control_message_type::accept)) {
            derived().sm_.process(event::accept{});
            advanceTransients();
        } else if (isControl(header, control_message_type::reject)) {
            derived().sm_.process(event::reject{});
        } else if (isControl(header, control_message_type::wait)) {
            derived().sm_.process(event::wait{});
        } else if (isControl(header, control_message_type::ps_rdy)) {
            derived().sm_.process(event::ps_rdy{});
            advanceTransients(); // a VCONN hand-off completion
        } else if (isControl(header, control_message_type::dr_swap)) {
            derived().sm_.process(event::dr_swap_received{{refusal<dr_swap_capable>}});
        } else if (isControl(header, control_message_type::pr_swap)) {
            derived().sm_.process(event::pr_swap_received{{refusal<pr_swap_capable>}});
        } else if (isControl(header, control_message_type::vconn_swap)) {
            derived().sm_.process(event::vconn_swap_received{{refusal<vconn_capable>}});
        } else if (isControl(header, control_message_type::get_sink_cap)) {
            sendSinkCapabilities();
        } else if (isControl(header, control_message_type::soft_reset)) {
            derived().sm_.process(event::soft_reset_received{});
        } else if (!isControl(header, control_message_type::good_crc) &&
                   !isControl(header, control_message_type::ping)) {
            // answered from Ready only; ignored while negotiating
            derived().sm_.process(event::unsupported{});
        }
    }

    // Get_Sink_Cap: the sink-role capabilities, which a sink has and a
    // DRP sourcing provides - a source-only port answers Not_Supported
    void sendSinkCapabilities()
    {
        if (sink_capabilities_.empty()) {
            derived().sm_.process(event::unsupported{});
            return;
        }
        derived().sm_.process(
            event::send_sink_capabilities{
                makeSinkCapabilitiesMessage(ROLE, dataRole(), sink_capabilities_)
            }
        );
    }

    // BIST entry, honored only under an explicit vSafe5V contract
    // (spec): Carrier Mode 2 transmits the test carrier for
    // tBISTContMode; Test Data silences the engine until a hard reset
    // or detach while the TCPC keeps answering GoodCRC
    void enterBist(std::uint32_t bdo)
    {
        auto const contract_voltage = derived().contractVoltage();
        if (contract_voltage != v_safe_5v) {
            return;
        }
        switch (bist::modeOf(bdo)) {
        case bist::mode::carrier_mode_2:
            if (derived().sm_.process(event::bist_carrier{})) {
                tcpc_.transmit(transmit_signal::bist_carrier_mode_2);
            }
            break;
        case bist::mode::test_data:
            if (derived().sm_.template annotation<ready_for_atomic_message_sequence>()) {
                setBistTestData(true);
            }
            break;
        default: break; // other modes are not supported
        }
    }

    void setBistTestData(bool enable)
    {
        bist_test_data_ = enable;
        if constexpr (requires { tcpc_.setBistTestData(enable); }) {
            tcpc_.setBistTestData(enable); // hardware may discard for us
        }
    }

    TCPC& tcpc_;
    // the capability lists: the own role's from construction, the other
    // role's provided by a DRP (empty: not a DRP, Get_*_Cap answers
    // Not_Supported)
    std::span<sink_capability const> sink_capabilities_{};
    std::span<std::uint32_t const> source_capabilities_{};
    bool bist_test_data_ = false;
    PrlPort port_{*this};
    ProtocolLayer<TCPC, TIMER> prl_; // also an observer of the machine
    fsm::QueuedTimer<TIMER> pe_timer_;
    fsm::timed<fsm::QueuedTimer<TIMER>&> timed_;
};

} // namespace pe

} // namespace usbc
