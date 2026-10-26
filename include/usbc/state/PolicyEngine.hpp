/*
 * Common ground of the USB PD policy engines' tables: the timers both
 * roles use, the events the protocol layer feeds into either engine,
 * the message builders, the connection context, the states both
 * engines share (the spec's role-neutral PE_DRS_/PE_PRS_/PE_VCS_
 * states and the resets and service states whose PE_SNK_/PE_SRC_ twins
 * differ in nothing but the role in the messages they build) with the
 * swap exchanges' sub-tables, and the guards both tables ask. The
 * facts the states publish live in annotation/PolicyEngine.hpp, the
 * questions they ask in interface/Questions.hpp, the engine base in
 * layer/PolicyEngine.hpp; the sink and source tables are
 * SinkPolicyEngine.hpp and SourcePolicyEngine.hpp next to this file.
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

#include <usbc/annotation/PolicyEngine.hpp>
#include <usbc/annotation/ProtocolLayer.hpp>
#include <usbc/annotation/TypeC.hpp>
#include <usbc/interface/Policy.hpp>
#include <usbc/interface/Questions.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Parameters.hpp>
#include <usbc/spec/Pdo.hpp>
#include <usbc/spec/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace usbc {

namespace concepts {

// The partner's swap request as an event: carries the refusal the
// catch-all row answers with when the table's question says no
template<typename T>
concept swap_request = requires(T const& request) {
    { request.refusal } -> std::convertible_to<control_message_type>;
};

} // namespace concepts

namespace pe {

inline constexpr auto t_sender_response = std::chrono::milliseconds{27}; // tSenderResponse
inline constexpr auto t_chunking_not_supported =
    std::chrono::milliseconds{45};                                      // tChunkingNotSupported
inline constexpr auto t_bist_cont_mode = std::chrono::milliseconds{45}; // tBISTContMode
inline constexpr auto t_pr_swap_wait = std::chrono::milliseconds{150};  // tPRSwapWait
inline constexpr auto t_dr_swap_wait = std::chrono::milliseconds{150};  // tDRSwapWait

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
    data_role data = data_role::ufp;     // flipped by an agreed DR_Swap
    std::uint8_t hard_reset_counter = 0; // HardResetCounter
    bool attached = false; // the source's restore asks whether the sink is still there
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
// guarded Ready row accepts when the injected policy answers yes
// (bool check(pe::pr_swap_allowed); the event carries the role this
// port would take for an answer wanting it), the catch-all sends the
// refusal: Reject, or Not_Supported without the feature (the Accept
// is built by the accepting state)
struct pr_swap_received {
    control_message_type refusal;
    power_role to_take;
};
struct dr_swap_received {
    control_message_type refusal;
    data_role to_take;
};
struct swap_wait_source_on { // the port asserted Rd mid PR_Swap: this
    data_role role;          // engine announces the supply is off
};
struct attached_swap { // activation continuing a PR_Swap (new source)
    data_role role;
};
struct swap_done {}; // advances the PR_Swap handover's transient state
// A swap exchange's word to the engine's table, emitted by the final
// states of its submachine (never processed from outside)
struct swap_refused {};    // our request met Reject, or no answer in time
struct swap_finished {};   // the exchange is complete: back to Ready
struct hand_off_failed {}; // the VCONN PS_RDY never left: hard reset
struct bist_carrier {};    // BIST Carrier Mode 2 requested (vSafe5V)

// VCONN_Swap messaging, shared by both engine roles; the role and
// switch choreography lives in the vconn machine (state/Vconn.hpp)
struct send_vconn_swap {};   // our VCONN_Swap goes out
struct vconn_swap_received { // the partner's VCONN_Swap
    control_message_type refusal;
};
struct send_vconn_ps_rdy {};  // the vconn machine turned the switch on
struct hard_reset_request {}; // port-level escalation (vconn timeout)
struct unsupported {};        // answered with Not_Supported from Ready
struct chunked_message {};    // a chunked extended message arrived
struct hard_reset_complete {};
struct hard_reset_received {};
// The capability lists live with the engine facade, so their messages
// travel pre-built: Get_Sink_Cap answered in either role
struct send_sink_capabilities {
    pd_message message;
};

} // namespace event

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

// Annotation tag the engines' facades query off the machine
// (fsm::annotation<T>) instead of enumerating states: Ready - no
// Atomic Message Sequence running, one may start (the spec allows
// swaps only there, under the explicit contract)
struct ready_for_atomic_message_sequence {
    constexpr bool operator==(ready_for_atomic_message_sequence const&) const = default;
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
// the spec has them, so its table reads against the spec.
//
// The swap states are sub-states: each swap exchange is a composite
// state of the engine's table (the spec's PE_DRS_/PE_PRS_/PE_VCS_
// diagram, one composite per entry from Ready - the request we sent,
// the request we answered), whose submachine runs the exchange and
// ends in a final state emitting the outcome. Every root state carries
// the spec's power level in its set; the composite carries it for its
// sub-states, which keep the plain member for their diagram note (a
// child machine notifies its initial state's annotations on
// construction, and an annotation type may not recur below the
// composite carrying it). A sub-state's protocol_error falls through
// to the composite's one row
namespace state {

// PE_SNK_/PE_SRC_Send_Not_Supported: answers a message the port does
// not support - or a swap request the table's question refused - then
// returns to Ready
template<power_role ROLE>
struct pe_send_not_supported {
    pe_send_not_supported(event::unsupported const&, pe_connection& connection_ref)
        : pe_send_not_supported(connection_ref)
    {
    }
    // a swap request the table's question refused (or nobody answers)
    template<concepts::swap_request REQUEST>
    pe_send_not_supported(REQUEST const& request, pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(request.refusal, ROLE, connection.data))
    {
    }
    // the chunking timeout ran out: Not_Supported as well
    explicit pe_send_not_supported(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::not_supported, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Chunk_Received: a non-chunking device lets the sender
// run into its chunking timeout before answering Not_Supported
struct pe_chunk_received {
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    static constexpr auto timeout = t_chunking_not_supported; // ChunkingNotSupportedTimer
};

// PE_SNK_/PE_SRC_BIST_Carrier_Mode: the tester's carrier runs for
// tBISTContMode (the engine commanded the TCPC on entry), then normal
// operation resumes
struct pe_bist_carrier {
    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "transmits the BIST carrier";

    static constexpr auto annotations = fsm::annotate(power, pd);

    static constexpr auto timeout = t_bist_cont_mode; // BISTContModeTimer
};

// PE_SNK_Give_Sink_Cap / PE_DR_SRC_Give_Sink_Cap: Get_Sink_Cap answered
// with the sink-role capabilities, then back to Ready
struct pe_give_sink_cap {
    pe_give_sink_cap() = default;
    explicit pe_give_sink_cap(event::send_sink_capabilities const& event) : message_(event.message)
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    pd_message const& values() const { return message_; }

private:
    pd_message message_{};
};

// PE_DRS_*_Send_Swap: our DR_Swap is out; no answer within
// tSenderResponse means the partner ignored it - stay Ready
template<power_role ROLE>
struct pe_drs_send_swap {
    using feature = dr_swap_feature;

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

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends DR_Swap";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_DRS_*_Accept_Swap: the partner's DR_Swap passed the arbitration
template<power_role ROLE>
struct pe_drs_accept_swap {
    using feature = dr_swap_feature;

    pe_drs_accept_swap(event::dr_swap_received const&, pe_connection& connection_ref)
        : pe_drs_accept_swap(connection_ref)
    {
    }
    explicit pe_drs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends Accept";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_DRS_*_Change_to_*: the agreed swap flips the data role; the
// report lets the engine's layer port stamp the header, the port flip
// the Type-C context, and an application's mux follow the role. The
// exchange ends here: the final state's word returns to Ready
struct pe_drs_change_data_role {
    using feature = dr_swap_feature;
    using emits = event::swap_finished;

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

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "flips the data role";

    data_role_changed values() const { return {.role = connection.data}; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;
};

// PE_VCS_Send_Swap: our VCONN_Swap is out
template<power_role ROLE>
struct pe_vcs_send_swap {
    using feature = vconn_feature;

    pe_vcs_send_swap(event::send_vconn_swap const&, pe_connection& connection_ref)
        : pe_vcs_send_swap(connection_ref)
    {
    }
    explicit pe_vcs_send_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::vconn_swap, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends VCONN_Swap";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_VCS_Accept_Swap: the partner's VCONN_Swap passed the arbitration
template<power_role ROLE>
struct pe_vcs_accept_swap {
    using feature = vconn_feature;

    pe_vcs_accept_swap(event::vconn_swap_received const&, pe_connection& connection_ref)
        : pe_vcs_accept_swap(connection_ref)
    {
    }
    explicit pe_vcs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends Accept";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

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
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        vconn_swap_agreed{}
    );
};

// PE_VCS_Send_PS_RDY: the vconn machine turned the switch on
template<power_role ROLE>
struct pe_vcs_send_ps_rdy {
    using feature = vconn_feature;

    pe_vcs_send_ps_rdy(event::send_vconn_ps_rdy const&, pe_connection& connection_ref)
        : pe_vcs_send_ps_rdy(connection_ref)
    {
    }
    explicit pe_vcs_send_ps_rdy(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::ps_rdy, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends PS_RDY";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_;
};

// The hand-off's ends, reporting its progress to the vconn machine:
// the final states of the VCONN_Swap exchange
struct pe_vcs_partner_on {
    using feature = vconn_feature;
    using emits = event::swap_finished;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        vconn_partner_on{}
    );
};

struct pe_vcs_ps_rdy_sent {
    using feature = vconn_feature;
    using emits = event::swap_finished;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        vconn_ps_rdy_sent{}
    );
};

// Our VCONN PS_RDY failed to go out: the exchange ends in a hard reset
struct pe_vcs_hand_off_failed {
    using feature = vconn_feature;
    using emits = event::hand_off_failed;
};

// PE_PRS_*_Send_Swap: our PR_Swap is out
template<power_role ROLE>
struct pe_prs_send_swap {
    using feature = pr_swap_feature;

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

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends PR_Swap";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_PRS_*_Accept_Swap: the partner's PR_Swap passed arbitration
template<power_role ROLE>
struct pe_prs_accept_swap {
    using feature = pr_swap_feature;

    pe_prs_accept_swap(event::pr_swap_received const&, pe_connection& connection_ref)
        : pe_prs_accept_swap(connection_ref)
    {
    }
    explicit pe_prs_accept_swap(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends Accept";

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_;
};

// The partner answered Wait: the swap request is retried after the
// spec's pause (still Ready, spec-wise)
struct pe_dr_swap_wait {
    using feature = dr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto timeout = t_dr_swap_wait; // tDRSwapWait
};

struct pe_pr_swap_wait {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto timeout = t_pr_swap_wait; // tPRSwapWait
};

// Our swap request was refused - Reject, or no answer within
// tSenderResponse (the partner ignored it): the exchange ends, Ready
// resumes
struct pe_swap_refused {
    using emits = event::swap_refused;
};

// --- the swap exchanges both engines run alike ---------------------------------
//
// One sub-table per swap kind - the spec's diagram in one piece, with
// its two entries - and two composite states over it: the request we
// sent starts the submachine at its initial state (Send_Swap), the
// request we answered names Accept_Swap as its initial. A composite
// carries the feature tag, the power element (an explicit contract
// holds throughout) and the submachine; the sub-states inherit the
// connection for the messages they build. The engine's table enters
// a composite from Ready and takes the outcome the final states emit

// PE_VCS_*: our VCONN_Swap out (Wait is not retried), or the
// partner's accepted; after the agreement the vconn machine owns the
// switch and the engine relays the PS_RDY traffic until its side is
// done
template<power_role ROLE>
using pe_vcs_transitions = fsm::transition_table<
    fsm::initial<pe_vcs_send_swap<ROLE>>,
    fsm::transition<fsm::from<pe_vcs_send_swap<ROLE>>, fsm::on<event::accept>, fsm::to<pe_vcs_active>>,
    fsm::transition<fsm::from<pe_vcs_send_swap<ROLE>>, fsm::on<event::reject>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_vcs_send_swap<ROLE>>, fsm::on<event::wait>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_vcs_send_swap<ROLE>>, fsm::on<fsm::timeout>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_vcs_accept_swap<ROLE>>, fsm::on<event::message_sent>, fsm::to<pe_vcs_active>>,
    fsm::transition<fsm::from<pe_vcs_active>, fsm::on<event::ps_rdy>, fsm::to<pe_vcs_partner_on>>,
    fsm::transition<fsm::from<pe_vcs_active>, fsm::on<event::send_vconn_ps_rdy>, fsm::to<pe_vcs_send_ps_rdy<ROLE>>>,
    fsm::transition<fsm::from<pe_vcs_send_ps_rdy<ROLE>>, fsm::on<event::message_sent>, fsm::to<pe_vcs_ps_rdy_sent>>,
    fsm::transition<fsm::from<pe_vcs_send_ps_rdy<ROLE>>, fsm::on<event::protocol_error>, fsm::to<pe_vcs_hand_off_failed>>,
    fsm::final<pe_swap_refused>,
    fsm::final<pe_vcs_partner_on>,
    fsm::final<pe_vcs_ps_rdy_sent>,
    fsm::final<pe_vcs_hand_off_failed>>;

template<power_role ROLE>
struct pe_vcs_table : pe_vcs_transitions<ROLE> {};

template<power_role ROLE>
struct pe_vcs_initiated {
    using feature = vconn_feature;
    using submachine = pe_vcs_table<ROLE>;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

template<power_role ROLE>
struct pe_vcs_responded {
    using feature = vconn_feature;
    using submachine = pe_vcs_table<ROLE>;
    using initial = pe_vcs_accept_swap<ROLE>;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

// PE_DRS_*_Accept_Swap and the flip, the answered half of the DR_Swap
// sub-tables (the sent half differs per role - the sink gates its
// retry on SinkTxOk - so each engine composes its own)
template<power_role ROLE>
using pe_drs_responded_transitions = fsm::transition_table<
    fsm::transition<fsm::from<pe_drs_accept_swap<ROLE>>, fsm::on<event::message_sent>, fsm::to<pe_drs_change_data_role>>,
    fsm::final<pe_drs_change_data_role>>;

// PE_SNK_/PE_SRC_Soft_Reset: accepts a received Soft_Reset; the
// protocol layer resets before the Accept goes out (guaranteed hook
// order)
template<power_role ROLE>
struct pe_soft_reset {
    pe_soft_reset(event::soft_reset_received const&, pe_connection& connection_ref)
        : pe_soft_reset(connection_ref)
    {
    }
    explicit pe_soft_reset(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::accept, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = prl::reset_action::note;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        prl::reset_action{}
    );

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Send_Soft_Reset: protocol errors first try a soft
// reset; the protocol layer is reset before the Soft_Reset goes out
template<power_role ROLE>
struct pe_send_soft_reset {
    explicit pe_send_soft_reset(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::soft_reset, ROLE, connection.data))
    {
    }

    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = prl::reset_action::note;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        prl::reset_action{}
    );

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_connection& connection;

private:
    pd_message message_;
};

// PE_SNK_/PE_SRC_Hard_Reset: the signal goes out, HardResetCounter counts
struct pe_hard_reset {
    explicit pe_hard_reset(pe_connection& connection_ref) : connection(connection_ref)
    {
        ++connection.hard_reset_counter;
    }

    static constexpr power_level power = power_level::contract_or_default;
    static constexpr std::string_view dot_action = prl::hard_reset_action::note;

    static constexpr auto annotations = fsm::annotate(
        power,
        prl::hard_reset_action{}
    );

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;
};

// nHardResetCount exhausted with no response: the port-level
// integration commands Type-C Error Recovery, whose teardown resets
// the engine
struct pe_error_recovery {
    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::not_connected;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        tc::request_error_recovery{}
    );
};

} // namespace state

// --- guards both tables ask ---------------------------------------------------

// HardResetCounter still allows another hard reset
struct hard_resets_left {
    static bool check(auto const& state)
    {
        return state.connection.hard_reset_counter <= spec::n_hard_reset_count;
    }
};

// An explicit contract is in place (the negotiation context says so)
struct explicit_contract_holds {
    static bool check(auto const& state) { return state.negotiation.explicit_contract; }
};

} // namespace pe

} // namespace usbc
