/*
 * USB PD sink policy engine (PE_SNK), the table: the states, the
 * swap exchanges' sub-tables and the guards. The engine running it is
 * usbc::SinkPolicyEngine (layer/SinkPolicyEngine.hpp).
 *
 * The application injects the sink's possible contracts as a span of
 * sink_capability entries (also answered to Get_Sink_Cap) and a policy
 * that selects the contract from the source's capabilities; PowerPolicy
 * (interface/Policy.hpp) is the power-based default. The engine negotiates: waits for
 * Source_Capabilities (SinkWaitCapTimer), evaluates through the policy,
 * requests (SenderResponseTimer), transitions the sink load to iSnkStdby
 * until PS_RDY (PSTransitionTimer), then applies the contract through
 * the injected SinkPower observer. Reject without an explicit
 * contract falls back to waiting for capabilities; protocol timeouts
 * and transmission errors escalate to a hard reset, Soft_Reset is
 * accepted and restarts negotiation. When the policy finds nothing
 * acceptable, the first source PDO is requested at its full current
 * with the Capability Mismatch flag.
 *
 * Unsupported messages are answered with Not_Supported from Ready
 * (PE_SNK_Send_Not_Supported); chunked extended messages are handled as
 * a non-chunking device: PE_SNK_Chunk_Received waits
 * ChunkingNotSupportedTimer, then answers Not_Supported.
 *
 * The machine follows the spec's PE_SNK state diagram: Startup (with
 * the mandatory protocol layer reset) -> Discovery -> Wait_for_
 * Capabilities -> Evaluate_Capability -> Select_Capability ->
 * Transition_Sink -> Ready, with Send_Soft_Reset on protocol errors,
 * Soft_Reset answering a received one, Hard_Reset ->
 * Transition_to_default on timeouts, and Give_Sink_Cap /
 * Send_Not_Supported / Chunk_Received serving Ready. Transient spec
 * states advance through events the engine injects sequentially;
 * Discovery -> Wait_for_Capabilities is driven by the externally
 * reported VBUS.
 *
 * Deviations kept for later: GotoMin answers Not_Supported, and
 * Discovery does not sense VBUS itself - the port layer owns the
 * sensing and reports it (a sink's attach implies VBUS; after a hard
 * reset the connection layer's hard-reset window re-reports the
 * returning VBUS, and wait_no_response governs the gap). Wait answers
 * are retried (tSinkRequest/tPRSwapWait/tDRSwapWait), hard resets are
 * counted (nHardResetCount, then Error Recovery is requested through
 * portReport), and PR_Swap/DR_Swap run their full exchanges.
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
#include <usbc/state/PolicyEngine.hpp>

#include <mtl/StateMachine.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace usbc {

namespace pe {

inline constexpr auto t_sink_wait_cap = std::chrono::milliseconds{465}; // tSinkWaitCap
inline constexpr auto t_ps_transition = std::chrono::milliseconds{500}; // tPSTransition
inline constexpr auto t_no_response = std::chrono::milliseconds{5000};  // tNoResponse
inline constexpr auto t_sink_request = std::chrono::milliseconds{150};  // tSinkRequest

// Machine-owned context, the sink's negotiation: one negotiation's
// worth, gone with every reset within the connection (the connection
// itself is the shared pe_connection)
struct pe_negotiation {
    contract_request pending{}; // proposed by the last Request
    contract_request request{}; // accepted by the source
    bool explicit_contract = false;
};

namespace event {

struct started {};
struct vbus_present {};
struct vbus_removed {};
struct source_capabilities {};
struct capabilities_evaluated { // the policy's pick; the state builds the Request
    contract_request terms;
};
struct send_source_caps { // a DRP asked for its source-role caps
    pd_message message;
};
struct request_retry {}; // SinkRequestTimer expired, SinkTxOk seen
struct default_level_reached {};

} // namespace event

namespace state {

// PE_SNK_Startup: the protocol layer reset here is mandatory; the
// default power restore covers the detach entry (after a hard reset,
// Transition_to_default already restored and suppression elides it)
struct pe_snk_startup {
    // detach forgets everything (the next partner meets a UFP); a reset
    // within the connection ends the negotiation only
    pe_snk_startup(
        event::vbus_removed const&,
        pe_connection& connection,
        pe_negotiation& negotiation
    )
        : connection(connection), negotiation(negotiation)
    {
        this->connection = {.data = defaultDataRole(power_role::sink)};
        this->negotiation = {};
    }
    explicit pe_snk_startup(pe_connection& connection, pe_negotiation& negotiation)
        : connection(connection), negotiation(negotiation)
    {
        this->negotiation = {};
    }

    static constexpr power_level power = power_level::default_power;
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    static constexpr auto annotations = fsm::annotate(
        power,
        prl::reset_action{},
        restore_default_action{}
    );

    using contexts = fsm::contexts<pe_connection, pe_negotiation>;

    pe_connection& connection;
    pe_negotiation& negotiation;
};

// Waits for the Type-C layer to report VBUS
struct pe_snk_discovery {
    explicit pe_snk_discovery(pe_connection& connection) : connection(connection) {}

    static constexpr power_level power = power_level::default_power;

    static constexpr auto annotations = fsm::annotate(power);

    using contexts = fsm::contexts<pe_connection>; // the guard reads the counter

    pe_connection& connection;
};

struct pe_snk_wait_for_capabilities {
    static constexpr power_level power = power_level::default_power;

    static constexpr auto annotations = fsm::annotate(power);

    static constexpr auto timeout = t_sink_wait_cap; // SinkWaitCapTimer
};

// PE_SNK_Wait_for_Capabilities after a hard reset: the governing
// deadline is NoResponseTimer - its expiry hard-resets again while
// HardResetCounter allows, then gives up into Type-C Error Recovery
struct pe_snk_wait_no_response {
    explicit pe_snk_wait_no_response(pe_connection& connection) : connection(connection) {}

    static constexpr power_level power = power_level::default_power;

    static constexpr auto annotations = fsm::annotate(power);

    using contexts = fsm::contexts<pe_connection>; // the guard reads the counter

    static constexpr auto timeout = t_no_response; // NoResponseTimer

    pe_connection& connection;
};

// The engine evaluates through the injected policy and advances with
// capabilities_evaluated
struct pe_snk_evaluate_capability {
    explicit pe_snk_evaluate_capability(pe_connection& connection) : connection(connection)
    {
        connection.hard_reset_counter = 0; // spec: reset on Source_Capabilities
    }

    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;
};

struct pe_snk_select_capability {
    // the proposal stays pending: only an Accept promotes it, so a
    // Reject cannot leak the proposed terms into the active contract
    pe_snk_select_capability(
        event::capabilities_evaluated const& event,
        pe_connection& connection_ref,
        pe_negotiation& negotiation_ref
    )
        : connection(connection_ref), negotiation(negotiation_ref)
    {
        negotiation.pending = event.terms;
        message_ = requestMessage();
    }
    // re-entry from the SinkRequestTimer: the same Request again
    pe_snk_select_capability(
        event::request_retry const&,
        pe_connection& connection_ref,
        pe_negotiation& negotiation_ref
    )
        : pe_snk_select_capability(connection_ref, negotiation_ref)
    {
    }
    pe_snk_select_capability(pe_connection& connection_ref, pe_negotiation& negotiation_ref)
        : connection(connection_ref), negotiation(negotiation_ref), message_(requestMessage())
    {
    }

    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection, pe_negotiation>;

    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_connection& connection;
    pe_negotiation& negotiation;

private:
    // the Request for the pending terms: one RDO
    pd_message requestMessage() const
    {
        auto const& terms = negotiation.pending;
        std::array<std::uint32_t, 1> const objects{pdo::makeFixedRequest(
            terms.position,
            terms.operating_current,
            terms.maximum_current,
            terms.mismatch
        )};
        return makeDataMessage(
            data_message_type::request,
            power_role::sink,
            connection.data,
            objects
        );
    }

    pd_message message_{};
};

struct pe_snk_transition_sink {
    // entered on Accept: the pending proposal becomes the contract
    pe_snk_transition_sink(event::accept const&, pe_negotiation& negotiation)
        : negotiation(negotiation)
    {
        negotiation.request = negotiation.pending;
    }
    explicit pe_snk_transition_sink(pe_negotiation& negotiation) : negotiation(negotiation) {}

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    standby_limit values() const { return {negotiation.request.voltage}; }

    using contexts = fsm::contexts<pe_negotiation>;

    static constexpr auto timeout = t_ps_transition; // PSTransitionTimer

    pe_negotiation& negotiation;
};

struct pe_snk_ready {
    explicit pe_snk_ready(pe_negotiation& negotiation) : negotiation(negotiation)
    {
        negotiation.explicit_contract = true;
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        ready_for_atomic_message_sequence{}
    );

    active_contract values() const
    {
        return {negotiation.request.voltage, negotiation.request.operating_current};
    }

    using contexts = fsm::contexts<pe_negotiation>;

    pe_negotiation& negotiation;
};

// The spec's Ready-with-SinkRequestTimer after a Wait answer to our
// Request: the same Request goes out again after tSinkRequest; new
// capabilities from the source preempt the retry
struct pe_snk_request_wait {
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    static constexpr auto timeout = t_sink_request; // SinkRequestTimer
};

// PE_DR_SNK_Give_Source_Cap: a DRP answers Get_Source_Cap with its
// source-role capabilities, then returns to Ready
struct pe_dr_snk_give_source_cap {
    pe_dr_snk_give_source_cap() = default;
    explicit pe_dr_snk_give_source_cap(event::send_source_caps const& event)
        : message_(event.message)
    {
    }

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    pd_message const& values() const { return message_; }

private:
    pd_message message_{};
};

// --- PR_Swap, sink side (PE_PRS_SNK_SRC_*) ------------------------------------

// The retry is due but PD3 collision avoidance gates it: the engine
// re-initiates once the source's Rp says SinkTxOk (no timeout - the
// source owns the schedule)
struct pe_snk_request_gate {
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        retry_gated{atomic_message_sequence::request}
    );
};

struct pe_snk_dr_swap_gate {
    using feature = dr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        retry_gated{atomic_message_sequence::data_role_swap}
    );
};

struct pe_snk_pr_swap_gate {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        retry_gated{atomic_message_sequence::power_role_swap}
    );
};

// PE_PRS_SNK_SRC_Transition_to_off: draw drops to standby while the
// old source turns off; the port holds the connection layer's swap
// standby, whose tPSSourceOff timeout restarts connection resolution
// (the spec's Error Recovery outcome) if the PS_RDY never comes
struct pe_snk_swap_transition_to_off {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "stops drawing, awaits the source's PS_RDY";

    // the standby draw for the power side, the standby entry for the
    // port - both compile-time facts of this state
    static constexpr auto annotations = fsm::annotate(
        standby_limit{v_safe_5v},
        tc::enter_swap_standby{power_role::source}
    );
};

// PE_PRS_SNK_SRC_Assert_Rp: the old source is off - the port flips its
// termination now and continues as the new source
struct pe_snk_swap_assert_rp {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        tc::assert_new_role{power_role::source}
    );
};

// PE_PRS_SRC_SNK_Wait_Source_on, this engine's half: the port was the
// source and asserted Rd - our PS_RDY announces the supply is off, the
// new source's PS_RDY is awaited (the connection layer's swap standby
// holds the tPSSourceOn deadline and restarts resolution on timeout)
struct pe_snk_swap_wait_source_on {
    using feature = pr_swap_feature;

    pe_snk_swap_wait_source_on(event::swap_wait_source_on const& event, pe_connection& connection)
        : connection(connection)
    {
        connection.data = event.role; // a power swap preserves the data role
        message_ =
            makeControlMessage(control_message_type::ps_rdy, power_role::sink, connection.data);
    }
    explicit pe_snk_swap_wait_source_on(pe_connection& connection) : connection(connection) {}

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_action = "sends PS_RDY, awaits the new source's";

    static constexpr auto annotations = fsm::annotate(power, pd);

    pd_message const& values() const { return message_; }

    using contexts = fsm::contexts<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_{};
};

// The new source's PS_RDY arrived: the port completes the swap into
// Attached.SNK, then the normal sink flow resumes (swap_done, pumped
// by the facade)
struct pe_snk_swap_source_on_seen {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        tc::swap_completed{}
    );
};

// PE_SNK_Transition_to_default: back to vSafe5V defaults; the engine
// then advances through Startup and Discovery
struct pe_snk_transition_to_default {
    // the connection persists, the negotiation ends
    explicit pe_snk_transition_to_default(pe_negotiation& negotiation) : negotiation(negotiation)
    {
        this->negotiation = {};
    }

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::not_connected;
    static constexpr std::string_view dot_action = restore_default_action::note;

    // the restore for the power side, the window for the port: the
    // connection layer must hold the attach while VBUS cycles
    static constexpr auto annotations = fsm::annotate(
        power,
        pd,
        restore_default_action{},
        tc::hard_reset_window{}
    );

    using contexts = fsm::contexts<pe_negotiation>;

    pe_negotiation& negotiation;
};

// The states shared with the source engine (state/PolicyEngine.hpp),
// under the spec's sink-side names where it has them
using pe_snk_error_recovery = pe_error_recovery;
using pe_snk_give_sink_cap = pe_give_sink_cap;
using pe_snk_bist_carrier = pe_bist_carrier;
using pe_snk_send_not_supported = pe_send_not_supported<power_role::sink>;
using pe_snk_chunk_received = pe_chunk_received;
using pe_snk_send_dr_swap = pe_drs_send_swap<power_role::sink>;
using pe_snk_accept_dr_swap = pe_drs_accept_swap<power_role::sink>;
using pe_snk_dr_swap_change = pe_drs_change_data_role;
using pe_snk_vcs_send_swap = pe_vcs_send_swap<power_role::sink>;
using pe_snk_send_pr_swap = pe_prs_send_swap<power_role::sink>;
using pe_snk_accept_pr_swap = pe_prs_accept_swap<power_role::sink>;
using pe_snk_dr_swap_wait = pe_dr_swap_wait;
using pe_snk_pr_swap_wait = pe_pr_swap_wait;
using pe_snk_soft_reset = pe_soft_reset<power_role::sink>;
using pe_snk_send_soft_reset = pe_send_soft_reset<power_role::sink>;
using pe_snk_hard_reset = pe_hard_reset;

// --- the swap exchanges, composite states of the sink's table ----------------
//
// One sub-table per swap kind, two composites over it: the request we
// sent starts it at Send_Swap, the request we answered at Accept_Swap.
// The VCONN exchange is the shared one (state/PolicyEngine.hpp); a
// sent request's retry after Wait passes the sink's SinkTxOk gate
using pe_snk_vconn_swap_initiated = pe_vcs_initiated<power_role::sink>;
using pe_snk_vconn_swap_responded = pe_vcs_responded<power_role::sink>;

// PE_DRS_UFP_DFP_*: Send_Swap and the retry, or Accept_Swap, then the flip
using pe_snk_dr_swap_transitions = fsm::transition_table<
    fsm::initial<pe_snk_send_dr_swap>,
    fsm::transition<fsm::from<pe_snk_send_dr_swap>, fsm::on<event::accept>, fsm::to<pe_snk_dr_swap_change>>,
    fsm::transition<fsm::from<pe_snk_send_dr_swap>, fsm::on<event::reject>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_snk_send_dr_swap>, fsm::on<fsm::timeout>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_snk_send_dr_swap>, fsm::on<event::wait>, fsm::to<pe_snk_dr_swap_wait>>,
    fsm::transition<fsm::from<pe_snk_dr_swap_wait>, fsm::on<fsm::timeout>, fsm::to<pe_snk_dr_swap_gate>>,
    fsm::transition<fsm::from<pe_snk_dr_swap_gate>, fsm::on<event::send_dr_swap>, fsm::to<pe_snk_send_dr_swap>>,
    pe_drs_responded_transitions<power_role::sink>,
    fsm::final<pe_swap_refused>>;

struct pe_snk_dr_swap_table : pe_snk_dr_swap_transitions {};

struct pe_snk_dr_swap_initiated {
    using feature = dr_swap_feature;
    using submachine = pe_snk_dr_swap_table;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

struct pe_snk_dr_swap_responded {
    using feature = dr_swap_feature;
    using submachine = pe_snk_dr_swap_table;
    using initial = pe_snk_accept_dr_swap;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

// PE_PRS_SNK_SRC_*: Send_Swap and the retry, or Accept_Swap; the
// agreement leads into Transition_to_off, the old source's PS_RDY into
// the termination flip, where this engine's part ends (the port
// completes the swap, the source engine takes over)
using pe_snk_pr_swap_transitions = fsm::transition_table<
    fsm::initial<pe_snk_send_pr_swap>,
    fsm::transition<fsm::from<pe_snk_send_pr_swap>, fsm::on<event::accept>, fsm::to<pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<pe_snk_send_pr_swap>, fsm::on<event::reject>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_snk_send_pr_swap>, fsm::on<fsm::timeout>, fsm::to<pe_swap_refused>>,
    fsm::transition<fsm::from<pe_snk_send_pr_swap>, fsm::on<event::wait>, fsm::to<pe_snk_pr_swap_wait>>,
    fsm::transition<fsm::from<pe_snk_pr_swap_wait>, fsm::on<fsm::timeout>, fsm::to<pe_snk_pr_swap_gate>>,
    fsm::transition<fsm::from<pe_snk_pr_swap_gate>, fsm::on<event::send_pr_swap>, fsm::to<pe_snk_send_pr_swap>>,
    fsm::transition<fsm::from<pe_snk_accept_pr_swap>, fsm::on<event::message_sent>, fsm::to<pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<pe_snk_swap_transition_to_off>, fsm::on<event::ps_rdy>, fsm::to<pe_snk_swap_assert_rp>>,
    fsm::final<pe_swap_refused>>;

struct pe_snk_pr_swap_table : pe_snk_pr_swap_transitions {};

struct pe_snk_pr_swap_initiated {
    using feature = pr_swap_feature;
    using submachine = pe_snk_pr_swap_table;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

struct pe_snk_pr_swap_responded {
    using feature = pr_swap_feature;
    using submachine = pe_snk_pr_swap_table;
    using initial = pe_snk_accept_pr_swap;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;

    static constexpr auto annotations = fsm::annotate(power, pd);

    using parent_contexts = fsm::contexts<pe_connection>;
};

} // namespace state

// Which capability wait applies: SinkWaitCapTimer before the first
// hard reset, NoResponseTimer afterwards
struct no_hard_reset_yet {
    static bool check(state::pe_snk_discovery const& state)
    {
        return state.connection.hard_reset_counter == 0;
    }
};

// The spec timer range of every timed state, checked against the table
using sink_timer_ranges = fsm::timer_ranges<
    fsm::timed_by<state::pe_snk_wait_for_capabilities, spec::t_sink_wait_cap>,
    fsm::timed_by<state::pe_snk_select_capability, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_transition_sink, spec::t_ps_transition>,
    fsm::timed_by<state::pe_snk_chunk_received, spec::t_chunking_not_supported>,
    fsm::timed_by<state::pe_snk_send_soft_reset, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_request_wait, spec::t_sink_request>,
    fsm::timed_by<state::pe_snk_wait_no_response, spec::t_no_response>,
    fsm::timed_by<state::pe_snk_bist_carrier, spec::t_bist_cont_mode>,
    // inside the swap exchanges
    fsm::timed_by<state::pe_snk_send_dr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_send_pr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_dr_swap_wait, spec::t_dr_swap_wait>,
    fsm::timed_by<state::pe_snk_pr_swap_wait, spec::t_pr_swap_wait>,
    fsm::timed_by<state::pe_snk_vcs_send_swap, spec::t_sender_response>>;

using sink_transitions = fsm::transition_table<
    fsm::initial<state::pe_snk_startup>,
    fsm::transition<fsm::from<state::pe_snk_startup>, fsm::on<event::started>, fsm::to<state::pe_snk_discovery>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::vbus_removed>, fsm::to<state::pe_snk_startup>>,
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::vbus_present>, fsm::guard<no_hard_reset_yet>, fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::vbus_present>, fsm::to<state::pe_snk_wait_no_response>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>, fsm::on<event::source_capabilities>, fsm::to<state::pe_snk_evaluate_capability>>,
    // after a hard reset: another one while the counter allows, Error
    // Recovery when it is spent
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>, fsm::on<event::source_capabilities>, fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>, fsm::on<fsm::timeout>, fsm::guard<hard_resets_left>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_error_recovery>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::source_capabilities>, fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_evaluate_capability>, fsm::on<event::capabilities_evaluated>, fsm::to<state::pe_snk_select_capability>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::accept>, fsm::to<state::pe_snk_transition_sink>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::reject>, fsm::guard<explicit_contract_holds>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::reject>, fsm::to<state::pe_snk_wait_for_capabilities>>,
    // Wait: retry the same Request after tSinkRequest; fresh
    // capabilities preempt the retry
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::wait>, fsm::to<state::pe_snk_request_wait>>,
    fsm::transition<fsm::from<state::pe_snk_request_wait>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_request_gate>>,
    fsm::transition<fsm::from<state::pe_snk_request_gate>, fsm::on<event::request_retry>, fsm::to<state::pe_snk_select_capability>>,
    fsm::transition<fsm::from<state::pe_snk_request_gate>, fsm::on<event::source_capabilities>, fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_request_wait>, fsm::on<event::source_capabilities>, fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_transition_sink>, fsm::on<event::ps_rdy>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_transition_sink>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_sink_capabilities>, fsm::to<state::pe_snk_give_sink_cap>>,
    fsm::transition<fsm::from<state::pe_snk_give_sink_cap>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_give_sink_cap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_source_caps>, fsm::to<state::pe_dr_snk_give_source_cap>>,
    fsm::transition<fsm::from<state::pe_dr_snk_give_source_cap>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_dr_snk_give_source_cap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::unsupported>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::bist_carrier>, fsm::to<state::pe_snk_bist_carrier>>,
    fsm::transition<fsm::from<state::pe_snk_bist_carrier>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::chunked_message>, fsm::to<state::pe_snk_chunk_received>>,
    fsm::transition<fsm::from<state::pe_snk_chunk_received>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_send_not_supported>>,
    // The swap exchanges, each a composite state entered from Ready:
    // the partner's request is the table's question to the injected
    // policy, a refused one is answered from Send_Not_Supported. The
    // exchange's end emits its outcome, a sub-state's protocol error
    // falls through to the composite's row
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_dr_swap>, fsm::to<state::pe_snk_dr_swap_initiated>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_initiated>, fsm::on<event::swap_refused>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_initiated>, fsm::on<event::swap_finished>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_initiated>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::dr_swap_received>, fsm::guard<dr_swap_allowed>, fsm::to<state::pe_snk_dr_swap_responded>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::dr_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_responded>, fsm::on<event::swap_finished>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_responded>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    // VCONN_Swap: the vconn machine owns the role, the switch and the
    // hand-off deadline; its escalation and a lost PS_RDY hard-reset
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_vconn_swap>, fsm::to<state::pe_snk_vconn_swap_initiated>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_initiated>, fsm::on<event::swap_refused>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_initiated>, fsm::on<event::swap_finished>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_initiated>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_initiated>, fsm::on<event::hand_off_failed>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_initiated>, fsm::on<event::hard_reset_request>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_received>, fsm::guard<vconn_source_role_held>, fsm::to<state::pe_snk_vconn_swap_responded>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_received>, fsm::guard<vconn_swap_allowed>, fsm::to<state::pe_snk_vconn_swap_responded>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_responded>, fsm::on<event::swap_finished>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_responded>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_responded>, fsm::on<event::hand_off_failed>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vconn_swap_responded>, fsm::on<event::hard_reset_request>, fsm::to<state::pe_snk_hard_reset>>,
    // PR_Swap while sinking: an agreed exchange ends in the termination
    // flip, where the port hands over to the source engine
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_pr_swap>, fsm::to<state::pe_snk_pr_swap_initiated>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_initiated>, fsm::on<event::swap_refused>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_initiated>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::pr_swap_received>, fsm::guard<sourcing_allowed, pr_swap_allowed>, fsm::to<state::pe_snk_pr_swap_responded>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::pr_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_responded>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    // PR_Swap's other half: this port was the source and asserted Rd -
    // PS_RDY out, then the new source's PS_RDY completes the swap
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::swap_wait_source_on>, fsm::to<state::pe_snk_swap_wait_source_on>>,
    fsm::transition<fsm::from<state::pe_snk_swap_wait_source_on>, fsm::on<event::ps_rdy>, fsm::to<state::pe_snk_swap_source_on_seen>>,
    fsm::transition<fsm::from<state::pe_snk_swap_wait_source_on>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_swap_source_on_seen>, fsm::on<event::swap_done>, fsm::to<state::pe_snk_discovery>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::soft_reset_received>, fsm::to<state::pe_snk_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_soft_reset>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_soft_reset>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<event::accept>, fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_hard_reset>, fsm::on<event::hard_reset_complete>, fsm::to<state::pe_snk_transition_to_default>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_received>, fsm::to<state::pe_snk_transition_to_default>>,
    fsm::transition<fsm::from<state::pe_snk_transition_to_default>, fsm::on<event::default_level_reached>, fsm::to<state::pe_snk_startup>>>;

// The engine's table, every optional feature included: the machine
// leaves out the features its policy does not answer for. A named
// struct, not an alias: the short name replaces the fully spelled
// table type in every mangled symbol - megabytes per object file,
// measured. The short name is the machine id in trace lines and DOT
// graphs: unique across the stack
struct pe_sink_table : sink_transitions {};

// The table checks (timeout bounds, reachability, both variants)
// live in test/compliance.cpp - one dedicated TU pays for them

} // namespace pe

} // namespace usbc
