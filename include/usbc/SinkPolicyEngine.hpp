/*
 * USB PD sink policy engine (PE_SNK) on top of the protocol layer.
 *
 * The application injects the sink's possible contracts as a span of
 * sink_capability entries (also answered to Get_Sink_Cap) and a policy
 * that selects the contract from the source's capabilities; PowerPolicy
 * below is the power-based default. The engine negotiates: waits for
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
 * Integration: the engine drives a pd_transport driver through its own
 * ProtocolLayer - itself an observer of the engine's machine, executing
 * the states' prl::reset_action annotations and transmitting the
 * pd_message a state publishes through values() - and
 * runs from construction on, resting in PE_SNK_Discovery until VBUS is
 * reported. The application injects its own observers into the machine;
 * the power side is one of them: derive from SinkPower (CRTP) and
 * implement setLimit/onContract/onContractLost. Extra observers (e.g.
 * for logging) see exactly the annotated edges of the DOT diagram. Feed
 * TCPC alerts into onAlert() and the Type-C layer's attach/detach into
 * vbusPresent() and vbusRemoved() - a sink's attach implies VBUS. After
 * a hard reset the engine waits in Discovery for the Type-C layer to
 * report the returning VBUS. Everything runs in the stack's serialized
 * context; observer callbacks may originate from the timer context (mtl
 * timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Pdo.hpp>
#include <usbc/PolicyEngine.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/SinkLoad.hpp>
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

// A policy's answer: which source PDO to request and at what current
struct contract_request {
    std::uint8_t position = 0; // 1-based object position in the source capabilities
    millivolt voltage = 5000;
    milliamp operating_current = 0;
    milliamp maximum_current = 0;
    bool mismatch = false;
};

namespace concepts {

template<typename T>
concept sink_policy = requires(
    T policy,
    std::span<std::uint32_t const> source_capabilities,
    std::span<sink_capability const> capabilities
) {
    {
        policy.select(source_capabilities, capabilities)
    } -> std::same_as<std::optional<contract_request>>;
};

// The interface a SinkPower-derived class provides: the sink load
// limit plus the contract notifications
template<typename T>
concept sink_power_client = requires(T client, millivolt voltage, milliamp current) {
    { client.setLimit(voltage, current) } -> std::convertible_to<bool>;
    client.onContract(voltage, current);
    client.onContractLost();
};

} // namespace concepts

// Default selection policy: only source PDOs whose voltage the sink
// lists are considered, with the current capped by the sink capability
// and the power budget. A contract must deliver min_power; the lowest
// voltage delivering max_power wins. When no PDO reaches max_power the
// one with the most power wins, equal power prefers the lower voltage.
class PowerPolicy {
public:
    constexpr PowerPolicy(milliwatt min_power, milliwatt max_power)
        : min_power_(min_power), max_power_(max_power)
    {
    }

    constexpr std::optional<contract_request> select(
        std::span<std::uint32_t const> source_capabilities,
        std::span<sink_capability const> capabilities
    ) const
    {
        std::optional<contract_request> best;
        milliwatt best_power = 0;
        for (std::uint8_t index = 0; index < source_capabilities.size(); ++index) {
            auto const object = source_capabilities[index];
            if (pdo::kindOf(object) != pdo::kind::fixed_supply) {
                continue;
            }
            auto const voltage = pdo::fixedVoltage(object);
            auto const accepted = std::find_if(
                capabilities.begin(),
                capabilities.end(),
                [voltage](sink_capability capability) { return capability.voltage == voltage; }
            );
            if (accepted == capabilities.end()) {
                continue; // the sink cannot take this voltage
            }
            auto const wanted =
                static_cast<milliamp>((static_cast<std::int64_t>(max_power_) * 1000) / voltage);
            auto const current =
                std::min({pdo::fixedMaxCurrent(object), accepted->current, wanted});
            auto const power =
                static_cast<milliwatt>((static_cast<std::int64_t>(voltage) * current) / 1000);
            if (power < min_power_) {
                continue;
            }
            if (!best || power > best_power || (power == best_power && voltage < best->voltage)) {
                best_power = power;
                best = contract_request{
                    .position = static_cast<std::uint8_t>(index + 1),
                    .voltage = voltage,
                    .operating_current = current,
                    .maximum_current = current,
                    .mismatch = false
                };
            }
        }
        return best;
    }

private:
    milliwatt min_power_;
    milliwatt max_power_;
};
static_assert(concepts::sink_policy<PowerPolicy>);

namespace pe {

inline constexpr auto t_sink_wait_cap = std::chrono::milliseconds{465}; // tSinkWaitCap
inline constexpr auto t_ps_transition = std::chrono::milliseconds{500}; // tPSTransition
inline constexpr auto t_no_response = std::chrono::milliseconds{5000};  // tNoResponse
inline constexpr auto t_sink_request = std::chrono::milliseconds{150};  // tSinkRequest

inline constexpr milliamp i_snk_stdby = spec::i_snk_stdby; // at any voltage

// Machine-owned context, the sink's negotiation: one negotiation's
// worth, gone with every reset within the connection (the connection
// itself is the shared pe_connection)
struct pe_negotiation {
    contract_request pending{}; // proposed by the last Request
    contract_request request{}; // accepted by the source
    bool explicit_contract = false;
};

// The observation the sink's standby transition reports
struct standby_limit {
    millivolt voltage;
    constexpr bool operator==(standby_limit const&) const = default;
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
    static constexpr pd_status pd = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    static constexpr auto annotations = fsm::annotate(
        prl::reset_action{},
        restore_default_action{}
    );

    using contexts = mtl::typelist<pe_connection, pe_negotiation>;

    pe_connection& connection;
    pe_negotiation& negotiation;
};

// Waits for the Type-C layer to report VBUS
struct pe_snk_discovery {
    explicit pe_snk_discovery(pe_connection& connection) : connection(connection) {}

    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    using contexts = mtl::typelist<pe_connection>; // the guard reads the counter

    pe_connection& connection;
};

struct pe_snk_wait_for_capabilities {
    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto timeout = t_sink_wait_cap; // SinkWaitCapTimer
};

// PE_SNK_Wait_for_Capabilities after a hard reset: the governing
// deadline is NoResponseTimer - its expiry hard-resets again while
// HardResetCounter allows, then gives up into Type-C Error Recovery
struct pe_snk_wait_no_response {
    explicit pe_snk_wait_no_response(pe_connection& connection) : connection(connection) {}

    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    using contexts = mtl::typelist<pe_connection>; // the guard reads the counter

    static constexpr auto timeout = t_no_response; // NoResponseTimer

    pe_connection& connection;
};

// The engine evaluates through the injected policy and advances with
// capabilities_evaluated
struct pe_snk_evaluate_capability {
    explicit pe_snk_evaluate_capability(pe_connection& connection) : connection(connection)
    {
        connection.hard_resets = 0; // spec: reset on Source_Capabilities
    }

    static constexpr power_level power = power_level::default_power;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    using contexts = mtl::typelist<pe_connection>;

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
    static constexpr std::string_view dot_note = specNote(power, pd);

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection, pe_negotiation>;

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
    static constexpr std::string_view dot_note = specNote(power, pd);

    standby_limit values() const { return {negotiation.request.voltage}; }

    using contexts = mtl::typelist<pe_negotiation>;

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
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        power,
        ready_for_atomic_message_sequence{}
    );

    active_contract values() const
    {
        return {negotiation.request.voltage, negotiation.request.operating_current};
    }

    using contexts = mtl::typelist<pe_negotiation>;

    pe_negotiation& negotiation;
};

// The spec's Ready-with-SinkRequestTimer after a Wait answer to our
// Request: the same Request goes out again after tSinkRequest; new
// capabilities from the source preempt the retry
struct pe_snk_request_wait {
    static constexpr power_level power = power_level::contract_or_default;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

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
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        power
    );

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
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        retry_gated{atomic_message_sequence::request}
    );
};

struct pe_snk_dr_swap_gate {
    using feature = dr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        power,
        retry_gated{atomic_message_sequence::data_role_swap}
    );
};

struct pe_snk_pr_swap_gate {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::explicit_contract;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        power,
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
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "stops drawing, awaits the source's PS_RDY";

    // the standby draw for the power side, the standby entry for the
    // port - both compile-time facts of this state
    static constexpr auto annotations = fsm::annotate(
        standby_limit{v_safe_5v},
        enter_swap_standby{power_role::source}
    );
};

// PE_PRS_SNK_SRC_Assert_Rp: the old source is off - the port flips its
// termination now and continues as the new source
struct pe_snk_swap_assert_rp {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        assert_new_role{power_role::source}
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
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY, awaits the new source's";

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;

    pe_connection& connection;

private:
    pd_message message_{};
};

// The new source's PS_RDY arrived: the port completes the swap into
// Attached.SNK, then the normal sink flow resumes
struct pe_snk_swap_source_on_seen {
    using feature = pr_swap_feature;

    static constexpr power_level power = power_level::transition;
    static constexpr pd_status pd = pd_status::connected;
    static constexpr std::string_view dot_note = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(
        swap_completed{}
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
    static constexpr std::string_view dot_note = specNote(power, pd);
    static constexpr std::string_view dot_action = restore_default_action::note;

    // the restore for the power side, the window for the port: the
    // connection layer must hold the attach while VBUS cycles
    static constexpr auto annotations = fsm::annotate(
        restore_default_action{},
        hard_reset_window{}
    );

    using contexts = mtl::typelist<pe_negotiation>;

    pe_negotiation& negotiation;
};

// The states shared with the source engine (PolicyEngine.hpp), under
// the spec's sink-side names where it has them
using pe_snk_error_recovery = pe_error_recovery;
using pe_snk_give_sink_cap = pe_give_sink_cap;
using pe_snk_bist_carrier = pe_bist_carrier;
using pe_snk_send_not_supported = pe_send_not_supported<power_role::sink>;
using pe_snk_chunk_received = pe_chunk_received;
using pe_snk_send_dr_swap = pe_drs_send_swap<power_role::sink>;
using pe_snk_accept_dr_swap = pe_drs_accept_swap<power_role::sink>;
using pe_snk_dr_swap_change = pe_drs_change_data_role;
using pe_snk_vcs_send_swap = pe_vcs_send_swap<power_role::sink>;
using pe_snk_vcs_accept = pe_vcs_accept_swap<power_role::sink>;
using pe_snk_vcs_active = pe_vcs_active;
using pe_snk_vcs_send_ps_rdy = pe_vcs_send_ps_rdy<power_role::sink>;
using pe_snk_vcs_partner_on = pe_vcs_partner_on;
using pe_snk_vcs_ps_rdy_sent = pe_vcs_ps_rdy_sent;
using pe_snk_send_pr_swap = pe_prs_send_swap<power_role::sink>;
using pe_snk_accept_pr_swap = pe_prs_accept_swap<power_role::sink>;
using pe_snk_dr_swap_wait = pe_dr_swap_wait;
using pe_snk_pr_swap_wait = pe_pr_swap_wait;
using pe_snk_soft_reset = pe_soft_reset<power_role::sink>;
using pe_snk_send_soft_reset = pe_send_soft_reset<power_role::sink>;
using pe_snk_hard_reset = pe_hard_reset;

} // namespace state

// Which capability wait applies: SinkWaitCapTimer before the first
// hard reset, NoResponseTimer afterwards
struct no_hard_reset_yet {
    static bool check(state::pe_snk_discovery const& state)
    {
        return state.connection.hard_resets == 0;
    }
};

// The spec timer range of every timed state, checked against the table
using sink_timer_ranges = mtl::typelist<
    fsm::timed_by<state::pe_snk_wait_for_capabilities, spec::t_sink_wait_cap>,
    fsm::timed_by<state::pe_snk_select_capability, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_transition_sink, spec::t_ps_transition>,
    fsm::timed_by<state::pe_snk_chunk_received, spec::t_chunking_not_supported>,
    fsm::timed_by<state::pe_snk_send_soft_reset, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_send_dr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_send_pr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_snk_request_wait, spec::t_sink_request>,
    fsm::timed_by<state::pe_snk_dr_swap_wait, spec::t_dr_swap_wait>,
    fsm::timed_by<state::pe_snk_pr_swap_wait, spec::t_pr_swap_wait>,
    fsm::timed_by<state::pe_snk_wait_no_response, spec::t_no_response>,
    fsm::timed_by<state::pe_snk_bist_carrier, spec::t_bist_cont_mode>,
    fsm::timed_by<state::pe_snk_vcs_send_swap, spec::t_sender_response>>;

using sink_transitions = mtl::typelist<
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
    // DR_Swap: sent from Ready, or accepted there; both sides flip on
    // the agreement, an ignored request falls back to Ready
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_dr_swap>, fsm::to<state::pe_snk_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::accept>, fsm::to<state::pe_snk_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::reject>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::wait>, fsm::to<state::pe_snk_dr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_wait>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_dr_swap_gate>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_gate>, fsm::on<event::send_dr_swap>, fsm::to<state::pe_snk_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    // the partner's DR_Swap: the table asks the injected policy
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::dr_swap_received>, fsm::guard<dr_swap_allowed>, fsm::to<state::pe_snk_accept_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::dr_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_accept_dr_swap>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_snk_accept_dr_swap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_change>, fsm::on<event::swap_done>, fsm::to<state::pe_snk_ready>>,
    // VCONN_Swap: the messages anchor here, the vconn machine owns
    // the role, the switch, and the hand-off deadline
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_vconn_swap>, fsm::to<state::pe_snk_vcs_send_swap>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::accept>, fsm::to<state::pe_snk_vcs_active>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::reject>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::wait>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_received>, fsm::guard<vconn_swap_allowed>, fsm::to<state::pe_snk_vcs_accept>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_accept>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_vcs_active>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_accept>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::ps_rdy>, fsm::to<state::pe_snk_vcs_partner_on>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::send_vconn_ps_rdy>, fsm::to<state::pe_snk_vcs_send_ps_rdy>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::hard_reset_request>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_ps_rdy>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_vcs_ps_rdy_sent>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_ps_rdy>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_partner_on>, fsm::on<event::swap_done>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_ps_rdy_sent>, fsm::on<event::swap_done>, fsm::to<state::pe_snk_ready>>,
    // PR_Swap while sinking: the agreement leads into Transition_to_off,
    // the old source's PS_RDY into the termination flip
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_pr_swap>, fsm::to<state::pe_snk_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::accept>, fsm::to<state::pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::reject>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::wait>, fsm::to<state::pe_snk_pr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_wait>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_pr_swap_gate>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_gate>, fsm::on<event::send_pr_swap>, fsm::to<state::pe_snk_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<fsm::timeout>, fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::pr_swap_received>, fsm::guard<pr_swap_allowed>, fsm::to<state::pe_snk_accept_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::pr_swap_received>, fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_accept_pr_swap>, fsm::on<event::message_sent>, fsm::to<state::pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_snk_accept_pr_swap>, fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_swap_transition_to_off>, fsm::on<event::ps_rdy>, fsm::to<state::pe_snk_swap_assert_rp>>,
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
// measured
struct sink_table : mtl::rebind_t<sink_transitions, fsm::transition_table> {};

// The table checks (timeout bounds, reachability, both variants)
// live in test/compliance.cpp - one dedicated TU pays for them

// The sink's third power observer next to the shared contract_store
// and power_effects: the standby limit applied during the transition
// (PE_SNK_Transition_Sink draws iSnkStdby until PS_RDY)
template<typename POWER>
struct standby_driver : fsm::observing<standby_driver<POWER>> {
    explicit standby_driver(POWER& power_ref) : power(power_ref) {}

    void notifyEntry(standby_limit limit) { power.derived().setLimit(limit.voltage, i_snk_stdby); }

    POWER& power;
};

} // namespace pe

// The power side of the sink policy engine, injectable into it as one
// observer. Derive from it (CRTP) and implement the effects
// (concepts::sink_power_client):
//
//   bool setLimit(millivolt, milliamp);   // limit the sink load
//   void onContract(millivolt, milliamp); // explicit contract in place
//   void onContractLost();                // back to default power
//
// The spec-compliant sequencing lives here: iSnkStdby during the sink
// transition, the contract applied when the diagram's Power column
// changes to Explicit Contract, vSafe5V defaults restored on the
// states carrying a restore action - with onContractLost() fired only
// when a contract was actually in place
template<typename DERIVED>
class SinkPower : public fsm::ObserverGroup<
                      pe::contract_store<SinkPower<DERIVED>>,
                      pe::standby_driver<SinkPower<DERIVED>>,
                      pe::power_effects<SinkPower<DERIVED>>> {
public:
    // store before the effects: the contract terms must be fresh when
    // the power annotation edge fires on the same entry
    SinkPower()
        : fsm::ObserverGroup<
              pe::contract_store<SinkPower>,
              pe::standby_driver<SinkPower>,
              pe::power_effects<SinkPower>>(store_, standby_, effects_)
    {
    }

    // checked once the machine is built, when DERIVED is complete
    static constexpr void validateClient()
    {
        static_assert(
            concepts::sink_power_client<DERIVED>,
            "SinkPower: the derived class must provide setLimit(millivolt, "
            "milliamp), onContract(millivolt, milliamp), onContractLost()"
        );
    }

private:
    friend pe::contract_store<SinkPower>;
    friend pe::standby_driver<SinkPower>;
    friend pe::power_effects<SinkPower>;

    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    void applyContract()
    {
        contract_active_ = true;
        derived().setLimit(contract_.voltage, contract_.current);
        derived().onContract(contract_.voltage, contract_.current);
    }

    void restoreDefaults()
    {
        if (contract_active_) {
            contract_active_ = false;
            derived().setLimit(pe::v_safe_5v, pe::i_default_current);
            derived().onContractLost();
        }
    }

    pe::contract_store<SinkPower> store_{*this};
    pe::standby_driver<SinkPower> standby_{*this};
    pe::power_effects<SinkPower> effects_{*this};
    pe::active_contract contract_{};
    bool contract_active_ = false;
};

template<
    concepts::pd_transport TCPC,
    fsm::concepts::timer TIMER,
    concepts::sink_policy POLICY,
    typename... OBSERVERs>
class SinkPolicyEngine : public pe::PolicyEngineBase<
                             SinkPolicyEngine<TCPC, TIMER, POLICY, OBSERVERs...>,
                             power_role::sink,
                             TCPC,
                             TIMER,
                             POLICY> {
    using base = pe::PolicyEngineBase<SinkPolicyEngine, power_role::sink, TCPC, TIMER, POLICY>;
    friend base;

public:
    // The observers are injected into the engine's machine after the
    // protocol layer; a SinkPower-derived one supplies the power side
    SinkPolicyEngine(
        TCPC& tcpc,
        TIMER& prl_timer,
        TIMER& pe_timer,
        std::span<sink_capability const> capabilities,
        POLICY& policy,
        OBSERVERs&... observers
    )
        : base(tcpc, prl_timer, pe_timer), policy_(policy),
          sm_(this->timed_, this->prl_, gates_, policy_, observers...)
    {
        this->sink_capabilities_ = capabilities;
        sm_.process(pe::event::started{}); // rest in Discovery until VBUS
    }

    // The Type-C layer reports attach: for a sink, VBUS is present
    void vbusPresent() { sm_.process(pe::event::vbus_present{}); }

    // ... and detach: negotiation state is gone, back to Discovery;
    // the next partner negotiates its own revision
    void vbusRemoved()
    {
        this->prl_.resetRevision();
        this->setBistTestData(false); // the test mode ends with the partner
        pending_sequence_.reset();
        sm_.process(pe::event::vbus_removed{});
        sm_.process(pe::event::started{});
    }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there). Under PD3
    // collision avoidance a request during SinkTxNG parks and fires
    // when the source's Rp says SinkTxOk
    bool requestPowerSwap()
    {
        if constexpr (!base::pr_swap_capable) {
            return false; // the feature is compiled out
        } else {
            return request(pe::atomic_message_sequence::power_role_swap);
        }
    }

    bool requestDataSwap()
    {
        if constexpr (!base::dr_swap_capable) {
            return false; // the feature is compiled out
        } else {
            return request(pe::atomic_message_sequence::data_role_swap);
        }
    }

    // PD3 collision avoidance: the source's Rp signals whether the
    // sink may initiate an AMS (SinkTxOk = 3.0 A, SinkTxNG = 1.5 A).
    // The port layer feeds every CC report; parked and gated requests
    // fire on the flip to Ok
    void sinkTxChanged(bool ok)
    {
        sink_tx_ok_ = ok;
        if (ok) {
            fireGated();
            firePending();
        }
    }

    // The port was the source and asserted Rd mid PR_Swap: announce
    // the supply is off and await the new source's PS_RDY. The event
    // carries the preserved data role (the entered state seeds the
    // context with it); the negotiated revision and the MessageID
    // lifecycle hold for the connection - a swap is no reset trigger
    // (6.7.1) - and are handed over from the retiring engine
    void
    startSwapWaitSourceOn(data_role role, pd_revision revision, prl::message_id_state const& ids)
    {
        this->prl_.seedRevision(revision);
        this->prl_.seedMessageIds(ids);
        sm_.process(pe::event::swap_wait_source_on{role});
    }

    // A DRP announces its source-role capabilities: Get_Source_Cap is
    // answered with them instead of Not_Supported
    void provideSourceCapabilities(std::span<std::uint32_t const> capabilities)
    {
        this->source_capabilities_ = capabilities;
    }

    // The swap completed into Attached.SNK: resume the sink flow (the
    // new source's PS_RDY implies VBUS is live)
    void finishSwap()
    {
        sm_.process(pe::event::swap_done{});
        sm_.process(pe::event::vbus_present{});
    }

private:
    // A retry whose gate opens while SinkTxOk already holds fires
    // right away: the retry_gated annotation marks the gate states,
    // and the queued machine delivers the retry after the gate's
    // entry completes (the parked case fires from sinkTxChanged)
    struct gate_watch : fsm::observing<gate_watch> {
        explicit gate_watch(SinkPolicyEngine& engine_ref) : engine(engine_ref) {}

        void notifyEntry(pe::retry_gated)
        {
            if (engine.sinkTxAllows()) {
                engine.fireGated();
            }
        }

        SinkPolicyEngine& engine;
    };

    // --- the base's hooks ---------------------------------------------------

    // The sink's own messages: Source_Capabilities to evaluate, and
    // Get_Source_Cap answered by a DRP with its source-role list
    bool dispatchRole(pd_header const& header, pd_message const& message)
    {
        if (isData(header, data_message_type::source_capabilities)) {
            evaluate(message, header.num_data_objects);
            return true;
        }
        if (isControl(header, control_message_type::get_source_cap)) {
            sendSourceCapabilities();
            return true;
        }
        return false;
    }

    // the contract's voltage while an explicit contract holds (BIST is
    // honored at vSafe5V only)
    std::optional<millivolt> contractVoltage() const
    {
        auto const& negotiation = sm_.template context<pe::pe_negotiation>();
        if (!negotiation.explicit_contract) {
            return std::nullopt;
        }
        return negotiation.request.voltage;
    }

    // Advances the transient spec states after a hard reset:
    // Transition_to_default -> Startup -> Discovery, where the engine
    // waits for the Type-C layer to report the returning VBUS
    void afterHardReset()
    {
        sm_.process(pe::event::default_level_reached{});
        sm_.process(pe::event::started{});
    }

    // --- collision avoidance --------------------------------------------------

    // A swap request from Ready: sent now, or parked until SinkTxOk
    bool request(pe::atomic_message_sequence sequence)
    {
        if (!sm_.template annotation<pe::ready_for_atomic_message_sequence>()) {
            return false;
        }
        if (!sinkTxAllows()) {
            pending_sequence_ = sequence;
            return true;
        }
        initiate(sequence);
        return true;
    }

    // Collision avoidance applies under PD3 with an explicit contract;
    // otherwise the sink initiates freely
    bool sinkTxAllows() const
    {
        if (this->prl_.revision() != pd_revision::rev_3_x) {
            return true;
        }
        return !sm_.template context<pe::pe_negotiation>().explicit_contract || sink_tx_ok_;
    }

    // A retry that timed out under SinkTxNG waits in its gate state,
    // whose annotation says which sequence (a gate of a compiled-out
    // feature never exists, its event is refused at compile time)
    void fireGated()
    {
        if (auto const gated = sm_.template annotation<pe::retry_gated>()) {
            initiate(gated->sequence);
        }
    }

    // A swap request parked under SinkTxNG fires once Ready under Ok
    void firePending()
    {
        auto const pending = pending_sequence_;
        pending_sequence_.reset();
        if (pending && sm_.template annotation<pe::ready_for_atomic_message_sequence>()) {
            initiate(*pending);
        }
    }

    void initiate(pe::atomic_message_sequence sequence)
    {
        switch (sequence) {
        case pe::atomic_message_sequence::request: sm_.process(pe::event::request_retry{}); break;
        case pe::atomic_message_sequence::power_role_swap:
            sm_.process(pe::event::send_pr_swap{});
            break;
        case pe::atomic_message_sequence::data_role_swap:
            sm_.process(pe::event::send_dr_swap{});
            break;
        }
    }

    // --- the sink's messages ------------------------------------------------

    // PE_SNK_Evaluate_Capability: ask the policy, fall back to the
    // first PDO with the Capability Mismatch flag
    void evaluate(pd_message const& message, std::uint8_t count)
    {
        if (!sm_.process(pe::event::source_capabilities{})) {
            return; // not in a state that evaluates capabilities
        }
        std::array<std::uint32_t, 7> objects{};
        auto const n = std::min<std::uint8_t>(count, objects.size());
        for (std::uint8_t index = 0; index < n; ++index) {
            objects[index] = dataObjectAt(message, index);
        }
        auto const offered = std::span<std::uint32_t const>{objects.data(), n};

        auto terms = policy_.select(offered, this->sink_capabilities_);
        if (!terms && n > 0) {
            terms = contract_request{
                .position = 1,
                .voltage = pdo::fixedVoltage(objects[0]),
                .operating_current = pdo::fixedMaxCurrent(objects[0]),
                .maximum_current = pdo::fixedMaxCurrent(objects[0]),
                .mismatch = true
            };
        }
        if (terms) {
            sm_.process(pe::event::capabilities_evaluated{.terms = *terms});
        }
    }

    // PE_DR_SNK_Give_Source_Cap; a sink-only port answers Not_Supported
    void sendSourceCapabilities()
    {
        if (this->source_capabilities_.empty()) {
            sm_.process(pe::event::unsupported{});
            return;
        }
        sm_.process(
            pe::event::send_source_caps{pe::makeSourceCapabilitiesMessage(
                power_role::sink,
                this->dataRole(),
                this->source_capabilities_
            )}
        );
    }

    POLICY& policy_;
    bool sink_tx_ok_ = true;                                      // last Rp seen (SinkTxOk/NG)
    std::optional<pe::atomic_message_sequence> pending_sequence_; // parked under SinkTxNG
    gate_watch gates_{*this};
    // the policy rides in the pack to answer the table's questions
    fsm::QueuedMachine<
        pe::sink_table,
        4,
        fsm::inline_work,
        fsm::no_lock,
        fsm::timed<fsm::QueuedTimer<TIMER>&>,
        ProtocolLayer<TCPC, TIMER>,
        gate_watch,
        POLICY,
        OBSERVERs...>
        sm_;
};

} // namespace usbc
