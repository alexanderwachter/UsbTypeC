/*
 * USB PD source policy engine (PE_SRC) on top of the protocol layer.
 *
 * The application injects the source's capabilities as a span of raw
 * fixed-supply PDOs (the Source_Capabilities content), a policy that
 * evaluates a sink's Request against them (RequestPolicy below is the
 * default), and the source_supply that delivers the contract. The
 * engine advertises: Send_Capabilities repeats through Discovery
 * (SourceCapabilityTimer) up to nCapsCount times, then rests in
 * Disabled for a PD-incapable sink. A Request is evaluated through the
 * policy: Accept -> tSrcTransition wait -> supply setOutput() -> the
 * settled callback -> PS_RDY -> Ready (PE_SRC_Capability_Response
 * sends the Reject otherwise). Protocol errors escalate through
 * Send_Soft_Reset to Hard_Reset; Transition_to_default restores the
 * supply to vSafe5V and waits tSrcRecover before advertising again.
 *
 * Unsupported messages are answered with Not_Supported from Ready;
 * chunked extended messages run into ChunkingNotSupportedTimer first
 * (non-chunking device), exactly like the sink engine.
 *
 * The machine follows the spec's PE_SRC state diagram with these
 * deviations kept for later: SenderResponseTimer runs from the
 * Send_Capabilities/Transition entry (it includes the transmission
 * instead of starting at the GoodCRC), PE_SRC_Hard_Reset_Received is
 * folded into the any_state edge to Transition_to_default, Wait and
 * GotoMin are not sent, lost regulation (at_target = false) is not yet
 * handled, and the no-response escalation is counter-driven (each
 * SenderResponse timeout under a PD contract hard-resets; after
 * nHardResetCount the engine requests Error Recovery) rather than
 * paced by a literal NoResponseTimer. The supply-transition
 * choreography of PE_SRC_Transition_Supply is spelled out as _delay,
 * _settle and _ps_rdy sub-states so the diagram shows it; the
 * hard-reset recovery likewise (_transition_to_default, _recover,
 * _restore_default).
 *
 * Integration: mirror image of the sink engine. The ProtocolLayer is
 * an observer of the machine (the states' prl::reset_action
 * annotations, the pd_message they publish through values()); the
 * application injects its own observers - derive
 * from SourcePower (CRTP) and implement onContract/onContractLost.
 * The source_supply is engine-owned like the TCPC: states report the
 * supply target, the engine applies it and feeds the settled callback
 * back as an event. Feed TCPC alerts into onAlert() and the Type-C
 * source layer's attach/detach into attached() and detached().
 * Everything runs in the stack's serialized context; observer
 * callbacks may originate from the timer context (mtl timer contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Pdo.hpp>
#include <usbc/PolicyEngine.hpp>
#include <usbc/ProtocolLayer.hpp>
#include <usbc/SourceSupply.hpp>
#include <usbc/Tcpc.hpp>
#include <usbc/Units.hpp>

#include <mtl/StateMachine.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

namespace usbc {

// A policy's answer: the operating point the supply must deliver
struct supply_target {
    millivolt voltage = pe::v_safe_5v;
    milliamp current  = pe::i_default_current;
    constexpr bool operator==(supply_target const&) const = default;
};

namespace concepts {

template<typename T>
concept source_policy = requires(T policy, std::uint32_t rdo,
                                 std::span<std::uint32_t const> capabilities) {
    { policy.evaluate(rdo, capabilities) } -> std::same_as<std::optional<supply_target>>;
};

// The interface a SourcePower-derived class provides
template<typename T>
concept source_power_client = requires(T client, millivolt voltage, milliamp current) {
    client.onContract(voltage, current);
    client.onContractLost();
};

} // namespace concepts

// Default evaluation policy: a Request is granted when it names a
// fixed-supply PDO of the capabilities and its operating current stays
// within that PDO's maximum
class RequestPolicy {
public:
    constexpr std::optional<supply_target> evaluate(
        std::uint32_t rdo, std::span<std::uint32_t const> capabilities) const
    {
        auto const position = pdo::requestPosition(rdo);
        if (position == 0 || position > capabilities.size()) {
            return std::nullopt;
        }
        auto const object = capabilities[position - 1u];
        if (pdo::kindOf(object) != pdo::kind::fixed_supply) {
            return std::nullopt;
        }
        auto const current = pdo::requestOperatingCurrent(rdo);
        if (current > pdo::fixedMaxCurrent(object)) {
            return std::nullopt;
        }
        return supply_target{.voltage = pdo::fixedVoltage(object), .current = current};
    }
};
static_assert(concepts::source_policy<RequestPolicy>);

namespace pe {

inline constexpr auto t_typec_send_source_cap = std::chrono::milliseconds{150}; // tTypeCSendSourceCap
inline constexpr auto t_src_transition        = std::chrono::milliseconds{30};  // tSrcTransition
inline constexpr auto t_src_recover           = std::chrono::milliseconds{800}; // tSrcRecover
inline constexpr auto t_source_start          = std::chrono::milliseconds{30};  // tSwapSourceStart
inline constexpr auto t_sink_tx               = std::chrono::milliseconds{18};  // tSinkTx

// PD3 collision avoidance, signalled through the source's Rp: states
// carrying the annotation drive it (SinkTxOk = 3.0 A, SinkTxNG = 1.5 A)
enum class sink_tx : std::uint8_t { ok, ng };

inline constexpr std::uint8_t n_caps_count = spec::n_caps_count;

// Engine-directed command: entering a state carrying it in its
// annotation set makes the engine transmit the Source_Capabilities
struct send_capabilities_action {
    static constexpr std::string_view note = "sends Source_Capabilities";
    constexpr bool operator==(send_capabilities_action const&) const = default;
};

// Machine-owned context, the source's negotiation: one negotiation's
// worth, gone with every reset within the connection (the connection
// itself is the shared pe_connection)
struct src_negotiation {
    std::uint8_t caps_counter = 0;     // CapsCounter
    bool pd_connected         = false; // a Source_Capabilities got its GoodCRC
    bool explicit_contract    = false;
    supply_target target{};
};

namespace event {

struct attached {};
struct detached {};
struct request {};
struct request_ok { // the policy granted; the state builds the Accept
    supply_target target;
};
struct request_bad {}; // the policy refused; the state builds the Reject
struct get_source_caps {};
struct begin_pr_swap {}; // PD3: SinkTxNG + tSinkTx precede the request
struct begin_dr_swap {};
struct supply_settled {};

} // namespace event

namespace state {

// PE_SRC_Startup: the protocol layer reset is mandatory; the default
// power restore covers the detach entry (after a hard reset,
// Transition_to_default already restored and suppression elides it)
struct pe_src_startup {
    static constexpr auto annotations =
        fsm::annotate(prl::reset_action{}, restore_default_action{});
    static constexpr power_level power           = power_level::default_power;
    static constexpr pd_status pd                = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    // detach: everything forgotten (the next partner meets a DFP)
    explicit pe_src_startup(pe_connection& connection, src_negotiation& negotiation)
        : connection(connection), negotiation(negotiation)
    {
        this->connection  = {.data = defaultDataRole(power_role::source)};
        this->negotiation = {};
    }
    using contexts = mtl::typelist<pe_connection, src_negotiation>;
    pe_connection& connection;
    src_negotiation& negotiation;
};

// Advertises the capabilities and waits for the Request; the timer
// includes the transmission (deviation, see the file comment)
struct pe_src_send_capabilities {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr auto annotations            = fsm::annotate(send_capabilities_action{});
    static constexpr power_level power           = power_level::default_power;
    static constexpr pd_status pd                = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = send_capabilities_action::note;

    pe_src_send_capabilities(event::attached const&, pe_connection& connection,
                             src_negotiation& negotiation)
        : pe_src_send_capabilities(connection, negotiation)
    {
        connection.attached = true;
    }
    explicit pe_src_send_capabilities(pe_connection& connection, src_negotiation& negotiation)
        : connection(connection), negotiation(negotiation)
    {
        ++negotiation.caps_counter;
    }

    // the GoodCRC on the capabilities means a PD sink is present
    void handle(pe::event::message_sent const&) { negotiation.pd_connected = true; }

    using contexts = mtl::typelist<pe_connection, src_negotiation>;
    pe_connection& connection;
    src_negotiation& negotiation;
};

// Waits SourceCapabilityTimer between advertisement attempts
struct pe_src_discovery {
    static constexpr auto timeout = t_typec_send_source_cap; // SourceCapabilityTimer
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_discovery(src_negotiation& negotiation) : negotiation(negotiation) {}
    using contexts = mtl::typelist<src_negotiation>; // the guard reads CapsCounter
    src_negotiation& negotiation;
};

// nCapsCount advertisements went unanswered: the sink speaks no PD,
// the port stays a plain Type-C source until detach
struct pe_src_disabled {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

// The engine evaluates the Request through the injected policy and
// advances with request_ok or request_bad
struct pe_src_negotiate_capability {
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_negotiate_capability(pe_connection& connection) : connection(connection)
    {
        connection.hard_resets = 0; // spec: the sink responded
    }
    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;
};

// PE_SRC_Transition_Supply: sends the Accept; the _delay, _settle and
// _ps_rdy sub-states spell out the supply choreography
struct pe_src_transition_supply {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_transition_supply(event::request_ok const& event, pe_connection& connection_ref,
                             src_negotiation& negotiation_ref)
        : pe_src_transition_supply(connection_ref, negotiation_ref)
    {
        negotiation.target = event.target;
    }
    pe_src_transition_supply(pe_connection& connection_ref, src_negotiation& negotiation_ref)
        : connection(connection_ref), negotiation(negotiation_ref),
          message_(makeControlMessage(control_message_type::accept, power_role::source,
                                      connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection, src_negotiation>;
    pe_connection& connection;
    src_negotiation& negotiation;

private:
    pd_message message_;
};

// The spec's tSrcTransition wait between the Accept and the change
struct pe_src_transition_supply_delay {
    static constexpr auto timeout = t_src_transition; // tSrcTransition
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

// Commands the supply to the new operating point and waits for the
// settled callback
struct pe_src_transition_supply_settle {
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "programs the supply";

    explicit pe_src_transition_supply_settle(src_negotiation& negotiation)
        : negotiation(negotiation)
    {
    }

    supply_target values() const { return negotiation.target; }

    using contexts = mtl::typelist<src_negotiation>;
    src_negotiation& negotiation;
};

// The supply is at the target: PS_RDY tells the sink to draw
struct pe_src_transition_supply_ps_rdy {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_transition_supply_ps_rdy(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::ps_rdy, power_role::source,
                                      connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

struct pe_src_ready {
    static constexpr power_level power          = power_level::explicit_contract;
    // the sink may initiate (SinkTxOk)
    static constexpr auto annotations =
        fsm::annotate(power, sink_tx::ok, ready_for_atomic_message_sequence{});
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_ready(src_negotiation& negotiation) : negotiation(negotiation)
    {
        negotiation.explicit_contract = true;
        negotiation.pd_connected      = true;
    }

    active_contract values() const
    {
        return {negotiation.target.voltage, negotiation.target.current};
    }

    using contexts = mtl::typelist<src_negotiation>;
    src_negotiation& negotiation;
};

// PE_SRC_Capability_Response: the policy refused, the Reject goes out
struct pe_src_capability_response {
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_capability_response(event::request_bad const&, pe_connection& connection_ref,
                               src_negotiation& negotiation_ref)
        : pe_src_capability_response(connection_ref, negotiation_ref)
    {
    }
    pe_src_capability_response(pe_connection& connection_ref, src_negotiation& negotiation_ref)
        : connection(connection_ref), negotiation(negotiation_ref),
          message_(makeControlMessage(control_message_type::reject, power_role::source,
                                      connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection, src_negotiation>;
    pe_connection& connection;
    src_negotiation& negotiation;

private:
    pd_message message_;
};

// --- PR_Swap, source side (PE_PRS_SRC_SNK_*) ----------------------------------

// PD3 collision avoidance ahead of a source-initiated AMS: SinkTxNG
// goes on the wire (the Rp annotation), the first message follows
// after tSinkTx
struct pe_src_sink_tx_wait_pr {
    using feature = pr_swap_feature;
    static constexpr auto timeout = t_sink_tx; // tSinkTx
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr auto annotations           = fsm::annotate(power, sink_tx::ng);
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

struct pe_src_sink_tx_wait_dr {
    using feature = dr_swap_feature;
    static constexpr auto timeout = t_sink_tx; // tSinkTx
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr auto annotations           = fsm::annotate(power, sink_tx::ng);
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

// PE_PRS_SRC_SNK_Transition_to_off, the spec's tSrcTransition wait
// between the agreement and removing power
struct pe_src_swap_transition_to_off {
    using feature = pr_swap_feature;
    static constexpr auto timeout = t_src_transition; // tSrcTransition
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

// ... the supply is commanded off and its settled report awaited
struct pe_src_swap_supply_off {
    using feature = pr_swap_feature;
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "turns the supply off";

    static constexpr auto annotations = fsm::annotate(supply_target{.voltage = 0, .current = 0});
};

// PE_PRS_SRC_SNK_Assert_Rd: VBUS is off - the port flips its
// termination now; the sink engine then announces our PS_RDY
struct pe_src_swap_assert_rd {
    using feature = pr_swap_feature;
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    static constexpr auto annotations = fsm::annotate(assert_new_role{power_role::sink});
};

// PE_PRS_SNK_SRC_Source_on, this engine's half: the port was the sink
// and asserted Rp - VBUS is driven to vSafe5V first
struct pe_src_swap_source_on {
    using feature = pr_swap_feature;
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "drives VBUS to vSafe5V";

    pe_src_swap_source_on(pe::event::attached_swap const& event, pe_connection& connection,
                          src_negotiation& negotiation)
        : connection(connection), negotiation(negotiation)
    {
        connection.attached       = true;
        connection.data           = event.role; // a power swap preserves the data role
        negotiation.pd_connected  = true;       // the swap was PD-negotiated
    }
    explicit pe_src_swap_source_on(pe_connection& connection, src_negotiation& negotiation)
        : connection(connection), negotiation(negotiation)
    {
    }

    static constexpr auto annotations =
        fsm::annotate(supply_target{.voltage = v_safe_5v, .current = i_default_current});

    using contexts = mtl::typelist<pe_connection, src_negotiation>;
    pe_connection& connection;
    src_negotiation& negotiation;
};

// ... at vSafe5V the PS_RDY completes the partner's wait
struct pe_src_swap_source_on_ps_rdy {
    using feature = pr_swap_feature;
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY";

    explicit pe_src_swap_source_on_ps_rdy(pe_connection& connection_ref)
        : connection(connection_ref),
          message_(makeControlMessage(control_message_type::ps_rdy, power_role::source,
                                      connection.data))
    {
    }

    pd_message const& values() const { return message_; }

    using contexts = mtl::typelist<pe_connection>;
    pe_connection& connection;

private:
    pd_message message_;
};

// SwapSourceStartTimer: the new source pauses before its first
// Source_Capabilities
struct pe_src_swap_source_start {
    using feature = pr_swap_feature;
    static constexpr auto timeout = t_source_start; // SwapSourceStartTimer
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_swap_source_start(src_negotiation& negotiation) : negotiation(negotiation)
    {
        negotiation.caps_counter = 0; // the advertisement starts over
    }

    using contexts = mtl::typelist<src_negotiation>;
    src_negotiation& negotiation;
};

// PE_SRC_Transition_to_default, spec-shaped: VBUS is removed first
// (supply to vSafe0V, settled awaited), tSrcRecover passes in
// pe_src_recover, then pe_src_restore_default re-applies vSafe5V and
// advertises once the supply settled (or rests in Startup after a
// detach)
struct pe_src_transition_to_default {
    // the protocol reset, and VBUS removed - both compile-time facts
    static constexpr auto annotations =
        fsm::annotate(prl::reset_action{}, supply_target{.voltage = 0, .current = 0});
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, removes VBUS";

    // the connection persists, the negotiation ends
    explicit pe_src_transition_to_default(src_negotiation& negotiation) : negotiation(negotiation)
    {
        this->negotiation = {};
    }

    using contexts = mtl::typelist<src_negotiation>;
    src_negotiation& negotiation;
};

// tSrcRecover at vSafe0V before the defaults return
struct pe_src_recover {
    static constexpr auto timeout = t_src_recover; // tSrcRecover
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
};

// vSafe5V defaults restored (the restore action also reports the
// contract lost); the settled supply resumes the advertisement
struct pe_src_restore_default {
    static constexpr auto annotations            = fsm::annotate(restore_default_action{});
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = restore_default_action::note;

    explicit pe_src_restore_default(pe_connection& connection) : connection(connection) {}
    using contexts = mtl::typelist<pe_connection>; // the guard asks whether still attached
    pe_connection& connection;
};

// The states shared with the sink engine (PolicyEngine.hpp), under
// the spec's source-side names where it has them
using pe_src_error_recovery     = pe_error_recovery;
using pe_dr_src_give_sink_cap   = pe_give_sink_cap;
using pe_src_bist_carrier       = pe_bist_carrier;
using pe_src_send_not_supported = pe_send_not_supported<power_role::source>;
using pe_src_chunk_received     = pe_chunk_received;
using pe_src_send_dr_swap       = pe_drs_send_swap<power_role::source>;
using pe_src_accept_dr_swap     = pe_drs_accept_swap<power_role::source>;
using pe_src_dr_swap_change     = pe_drs_change_data_role;
using pe_src_vcs_send_swap      = pe_vcs_send_swap<power_role::source>;
using pe_src_vcs_accept         = pe_vcs_accept_swap<power_role::source>;
using pe_src_vcs_active         = pe_vcs_active;
using pe_src_vcs_send_ps_rdy    = pe_vcs_send_ps_rdy<power_role::source>;
using pe_src_vcs_partner_on     = pe_vcs_partner_on;
using pe_src_vcs_ps_rdy_sent    = pe_vcs_ps_rdy_sent;
using pe_src_send_pr_swap       = pe_prs_send_swap<power_role::source>;
using pe_src_accept_pr_swap     = pe_prs_accept_swap<power_role::source>;
using pe_src_dr_swap_wait       = pe_dr_swap_wait;
using pe_src_pr_swap_wait       = pe_pr_swap_wait;
using pe_src_soft_reset         = pe_soft_reset<power_role::source>;
using pe_src_send_soft_reset    = pe_send_soft_reset<power_role::source>;
using pe_src_hard_reset         = pe_hard_reset;

} // namespace state

struct caps_count_allows {
    static bool check(state::pe_src_discovery const& state)
    {
        return state.negotiation.caps_counter <= n_caps_count;
    }
};

struct pd_was_connected {
    static bool check(state::pe_src_send_capabilities const& state)
    {
        return state.negotiation.pd_connected;
    }
};

struct still_attached {
    static bool check(state::pe_src_restore_default const& state)
    {
        return state.connection.attached;
    }
};

// The spec timer range of every timed state, checked against the table
using source_timer_ranges = mtl::typelist<
    fsm::timed_by<state::pe_src_send_capabilities, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_discovery, spec::t_typec_send_source_cap>,
    fsm::timed_by<state::pe_src_transition_supply_delay, spec::t_src_transition>,
    fsm::timed_by<state::pe_src_chunk_received, spec::t_chunking_not_supported>,
    fsm::timed_by<state::pe_src_send_soft_reset, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_recover, spec::t_src_recover>,
    fsm::timed_by<state::pe_src_send_dr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_send_pr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_swap_transition_to_off, spec::t_src_transition>,
    fsm::timed_by<state::pe_src_swap_source_start, spec::t_swap_source_start>,
    fsm::timed_by<state::pe_src_dr_swap_wait, spec::t_dr_swap_wait>,
    fsm::timed_by<state::pe_src_pr_swap_wait, spec::t_pr_swap_wait>,
    fsm::timed_by<state::pe_src_bist_carrier, spec::t_bist_cont_mode>,
    fsm::timed_by<state::pe_src_sink_tx_wait_pr, spec::t_sink_tx>,
    fsm::timed_by<state::pe_src_sink_tx_wait_dr, spec::t_sink_tx>,
    fsm::timed_by<state::pe_src_vcs_send_swap, spec::t_sender_response>>;

using source_transitions = mtl::typelist<
    fsm::initial<state::pe_src_startup>,
    fsm::transition<fsm::from<state::pe_src_startup>, fsm::on<event::attached>,
                    fsm::to<state::pe_src_send_capabilities>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::detached>,
                    fsm::to<state::pe_src_startup>>,
    // capabilities out; GoodCRC marks the sink PD-capable (internal)
    fsm::internal_transition<fsm::from<state::pe_src_send_capabilities>,
                             fsm::on<pe::event::message_sent>>,
    fsm::transition<fsm::from<state::pe_src_send_capabilities>, fsm::on<event::request>,
                    fsm::to<state::pe_src_negotiate_capability>>,
    // no Request from a PD-capable sink: hard reset while the counter
    // allows, then Error Recovery; a PD-incapable one goes to Discovery
    fsm::transition<fsm::from<state::pe_src_send_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_hard_reset>,
                    fsm::guard<pd_was_connected, hard_resets_left>>,
    fsm::transition<fsm::from<state::pe_src_send_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_error_recovery>, fsm::guard<pd_was_connected>>,
    fsm::transition<fsm::from<state::pe_src_send_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_discovery>>,
    fsm::transition<fsm::from<state::pe_src_send_capabilities>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_send_soft_reset>,
                    fsm::guard<pd_was_connected>>,
    fsm::transition<fsm::from<state::pe_src_send_capabilities>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_discovery>>,
    // Discovery retries the advertisement up to nCapsCount times
    fsm::transition<fsm::from<state::pe_src_discovery>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_capabilities>, fsm::guard<caps_count_allows>>,
    fsm::transition<fsm::from<state::pe_src_discovery>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_disabled>>,
    // negotiation: the engine injects the policy's verdict
    fsm::transition<fsm::from<state::pe_src_negotiate_capability>, fsm::on<event::request_ok>,
                    fsm::to<state::pe_src_transition_supply>>,
    fsm::transition<fsm::from<state::pe_src_negotiate_capability>, fsm::on<event::request_bad>,
                    fsm::to<state::pe_src_capability_response>>,
    // supply transition: Accept -> tSrcTransition -> settle -> PS_RDY
    fsm::transition<fsm::from<state::pe_src_transition_supply>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_transition_supply_delay>>,
    fsm::transition<fsm::from<state::pe_src_transition_supply>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_transition_supply_delay>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_transition_supply_settle>>,
    fsm::transition<fsm::from<state::pe_src_transition_supply_settle>,
                    fsm::on<event::supply_settled>,
                    fsm::to<state::pe_src_transition_supply_ps_rdy>>,
    fsm::transition<fsm::from<state::pe_src_transition_supply_ps_rdy>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_transition_supply_ps_rdy>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_hard_reset>>,
    // the Reject: back to Ready under a contract, hard reset without
    fsm::transition<fsm::from<state::pe_src_capability_response>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_ready>,
                    fsm::guard<explicit_contract_holds>>,
    fsm::transition<fsm::from<state::pe_src_capability_response>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_capability_response>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_hard_reset>>,
    // Ready serves the sink
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::request>,
                    fsm::to<state::pe_src_negotiate_capability>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::get_source_caps>,
                    fsm::to<state::pe_src_send_capabilities>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_sink_capabilities>,
                    fsm::to<state::pe_dr_src_give_sink_cap>>,
    fsm::transition<fsm::from<state::pe_dr_src_give_sink_cap>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_dr_src_give_sink_cap>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::unsupported>,
                    fsm::to<state::pe_src_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_src_send_not_supported>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_send_not_supported>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::bist_carrier>,
                    fsm::to<state::pe_src_bist_carrier>>,
    fsm::transition<fsm::from<state::pe_src_bist_carrier>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::chunked_message>,
                    fsm::to<state::pe_src_chunk_received>>,
    fsm::transition<fsm::from<state::pe_src_chunk_received>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_not_supported>>,
    // DR_Swap: sent from Ready, or accepted there; both sides flip on
    // the agreement, an ignored request falls back to Ready
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_dr_swap>,
                    fsm::to<state::pe_src_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::begin_dr_swap>,
                    fsm::to<state::pe_src_sink_tx_wait_dr>>,
    fsm::transition<fsm::from<state::pe_src_sink_tx_wait_dr>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_src_send_dr_swap>, fsm::on<pe::event::accept>,
                    fsm::to<state::pe_src_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_src_send_dr_swap>, fsm::on<pe::event::reject>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_send_dr_swap>, fsm::on<pe::event::wait>,
                    fsm::to<state::pe_src_dr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_src_dr_swap_wait>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_src_send_dr_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_send_dr_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    // the partner's DR_Swap: the table asks the injected policy
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::dr_swap_received>,
                    fsm::to<state::pe_src_accept_dr_swap>, fsm::guard<pe::dr_swap_allowed>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::dr_swap_received>,
                    fsm::to<state::pe_src_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_src_accept_dr_swap>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_src_accept_dr_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_dr_swap_change>, fsm::on<pe::event::swap_done>,
                    fsm::to<state::pe_src_ready>>,
    // VCONN_Swap: the messages anchor here, the vconn machine owns
    // the role, the switch, and the hand-off deadline
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_vconn_swap>,
                    fsm::to<state::pe_src_vcs_send_swap>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_swap>, fsm::on<pe::event::accept>,
                    fsm::to<state::pe_src_vcs_active>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_swap>, fsm::on<pe::event::reject>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_swap>, fsm::on<pe::event::wait>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::vconn_swap_received>,
                    fsm::to<state::pe_src_vcs_accept>, fsm::guard<pe::vconn_swap_allowed>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::vconn_swap_received>,
                    fsm::to<state::pe_src_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_src_vcs_accept>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_vcs_active>>,
    fsm::transition<fsm::from<state::pe_src_vcs_accept>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_vcs_active>, fsm::on<pe::event::ps_rdy>,
                    fsm::to<state::pe_src_vcs_partner_on>>,
    fsm::transition<fsm::from<state::pe_src_vcs_active>, fsm::on<pe::event::send_vconn_ps_rdy>,
                    fsm::to<state::pe_src_vcs_send_ps_rdy>>,
    fsm::transition<fsm::from<state::pe_src_vcs_active>, fsm::on<pe::event::hard_reset_request>,
                    fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_ps_rdy>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_vcs_ps_rdy_sent>>,
    fsm::transition<fsm::from<state::pe_src_vcs_send_ps_rdy>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_vcs_partner_on>, fsm::on<pe::event::swap_done>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_vcs_ps_rdy_sent>, fsm::on<pe::event::swap_done>,
                    fsm::to<state::pe_src_ready>>,
    // PR_Swap while sourcing: the agreement leads through tSrcTransition
    // into the supply-off wait, then the termination flip
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_pr_swap>,
                    fsm::to<state::pe_src_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::begin_pr_swap>,
                    fsm::to<state::pe_src_sink_tx_wait_pr>>,
    fsm::transition<fsm::from<state::pe_src_sink_tx_wait_pr>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_src_send_pr_swap>, fsm::on<pe::event::accept>,
                    fsm::to<state::pe_src_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_src_send_pr_swap>, fsm::on<pe::event::reject>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_send_pr_swap>, fsm::on<pe::event::wait>,
                    fsm::to<state::pe_src_pr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_src_pr_swap_wait>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_src_send_pr_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_ready>>,
    fsm::transition<fsm::from<state::pe_src_send_pr_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::pr_swap_received>,
                    fsm::to<state::pe_src_accept_pr_swap>, fsm::guard<pe::pr_swap_allowed>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::pr_swap_received>,
                    fsm::to<state::pe_src_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_src_accept_pr_swap>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_src_accept_pr_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_swap_transition_to_off>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_swap_supply_off>>,
    fsm::transition<fsm::from<state::pe_src_swap_supply_off>, fsm::on<event::supply_settled>,
                    fsm::to<state::pe_src_swap_assert_rd>>,
    // PR_Swap's other half: this port was the sink and asserted Rp -
    // VBUS on, PS_RDY out, a pause, then the capabilities
    fsm::transition<fsm::from<state::pe_src_startup>, fsm::on<pe::event::attached_swap>,
                    fsm::to<state::pe_src_swap_source_on>>,
    fsm::transition<fsm::from<state::pe_src_swap_source_on>, fsm::on<event::supply_settled>,
                    fsm::to<state::pe_src_swap_source_on_ps_rdy>>,
    fsm::transition<fsm::from<state::pe_src_swap_source_on_ps_rdy>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_swap_source_start>>,
    fsm::transition<fsm::from<state::pe_src_swap_source_on_ps_rdy>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_swap_source_start>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_capabilities>>,
    // resets
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<pe::event::soft_reset_received>,
                    fsm::to<state::pe_src_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_soft_reset>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_send_capabilities>>,
    fsm::transition<fsm::from<state::pe_src_soft_reset>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_send_soft_reset>, fsm::on<pe::event::accept>,
                    fsm::to<state::pe_src_send_capabilities>>,
    fsm::transition<fsm::from<state::pe_src_send_soft_reset>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_send_soft_reset>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_hard_reset>, fsm::on<pe::event::hard_reset_complete>,
                    fsm::to<state::pe_src_transition_to_default>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<pe::event::hard_reset_received>,
                    fsm::to<state::pe_src_transition_to_default>>,
    // the hard-reset VBUS cycle: off and settled, tSrcRecover, the
    // defaults back and settled - then advertise again, or rest in
    // Startup when the sink is gone
    fsm::transition<fsm::from<state::pe_src_transition_to_default>,
                    fsm::on<event::supply_settled>, fsm::to<state::pe_src_recover>>,
    fsm::transition<fsm::from<state::pe_src_recover>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_restore_default>>,
    fsm::transition<fsm::from<state::pe_src_restore_default>, fsm::on<event::supply_settled>,
                    fsm::to<state::pe_src_send_capabilities>, fsm::guard<still_attached>>,
    fsm::transition<fsm::from<state::pe_src_restore_default>, fsm::on<event::supply_settled>,
                    fsm::to<state::pe_src_startup>>>;
// The engine's table with every disabled optional feature filtered
// out (a feature is enabled when the injected policy carries its
// arbitration hook). A named struct, not an alias: the short name
// replaces the fully spelled table type in every mangled symbol -
// megabytes per object file, measured
template<bool PR_SWAP, bool DR_SWAP, bool VCONN>
struct source_table_for
    : mtl::rebind_t<without_disabled_t<source_transitions, PR_SWAP, DR_SWAP, VCONN>,
                    fsm::transition_table> {};

template<bool PR_SWAP, bool DR_SWAP, bool VCONN>
using source_timer_ranges_for =
    without_disabled_t<source_timer_ranges, PR_SWAP, DR_SWAP, VCONN>;

// The table checks (timeout bounds, reachability, both variants)
// live in test/compliance.cpp - one dedicated TU pays for them

} // namespace pe

// The contract-notification side of the source engine, injectable as
// one observer. Derive from it (CRTP) and implement
// (concepts::source_power_client):
//
//   void onContract(millivolt, milliamp); // explicit contract in place
//   void onContractLost();                // back to vSafe5V defaults
//
// The supply itself is engine-owned; this observer only reports, with
// onContractLost() fired only when a contract was actually in place
template<typename DERIVED>
class SourcePower : public fsm::ObserverGroup<pe::contract_store<SourcePower<DERIVED>>,
                                              pe::power_effects<SourcePower<DERIVED>>> {
public:
    // store before the effects: the contract terms must be fresh when
    // the power annotation edge fires on the same entry
    SourcePower()
        : fsm::ObserverGroup<pe::contract_store<SourcePower>, pe::power_effects<SourcePower>>(
              store_, effects_)
    {
    }

    // checked once the machine is built, when DERIVED is complete
    static constexpr void validateClient()
    {
        static_assert(concepts::source_power_client<DERIVED>,
                      "SourcePower: the derived class must provide onContract(millivolt, "
                      "milliamp) and onContractLost()");
    }

private:
    friend pe::contract_store<SourcePower>;
    friend pe::power_effects<SourcePower>;

    DERIVED& derived() { return static_cast<DERIVED&>(*this); }

    void applyContract()
    {
        contract_active_ = true;
        derived().onContract(contract_.voltage, contract_.current);
    }

    void restoreDefaults()
    {
        if (contract_active_) {
            contract_active_ = false;
            derived().onContractLost();
        }
    }

    pe::contract_store<SourcePower> store_{*this};
    pe::power_effects<SourcePower> effects_{*this};
    pe::active_contract contract_{};
    bool contract_active_ = false;
};

template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER, concepts::source_policy POLICY,
         concepts::source_supply SUPPLY, typename... OBSERVERs>
class SourcePolicyEngine
    : public pe::PolicyEngineBase<SourcePolicyEngine<TCPC, TIMER, POLICY, SUPPLY, OBSERVERs...>,
                                  power_role::source, TCPC, TIMER, POLICY> {
    using base =
        pe::PolicyEngineBase<SourcePolicyEngine, power_role::source, TCPC, TIMER, POLICY>;
    friend base;

public:
    // The observers are injected into the engine's machine after the
    // protocol layer and the supply driver; a SourcePower-derived one
    // supplies the contract notifications
    SourcePolicyEngine(TCPC& tcpc, TIMER& prl_timer, TIMER& pe_timer,
                       std::span<std::uint32_t const> capabilities, POLICY& policy,
                       SUPPLY& supply, OBSERVERs&... observers)
        : base(tcpc, prl_timer, pe_timer),
          policy_(policy),
          supply_(supply),
          sm_(this->timed_, this->prl_, action_driver_, policy_, observers...)
    {
        this->source_capabilities_ = capabilities;
        supply_.setReadyCallback(
            [](void* self, bool at_target) {
                auto& engine = *static_cast<SourcePolicyEngine*>(self);
                if (at_target) {
                    engine.sm_.process(pe::event::supply_settled{});
                }
            },
            this);
    }

    // The Type-C source layer reports the attached sink: advertise
    void attached() { sm_.process(pe::event::attached{}); }

    // ... and the detach: everything resets, back to Startup; the
    // next partner negotiates its own revision
    void detached()
    {
        this->prl_.resetRevision();
        this->setBistTestData(false); // the test mode ends with the partner
        sm_.process(pe::event::detached{});
    }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there). Under PD3
    // the source signals SinkTxNG and waits tSinkTx before the request
    // goes out (collision avoidance)
    bool requestPowerSwap()
    {
        if constexpr (!base::pr_swap_capable) { // the feature is compiled out
            return false;
        } else {
            if (this->prl_.revision() == pd_revision::rev_3_x) {
                return sm_.process(pe::event::begin_pr_swap{});
            }
            return sm_.process(pe::event::send_pr_swap{});
        }
    }

    bool requestDataSwap()
    {
        if constexpr (!base::dr_swap_capable) { // the feature is compiled out
            return false;
        } else {
            if (this->prl_.revision() == pd_revision::rev_3_x) {
                return sm_.process(pe::event::begin_dr_swap{});
            }
            return sm_.process(pe::event::send_dr_swap{});
        }
    }

    // The port was the sink and asserted Rp mid PR_Swap: drive VBUS to
    // vSafe5V, announce PS_RDY, pause tSwapSourceStart, then advertise.
    // The event carries the preserved data role (the entered state
    // seeds the context with it); the negotiated revision and the
    // MessageID lifecycle hold for the connection - a swap is no reset
    // trigger (6.7.1) - and are handed over from the retiring engine
    void attachedAfterSwap(data_role role, pd_revision revision,
                           prl::message_id_state const& ids)
    {
        this->prl_.seedRevision(revision);
        this->prl_.seedMessageIds(ids);
        sm_.process(pe::event::attached_swap{role});
    }

    // A DRP announces its sink-role capabilities: Get_Sink_Cap is
    // answered with them instead of Not_Supported
    void provideSinkCapabilities(std::span<sink_capability const> capabilities)
    {
        this->sink_capabilities_ = capabilities;
    }

private:
    // Executes the states' engine-directed annotations, delivered
    // from their sets by overload: the Source_Capabilities
    // transmission, the PD3 collision-avoidance Rp (only meaningful
    // under an explicit contract with a PD3 partner - elsewhere the
    // configured advertisement stands), and the supply's vSafe5V
    // restore; the settle states' target arrives through the
    // nonstatic observation
    struct ActionDriver : fsm::observing<ActionDriver> {
        explicit ActionDriver(SourcePolicyEngine& engine_ref) : engine(engine_ref) {}

        void notifyEntry(pe::send_capabilities_action) { engine.transmitSourceCaps(); }
        void notifyEntry(pe::sink_tx tx)
        {
            if (engine.prl_.revision() != pd_revision::rev_3_x ||
                !engine.sm_.template context<pe::src_negotiation>().explicit_contract) {
                return;
            }
            engine.tcpc_.setCc(cc_pull::rp,
                               tx == pe::sink_tx::ok ? rp_value::p_3a0 : rp_value::p_1a5);
        }
        void notifyEntry(pe::restore_default_action)
        {
            engine.supply_.setOutput(pe::v_safe_5v, pe::i_default_current);
        }

        void notifyEntry(supply_target target)
        {
            engine.supply_.setOutput(target.voltage, target.current);
        }

        SourcePolicyEngine& engine;
    };

    // --- the base's hooks ---------------------------------------------------

    // The source's own messages: the Request to negotiate, and
    // Get_Source_Cap answered by re-advertising
    bool dispatchRole(pd_header const& header, pd_message const& message)
    {
        if (isData(header, data_message_type::request)) {
            negotiate(message);
            return true;
        }
        if (isControl(header, control_message_type::get_source_cap)) {
            sm_.process(pe::event::get_source_caps{});
            return true;
        }
        return false;
    }

    // the contract's voltage while an explicit contract holds (BIST is
    // honored at vSafe5V only)
    std::optional<millivolt> contractVoltage() const
    {
        auto const& negotiation = sm_.template context<pe::src_negotiation>();
        if (!negotiation.explicit_contract) {
            return std::nullopt;
        }
        return negotiation.target.voltage;
    }

    // --- the source's messages ----------------------------------------------

    void transmitSourceCaps()
    {
        this->prl_.transmit(pe::makeSourceCapabilitiesMessage(
            power_role::source, this->dataRole(), this->source_capabilities_));
    }

    // PE_SRC_Negotiate_Capability: the policy's verdict advances the
    // machine with request_ok or request_bad
    void negotiate(pd_message const& message)
    {
        if (!sm_.process(pe::event::request{})) {
            return; // not in a state that takes a Request
        }
        auto const rdo = dataObjectAt(message, 0);
        if (auto const target = policy_.evaluate(rdo, this->source_capabilities_)) {
            sm_.process(pe::event::request_ok{.target = *target});
        } else {
            sm_.process(pe::event::request_bad{});
        }
    }

    POLICY& policy_;
    SUPPLY& supply_;
    ActionDriver action_driver_{*this};
    // the policy rides in the pack to answer the table's questions
    fsm::QueuedMachine<pe::source_table_for<base::pr_swap_capable, base::dr_swap_capable,
                                            base::vconn_capable>,
                       4, fsm::inline_work, fsm::no_lock, fsm::timed<fsm::QueuedTimer<TIMER>&>,
                       ProtocolLayer<TCPC, TIMER>, ActionDriver, POLICY, OBSERVERs...>
        sm_;
};

} // namespace usbc
