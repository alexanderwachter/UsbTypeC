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
 * the states' prl_action commands and txMessage() transmissions - and
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
    millivolt voltage     = 5000;
    milliamp operating_current = 0;
    milliamp maximum_current   = 0;
    bool mismatch              = false;
};

namespace concepts {

template<typename T>
concept sink_policy = requires(T policy, std::span<std::uint32_t const> source_capabilities,
                               std::span<sink_capability const> capabilities) {
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
        std::span<sink_capability const> capabilities) const
    {
        std::optional<contract_request> best;
        milliwatt best_power = 0;
        for (std::uint8_t index = 0; index < source_capabilities.size(); ++index) {
            auto const object = source_capabilities[index];
            if (pdo::kindOf(object) != pdo::kind::fixed_supply) {
                continue;
            }
            auto const voltage  = pdo::fixedVoltage(object);
            auto const accepted = std::find_if(
                capabilities.begin(), capabilities.end(),
                [voltage](sink_capability capability) { return capability.voltage == voltage; });
            if (accepted == capabilities.end()) {
                continue; // the sink cannot take this voltage
            }
            auto const wanted = static_cast<milliamp>(
                (static_cast<std::int64_t>(max_power_) * 1000) / voltage);
            auto const current =
                std::min({pdo::fixedMaxCurrent(object), accepted->current, wanted});
            auto const power   = static_cast<milliwatt>(
                (static_cast<std::int64_t>(voltage) * current) / 1000);
            if (power < min_power_) {
                continue;
            }
            if (!best || power > best_power ||
                (power == best_power && voltage < best->voltage)) {
                best_power = power;
                best       = contract_request{.position          = static_cast<std::uint8_t>(index + 1),
                                              .voltage           = voltage,
                                              .operating_current = current,
                                              .maximum_current   = current,
                                              .mismatch          = false};
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
inline constexpr auto t_no_response   = std::chrono::milliseconds{5000}; // tNoResponse
inline constexpr auto t_sink_request  = std::chrono::milliseconds{150}; // tSinkRequest
inline constexpr auto t_pr_swap_wait  = std::chrono::milliseconds{150}; // tPRSwapWait
inline constexpr auto t_dr_swap_wait  = std::chrono::milliseconds{150}; // tDRSwapWait

inline constexpr milliamp i_snk_stdby = spec::i_snk_stdby; // at any voltage

struct pe_context {
    contract_request pending{};  // proposed by the last Request
    contract_request request{};  // accepted by the source
    pd_message reply{};          // pending Not_Supported answer
    pd_message request_message{}; // the last Request, for the Wait retry
    bool explicit_contract = false;
    data_role data = data_role::ufp;  // flipped by an agreed DR_Swap
    std::uint8_t hard_resets = 0;     // HardResetCounter
};

// The observation the sink's standby transition reports
struct standby_limit {
    millivolt voltage;
};

namespace event {

struct started {};
struct vbus_present {};
struct vbus_removed {};
struct source_capabilities {};
struct capabilities_evaluated {
    pd_message message;
    contract_request terms;
};
struct send_sink_caps {
    pd_message message;
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
    static constexpr prl::reset_action prl_action{};
    static constexpr restore_default_action power_action{};
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    // detach forgets everything; a reset within the connection keeps
    // the data role (a hard reset does not change it) and the
    // HardResetCounter
    pe_snk_startup(event::vbus_removed const&, pe_context& ctx) : context(ctx)
    {
        context = {};
    }
    explicit pe_snk_startup(pe_context& ctx) : context(ctx)
    {
        context = pe_context{.data = context.data, .hard_resets = context.hard_resets};
    }
    pe_context& context;
};

// Waits for the Type-C layer to report VBUS
struct pe_snk_discovery {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_discovery(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_wait_for_capabilities {
    static constexpr auto timeout = t_sink_wait_cap; // SinkWaitCapTimer
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_wait_for_capabilities(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// PE_SNK_Wait_for_Capabilities after a hard reset: the governing
// deadline is NoResponseTimer - its expiry hard-resets again while
// HardResetCounter allows, then gives up into Type-C Error Recovery
struct pe_snk_wait_no_response {
    static constexpr auto timeout = t_no_response; // NoResponseTimer
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_wait_no_response(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// nHardResetCount exhausted with no response: the port-level
// integration commands Type-C Error Recovery, whose teardown resets
// this engine
struct pe_snk_error_recovery {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_error_recovery(pe_context& ctx) : context(ctx) {}

    request_error_recovery portReport() const { return {}; }

    pe_context& context;
};

// The engine evaluates through the injected policy and advances with
// capabilities_evaluated
struct pe_snk_evaluate_capability {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_evaluate_capability(pe_context& ctx) : context(ctx)
    {
        context.hard_resets = 0; // spec: reset on Source_Capabilities
    }
    pe_context& context;
};

struct pe_snk_select_capability {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    // the proposal stays pending: only an Accept promotes it, so a
    // Reject cannot leak the proposed terms into the active contract
    pe_snk_select_capability(event::capabilities_evaluated const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
        context.pending         = event.terms;
        context.request_message = event.message; // kept for the Wait retry
    }
    // re-entry from the SinkRequestTimer: the same Request again
    pe_snk_select_capability(event::request_retry const&, pe_context& ctx)
        : pe_snk_select_capability(ctx)
    {
    }
    explicit pe_snk_select_capability(pe_context& ctx)
        : context(ctx), message_(ctx.request_message)
    {
    }

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

struct pe_snk_transition_sink {
    static constexpr auto timeout = t_ps_transition; // PSTransitionTimer
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    // entered on Accept: the pending proposal becomes the contract
    pe_snk_transition_sink(event::accept const&, pe_context& ctx) : context(ctx)
    {
        context.request = context.pending;
    }
    explicit pe_snk_transition_sink(pe_context& ctx) : context(ctx) {}

    standby_limit report() const { return {context.request.voltage}; }

    pe_context& context;
};

struct pe_snk_ready {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_ready(pe_context& ctx) : context(ctx) { context.explicit_contract = true; }

    active_contract report() const
    {
        return {context.request.voltage, context.request.operating_current};
    }

    pe_context& context;
};

struct pe_snk_give_sink_cap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_snk_give_sink_cap(event::send_sink_caps const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit pe_snk_give_sink_cap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_SNK_BIST_Carrier_Mode: the tester's carrier runs for
// tBISTContMode (the engine commanded the TCPC on entry), then normal
// operation resumes
struct pe_snk_bist_carrier {
    static constexpr auto timeout = t_bist_cont_mode; // BISTContModeTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "transmits the BIST carrier";

    explicit pe_snk_bist_carrier(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// The spec's Ready-with-SinkRequestTimer after a Wait answer to our
// Request: the same Request goes out again after tSinkRequest; new
// capabilities from the source preempt the retry
struct pe_snk_request_wait {
    static constexpr auto timeout = t_sink_request; // SinkRequestTimer
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_request_wait(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// PE_DR_SNK_Give_Source_Cap: a DRP answers Get_Source_Cap with its
// source-role capabilities, then returns to Ready
struct pe_dr_snk_give_source_cap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_dr_snk_give_source_cap(event::send_source_caps const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit pe_dr_snk_give_source_cap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_SNK_Send_Not_Supported: answers a message the sink does not
// support, then returns to Ready
struct pe_snk_send_not_supported {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_snk_send_not_supported(event::unsupported const& event, pe_context& ctx) : context(ctx)
    {
        context.reply = event.reply;
    }
    explicit pe_snk_send_not_supported(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return context.reply; }

    pe_context& context;
};

// PE_SNK_Chunk_Received: a non-chunking device lets the sender run
// into its chunking timeout before answering Not_Supported
struct pe_snk_chunk_received {
    static constexpr auto timeout = t_chunking_not_supported; // ChunkingNotSupportedTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_snk_chunk_received(event::chunked_message const& event, pe_context& ctx) : context(ctx)
    {
        context.reply = event.reply;
    }
    explicit pe_snk_chunk_received(pe_context& ctx) : context(ctx) {}

    pe_context& context;
};

// --- role swap messaging (PE_DRS / PE_PRS, sink side) ------------------------

// PE_DRS_UFP_DFP/DFP_UFP_Send_Swap: our DR_Swap is out; no answer
// within tSenderResponse means the partner ignored it - stay Ready
struct pe_snk_send_dr_swap {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends DR_Swap";

    pe_snk_send_dr_swap(event::send_dr_swap const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    // re-entry from the tDRSwapWait retry rebuilds the request
    explicit pe_snk_send_dr_swap(pe_context& ctx)
        : context(ctx), message_(makeControlMessage(control_message_type::dr_swap,
                                                    power_role::sink, ctx.data))
    {
    }

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_DRS_*_Accept_Swap: the partner's DR_Swap passed the arbitration
struct pe_snk_accept_dr_swap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_snk_accept_dr_swap(event::dr_swap_accepted const& event, pe_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_snk_accept_dr_swap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_DRS_*_Change_to_*: the agreed swap flips the data role; the
// report lets the port update the TCPC header and the Type-C context
struct pe_snk_dr_swap_change {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "flips the data role";

    pe_snk_dr_swap_change(event::accept const&, pe_context& ctx) : pe_snk_dr_swap_change(ctx) {}
    pe_snk_dr_swap_change(event::message_sent const&, pe_context& ctx)
        : pe_snk_dr_swap_change(ctx)
    {
    }
    explicit pe_snk_dr_swap_change(pe_context& ctx) : context(ctx)
    {
        context.data = context.data == data_role::ufp ? data_role::dfp : data_role::ufp;
    }

    data_role_changed portReport() const { return {context.data}; }

    pe_context& context;
};

// PE_VCS_Send_Swap: our VCONN_Swap is out
struct pe_snk_vcs_send_swap {
    static constexpr bool vconn_feature = true;
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends VCONN_Swap";

    pe_snk_vcs_send_swap(event::send_vconn_swap const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit pe_snk_vcs_send_swap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_VCS_Accept_Swap: the partner's VCONN_Swap passed the arbitration
struct pe_snk_vcs_accept {
    static constexpr bool vconn_feature = true;
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_snk_vcs_accept(event::vconn_swap_accepted const& event, pe_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_snk_vcs_accept(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// The agreed swap's message anchor (glue, not a spec state): the
// vconn machine choreographs the hand-off; this engine relays the
// PS_RDY traffic and stays here until its side is done
struct pe_snk_vcs_active {
    static constexpr bool vconn_feature = true;
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_snk_vcs_active(event::accept const&, pe_context& ctx) : pe_snk_vcs_active(ctx) {}
    pe_snk_vcs_active(event::message_sent const&, pe_context& ctx) : pe_snk_vcs_active(ctx) {}
    explicit pe_snk_vcs_active(pe_context& ctx) : context(ctx) {}

    vconn_swap_agreed portReport() const { return {}; }

    pe_context& context;
};

// PE_VCS_Send_PS_RDY: the vconn machine turned the switch on
struct pe_snk_vcs_send_ps_rdy {
    static constexpr bool vconn_feature = true;
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY";

    pe_snk_vcs_send_ps_rdy(event::send_vconn_ps_rdy const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit pe_snk_vcs_send_ps_rdy(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// Transients reporting the hand-off progress to the vconn machine
struct pe_snk_vcs_partner_on {
    static constexpr bool vconn_feature = true;
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_vcs_partner_on(pe_context& ctx) : context(ctx) {}

    vconn_partner_on portReport() const { return {}; }

    pe_context& context;
};

struct pe_snk_vcs_ps_rdy_sent {
    static constexpr bool vconn_feature = true;
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_vcs_ps_rdy_sent(pe_context& ctx) : context(ctx) {}

    vconn_ps_rdy_sent portReport() const { return {}; }

    pe_context& context;
};

// PE_PRS_SNK_SRC_Send_Swap: our PR_Swap is out
struct pe_snk_send_pr_swap {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PR_Swap";

    pe_snk_send_pr_swap(event::send_pr_swap const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    // re-entry from the tPRSwapWait retry rebuilds the request
    explicit pe_snk_send_pr_swap(pe_context& ctx)
        : context(ctx), message_(makeControlMessage(control_message_type::pr_swap,
                                                    power_role::sink, ctx.data))
    {
    }

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_PRS_SNK_SRC_Accept_Swap: the partner's PR_Swap passed arbitration
struct pe_snk_accept_pr_swap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_snk_accept_pr_swap(event::pr_swap_accepted const& event, pe_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_snk_accept_pr_swap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// The partner answered Wait: the swap request is retried after the
// spec's pause (still Ready, spec-wise)
struct pe_snk_dr_swap_wait {
    static constexpr auto timeout = t_dr_swap_wait; // tDRSwapWait
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_dr_swap_wait(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_pr_swap_wait {
    static constexpr auto timeout = t_pr_swap_wait; // tPRSwapWait
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_pr_swap_wait(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// The retry is due but PD3 collision avoidance gates it: the engine
// re-initiates once the source's Rp says SinkTxOk (no timeout - the
// source owns the schedule)
struct pe_snk_request_gate {
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_request_gate(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_dr_swap_gate {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_dr_swap_gate(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_pr_swap_gate {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_pr_swap_gate(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// PE_PRS_SNK_SRC_Transition_to_off: draw drops to standby while the
// old source turns off; the port holds the connection layer's swap
// standby, whose tPSSourceOff timeout restarts connection resolution
// (the spec's Error Recovery outcome) if the PS_RDY never comes
struct pe_snk_swap_transition_to_off {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "stops drawing, awaits the source's PS_RDY";

    pe_snk_swap_transition_to_off(event::accept const&, pe_context& ctx)
        : pe_snk_swap_transition_to_off(ctx)
    {
    }
    pe_snk_swap_transition_to_off(event::message_sent const&, pe_context& ctx)
        : pe_snk_swap_transition_to_off(ctx)
    {
    }
    explicit pe_snk_swap_transition_to_off(pe_context& ctx) : context(ctx) {}

    standby_limit report() const { return {v_safe_5v}; }
    enter_swap_standby portReport() const { return {power_role::source}; }

    pe_context& context;
};

// PE_PRS_SNK_SRC_Assert_Rp: the old source is off - the port flips its
// termination now and continues as the new source
struct pe_snk_swap_assert_rp {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_swap_assert_rp(pe_context& ctx) : context(ctx) {}

    assert_new_role portReport() const { return {power_role::source}; }

    pe_context& context;
};

// PE_PRS_SRC_SNK_Wait_Source_on, this engine's half: the port was the
// source and asserted Rd - our PS_RDY announces the supply is off, the
// new source's PS_RDY is awaited (the connection layer's swap standby
// holds the tPSSourceOn deadline and restarts resolution on timeout)
struct pe_snk_swap_wait_source_on {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY, awaits the new source's";

    pe_snk_swap_wait_source_on(event::swap_wait_source_on const& event, pe_context& ctx)
        : context(ctx)
    {
        context.data = event.role; // a power swap preserves the data role
        message_ = makeControlMessage(control_message_type::ps_rdy, power_role::sink,
                                      context.data);
    }
    explicit pe_snk_swap_wait_source_on(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// The new source's PS_RDY arrived: the port completes the swap into
// Attached.SNK, then the normal sink flow resumes
struct pe_snk_swap_source_on_seen {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_swap_source_on_seen(pe_context& ctx) : context(ctx) {}

    swap_completed portReport() const { return {}; }

    pe_context& context;
};

// Accepts a received Soft_Reset; the reporter resets the protocol
// layer before the sender transmits the Accept
struct pe_snk_soft_reset {
    static constexpr prl::reset_action prl_action{};
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    pe_snk_soft_reset(event::soft_reset_received const& event, pe_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_snk_soft_reset(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// PE_SNK_Send_Soft_Reset: protocol errors first try a soft reset; the
// protocol layer is reset before the Soft_Reset goes out
struct pe_snk_send_soft_reset {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr prl::reset_action prl_action{};
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd        = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    explicit pe_snk_send_soft_reset(pe_context& ctx) : context(ctx)
    {
        context.reply = makeControlMessage(control_message_type::soft_reset, power_role::sink,
                                           context.data);
    }

    pd_message const& txMessage() const { return context.reply; }

    pe_context& context;
};

struct pe_snk_hard_reset {
    static constexpr prl::hard_reset_action prl_action{};
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::hard_reset_action::note;

    explicit pe_snk_hard_reset(pe_context& ctx) : context(ctx)
    {
        ++context.hard_resets; // HardResetCounter
    }
    pe_context& context;
};

// PE_SNK_Transition_to_default: back to vSafe5V defaults; the engine
// then advances through Startup and Discovery
struct pe_snk_transition_to_default {
    static constexpr restore_default_action power_action{};
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd        = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = restore_default_action::note;

    explicit pe_snk_transition_to_default(pe_context& ctx) : context(ctx)
    {
        // the connection persists: keep the data role and the counter
        context = pe_context{.data = context.data, .hard_resets = context.hard_resets};
    }

    // the source legitimately cycles VBUS now: the connection layer
    // must hold the attach instead of reading it as a detach
    hard_reset_window portReport() const { return {}; }

    pe_context& context;
};

} // namespace state

struct has_explicit_contract {
    static bool check(state::pe_snk_select_capability const& state)
    {
        return state.context.explicit_contract;
    }
};

// Which capability wait applies: SinkWaitCapTimer before the first
// hard reset, NoResponseTimer afterwards
struct no_hard_reset_yet {
    static bool check(state::pe_snk_discovery const& state)
    {
        return state.context.hard_resets == 0;
    }
};

struct hard_resets_left {
    static bool check(state::pe_snk_wait_no_response const& state)
    {
        return state.context.hard_resets <= spec::n_hard_reset_count;
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
    fsm::transition<fsm::from<state::pe_snk_startup>, fsm::on<event::started>,
                    fsm::to<state::pe_snk_discovery>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::vbus_removed>,
                    fsm::to<state::pe_snk_startup>>,
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::vbus_present>,
                    fsm::to<state::pe_snk_wait_for_capabilities>,
                    fsm::guard<no_hard_reset_yet>>,
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::vbus_present>,
                    fsm::to<state::pe_snk_wait_no_response>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>,
                    fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
    // after a hard reset: another one while the counter allows, Error
    // Recovery when it is spent
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>,
                    fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>, fsm::guard<hard_resets_left>>,
    fsm::transition<fsm::from<state::pe_snk_wait_no_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_error_recovery>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_evaluate_capability>,
                    fsm::on<event::capabilities_evaluated>,
                    fsm::to<state::pe_snk_select_capability>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::accept>,
                    fsm::to<state::pe_snk_transition_sink>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::reject>,
                    fsm::to<state::pe_snk_ready>, fsm::guard<has_explicit_contract>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::reject>,
                    fsm::to<state::pe_snk_wait_for_capabilities>>,
    // Wait: retry the same Request after tSinkRequest; fresh
    // capabilities preempt the retry
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_request_wait>>,
    fsm::transition<fsm::from<state::pe_snk_request_wait>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_request_gate>>,
    fsm::transition<fsm::from<state::pe_snk_request_gate>, fsm::on<event::request_retry>,
                    fsm::to<state::pe_snk_select_capability>>,
    fsm::transition<fsm::from<state::pe_snk_request_gate>, fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_request_wait>, fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_transition_sink>, fsm::on<event::ps_rdy>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_transition_sink>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_sink_caps>,
                    fsm::to<state::pe_snk_give_sink_cap>>,
    fsm::transition<fsm::from<state::pe_snk_give_sink_cap>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_give_sink_cap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_source_caps>,
                    fsm::to<state::pe_dr_snk_give_source_cap>>,
    fsm::transition<fsm::from<state::pe_dr_snk_give_source_cap>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_dr_snk_give_source_cap>,
                    fsm::on<event::protocol_error>, fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::unsupported>,
                    fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::bist_carrier>,
                    fsm::to<state::pe_snk_bist_carrier>>,
    fsm::transition<fsm::from<state::pe_snk_bist_carrier>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::chunked_message>,
                    fsm::to<state::pe_snk_chunk_received>>,
    fsm::transition<fsm::from<state::pe_snk_chunk_received>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_send_not_supported>>,
    // DR_Swap: sent from Ready, or accepted there; both sides flip on
    // the agreement, an ignored request falls back to Ready
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_dr_swap>,
                    fsm::to<state::pe_snk_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::accept>,
                    fsm::to<state::pe_snk_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::reject>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_dr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_wait>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_dr_swap_gate>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_gate>, fsm::on<event::send_dr_swap>,
                    fsm::to<state::pe_snk_send_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_dr_swap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::dr_swap_accepted>,
                    fsm::to<state::pe_snk_accept_dr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_accept_dr_swap>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_snk_accept_dr_swap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_dr_swap_change>, fsm::on<event::swap_done>,
                    fsm::to<state::pe_snk_ready>>,
    // VCONN_Swap: the messages anchor here, the vconn machine owns
    // the role, the switch, and the hand-off deadline
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_vconn_swap>,
                    fsm::to<state::pe_snk_vcs_send_swap>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::accept>,
                    fsm::to<state::pe_snk_vcs_active>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::reject>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_swap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::vconn_swap_accepted>,
                    fsm::to<state::pe_snk_vcs_accept>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_accept>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_vcs_active>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_accept>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::ps_rdy>,
                    fsm::to<state::pe_snk_vcs_partner_on>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::send_vconn_ps_rdy>,
                    fsm::to<state::pe_snk_vcs_send_ps_rdy>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_active>, fsm::on<event::hard_reset_request>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_ps_rdy>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_vcs_ps_rdy_sent>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_send_ps_rdy>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_partner_on>, fsm::on<event::swap_done>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_vcs_ps_rdy_sent>, fsm::on<event::swap_done>,
                    fsm::to<state::pe_snk_ready>>,
    // PR_Swap while sinking: the agreement leads into Transition_to_off,
    // the old source's PS_RDY into the termination flip
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::send_pr_swap>,
                    fsm::to<state::pe_snk_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::accept>,
                    fsm::to<state::pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::reject>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_pr_swap_wait>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_wait>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_pr_swap_gate>>,
    fsm::transition<fsm::from<state::pe_snk_pr_swap_gate>, fsm::on<event::send_pr_swap>,
                    fsm::to<state::pe_snk_send_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_pr_swap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::pr_swap_accepted>,
                    fsm::to<state::pe_snk_accept_pr_swap>>,
    fsm::transition<fsm::from<state::pe_snk_accept_pr_swap>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_swap_transition_to_off>>,
    fsm::transition<fsm::from<state::pe_snk_accept_pr_swap>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_swap_transition_to_off>, fsm::on<event::ps_rdy>,
                    fsm::to<state::pe_snk_swap_assert_rp>>,
    // PR_Swap's other half: this port was the source and asserted Rd -
    // PS_RDY out, then the new source's PS_RDY completes the swap
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::swap_wait_source_on>,
                    fsm::to<state::pe_snk_swap_wait_source_on>>,
    fsm::transition<fsm::from<state::pe_snk_swap_wait_source_on>, fsm::on<event::ps_rdy>,
                    fsm::to<state::pe_snk_swap_source_on_seen>>,
    fsm::transition<fsm::from<state::pe_snk_swap_wait_source_on>,
                    fsm::on<event::protocol_error>, fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_swap_source_on_seen>, fsm::on<event::swap_done>,
                    fsm::to<state::pe_snk_discovery>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::soft_reset_received>,
                    fsm::to<state::pe_snk_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_soft_reset>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_soft_reset>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<event::accept>,
                    fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_send_soft_reset>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_hard_reset>, fsm::on<event::hard_reset_complete>,
                    fsm::to<state::pe_snk_transition_to_default>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_received>,
                    fsm::to<state::pe_snk_transition_to_default>>,
    fsm::transition<fsm::from<state::pe_snk_transition_to_default>,
                    fsm::on<event::default_level_reached>, fsm::to<state::pe_snk_startup>>>;

// The engine's table, with the optional VCONN feature filtered out
// when the injected policy cannot arbitrate it
template<bool VCONN>
using sink_table_for = mtl::rebind_t<
    std::conditional_t<VCONN, sink_transitions,
                       mtl::remove_if_t<sink_transitions, touches_vconn>>,
    fsm::transition_table>;

template<bool VCONN>
using sink_timer_ranges_for =
    std::conditional_t<VCONN, sink_timer_ranges,
                       mtl::remove_if_t<sink_timer_ranges, times_vconn_state>>;

// The table checks (timeout bounds, reachability, both variants)
// live in test/compliance.cpp - one dedicated TU pays for them

// The member observers behind SinkPower (POWER is SinkPower<DERIVED>);
// injected together as one fsm::observer_group

// Stores the runtime values the states report: the contract terms on
// Ready entry, and the standby limit applied during the transition
template<typename POWER>
struct contract_store : fsm::observing<contract_store<POWER>> {
    explicit contract_store(POWER& power_ref) : power(power_ref) {}

    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert(concepts::sink_power_client<typename POWER::derived_type>,
                      "SinkPower: the derived class must provide setLimit(millivolt, "
                      "milliamp), onContract(millivolt, milliamp), onContractLost()");
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.report()))
    {
        return state.report();
    }
    void notifyEntry(standby_limit limit)
    {
        power.derived().setLimit(limit.voltage, i_snk_stdby);
    }
    // store only - contract_apply acts on the power annotation edge
    void notifyEntry(active_contract contract) { power.contract_ = contract; }

    POWER& power;
};

// Applies the stored contract exactly when the diagram's Power column
// changes to Explicit Contract; with change suppression, bounces
// between Ready and its service states stay silent
template<typename POWER>
struct contract_apply : fsm::observing<contract_apply<POWER>> {
    explicit contract_apply(POWER& power_ref) : power(power_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::power)
    {
        return STATE::power;
    }
    void notifyEntry(power_level level)
    {
        if (level == power_level::explicit_contract) {
            power.applyContract();
        }
    }

    POWER& power;
};

// Restores vSafe5V defaults on the states carrying a restore
// power_action (Startup, Transition_to_default)
template<typename POWER>
struct default_restore : fsm::observing<default_restore<POWER>> {
    explicit default_restore(POWER& power_ref) : power(power_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::power_action)
    {
        return STATE::power_action;
    }
    void notifyEntry(restore_default_action) { power.restoreDefaults(); }

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
class SinkPower : public fsm::observer_group<pe::contract_store<SinkPower<DERIVED>>,
                                             pe::contract_apply<SinkPower<DERIVED>>,
                                             pe::default_restore<SinkPower<DERIVED>>> {
public:
    using derived_type = DERIVED;

    // store before apply: the contract terms must be fresh when the
    // power annotation edge fires on the same entry
    SinkPower()
        : fsm::observer_group<pe::contract_store<SinkPower>, pe::contract_apply<SinkPower>,
                              pe::default_restore<SinkPower>>(store_, apply_, restore_)
    {
    }

private:
    friend pe::contract_store<SinkPower>;
    friend pe::contract_apply<SinkPower>;
    friend pe::default_restore<SinkPower>;

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
    pe::contract_apply<SinkPower> apply_{*this};
    pe::default_restore<SinkPower> restore_{*this};
    pe::active_contract contract_{};
    bool contract_active_ = false;
};

template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER, concepts::sink_policy POLICY,
         typename... OBSERVERs>
class SinkPolicyEngine {
public:
    // The observers are injected into the engine's machine after the
    // protocol layer; a SinkPower-derived one supplies the power side
    SinkPolicyEngine(TCPC& tcpc, TIMER& prl_timer, TIMER& pe_timer,
                     std::span<sink_capability const> capabilities, POLICY& policy,
                     OBSERVERs&... observers)
        : tcpc_(tcpc),
          capabilities_(capabilities),
          policy_(policy),
          prl_pumped_{prl_timer, *this},
          prl_(tcpc, prl_pumped_, port_),
          pumped_{pe_timer, *this},
          timed_(pumped_),
          sm_(timed_, prl_, observers...)
    {
        tcpc_.setMessageHeaderInfo(
            {power_role::sink, data_role::ufp, pd_revision::rev_3_x});
        tcpc_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
        sm_.process(pe::event::started{}); // rest in Discovery until VBUS
    }

    // The Type-C layer reports attach: for a sink, VBUS is present
    void vbusPresent() { sm_.process(pe::event::vbus_present{}); }

    // ... and detach: negotiation state is gone, back to Discovery;
    // the next partner negotiates its own revision
    void vbusRemoved()
    {
        prl_.resetRevision();
        setBistTestData(false); // the test mode ends with the partner
        pending_ams_ = pending_ams::none;
        sm_.process(pe::event::vbus_removed{});
        sm_.process(pe::event::started{});
    }

    // Feed the TCPC's PD alerts (message/transmit/hard reset bits)
    void onAlert(alert_status alerts) { prl_.onAlert(alerts); }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there). Under PD3
    // collision avoidance a request during SinkTxNG parks and fires
    // when the source's Rp says SinkTxOk
    bool requestPowerSwap()
    {
        if (!sm_.template is<pe::state::pe_snk_ready>()) {
            return false;
        }
        if (!sinkTxAllows()) {
            pending_ams_ = pending_ams::pr_swap;
            return true;
        }
        return sm_.process(
            pe::event::send_pr_swap{makeControl(control_message_type::pr_swap)});
    }

    bool requestDataSwap()
    {
        if (!sm_.template is<pe::state::pe_snk_ready>()) {
            return false;
        }
        if (!sinkTxAllows()) {
            pending_ams_ = pending_ams::dr_swap;
            return true;
        }
        return sm_.process(
            pe::event::send_dr_swap{makeControl(control_message_type::dr_swap)});
    }

    // VCONN_Swap, driven by the vconn machine through the facade:
    // request the swap, transmit our PS_RDY once the switch is on,
    // and escalate a failed hand-off
    bool requestVconnSwap()
    {
        return sm_.process(
            pe::event::send_vconn_swap{makeControl(control_message_type::vconn_swap)});
    }

    bool sendVconnPsRdy()
    {
        return sm_.process(
            pe::event::send_vconn_ps_rdy{makeControl(control_message_type::ps_rdy)});
    }

    bool hardReset() { return sm_.process(pe::event::hard_reset_request{}); }

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
    // context with it); the negotiated revision holds for the
    // connection and is handed over from the retiring engine
    void startSwapWaitSourceOn(data_role role, pd_revision revision)
    {
        prl_.seedRevision(revision);
        sm_.process(pe::event::swap_wait_source_on{role});
    }

    // The revision the protocol layer negotiated with this partner
    pd_revision negotiatedRevision() const { return prl_.revision(); }

    // A DRP announces its source-role capabilities: Get_Source_Cap is
    // answered with them instead of Not_Supported
    void provideSourceCapabilities(std::span<std::uint32_t const> capabilities)
    {
        source_capabilities_ = capabilities;
    }

    // The facade's deferred port actions run through this hook once a
    // timeout-driven transition finished processing - the machine is
    // idle then (message-driven actions pump after alert routing)
    void setIdleHook(void (*hook)(void*), void* hook_context)
    {
        idle_hook_    = hook;
        idle_context_ = hook_context;
    }

    // The swap completed into Attached.SNK: resume the sink flow (the
    // new source's PS_RDY implies VBUS is live)
    void finishSwap()
    {
        sm_.process(pe::event::swap_done{});
        sm_.process(pe::event::vbus_present{});
    }

private:
    // The engine's timer, wrapped: a timeout-driven transition may ask
    // the port for an action that tears this engine down - the idle
    // hook runs it once the machine finished processing
    struct PumpedTimer {
        TIMER& inner;
        SinkPolicyEngine& pe;
        fsm::timer_callback callback = nullptr;
        void* context                = nullptr;

        void start(std::chrono::milliseconds duration, fsm::timer_callback cb, void* ctx)
        {
            callback = cb;
            context  = ctx;
            inner.start(
                duration,
                [](void* self) {
                    auto& timer = *static_cast<PumpedTimer*>(self);
                    timer.callback(timer.context);
                    timer.pe.afterTimeout();
                    if (timer.pe.idle_hook_ != nullptr) {
                        timer.pe.idle_hook_(timer.pe.idle_context_);
                    }
                },
                this);
        }
        void stop() { inner.stop(); }
    };

    // The protocol layer's client, forwarding into the engine
    struct PrlPort {
        SinkPolicyEngine& pe;

        void onMessage(pd_message const& message) { pe.dispatch(message); }
        // adopted revision: the TCPC's GoodCRC header must follow
        void onRevision(pd_revision revision)
        {
            pe.tcpc_.setMessageHeaderInfo(
                {power_role::sink, pe.sm_.template context<pe::pe_context>().data, revision});
        }
        void onTxDone()
        {
            pe.sm_.process(pe::event::message_sent{});
            pe.advanceTransients();
        }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { pe.sm_.process(pe::event::protocol_error{}); }
        void onHardReset()
        {
            pe.setBistTestData(false); // a hard reset ends the test mode
            pe.sm_.process(pe::event::hard_reset_received{});
            pe.restart();
        }
        void onHardResetSent()
        {
            pe.setBistTestData(false);
            pe.sm_.process(pe::event::hard_reset_complete{});
            pe.restart();
        }
    };

    // Advances the transient spec states after a hard reset:
    // Transition_to_default -> Startup -> Discovery, where the engine
    // waits for the Type-C layer to report the returning VBUS
    void restart()
    {
        sm_.process(pe::event::default_level_reached{});
        sm_.process(pe::event::started{});
    }

    // Collision avoidance applies under PD3 with an explicit contract;
    // otherwise the sink initiates freely
    bool sinkTxAllows() const
    {
        if (prl_.revision() != pd_revision::rev_3_x) {
            return true;
        }
        return !sm_.template context<pe::pe_context>().explicit_contract || sink_tx_ok_;
    }

    // A retry that timed out under SinkTxNG waits in its gate state
    void fireGated()
    {
        if (sm_.template is<pe::state::pe_snk_request_gate>()) {
            sm_.process(pe::event::request_retry{});
        } else if (sm_.template is<pe::state::pe_snk_dr_swap_gate>()) {
            sm_.process(pe::event::send_dr_swap{makeControl(control_message_type::dr_swap)});
        } else if (sm_.template is<pe::state::pe_snk_pr_swap_gate>()) {
            sm_.process(pe::event::send_pr_swap{makeControl(control_message_type::pr_swap)});
        }
    }

    void firePending()
    {
        auto const pending = pending_ams_;
        pending_ams_       = pending_ams::none;
        if (!sm_.template is<pe::state::pe_snk_ready>()) {
            return;
        }
        switch (pending) {
        case pending_ams::pr_swap:
            sm_.process(pe::event::send_pr_swap{makeControl(control_message_type::pr_swap)});
            break;
        case pending_ams::dr_swap:
            sm_.process(pe::event::send_dr_swap{makeControl(control_message_type::dr_swap)});
            break;
        case pending_ams::none: break;
        }
    }

    // Invoked by the pumped timer after a timeout-driven transition: a
    // retry landing in its gate under SinkTxOk fires right away
    void afterTimeout()
    {
        if (sinkTxAllows()) {
            fireGated();
        }
    }

    // A transient state left standing after its trigger was processed
    // is advanced here (the spec chains them without further input)
    void advanceTransients()
    {
        bool transient = sm_.template is<pe::state::pe_snk_dr_swap_change>();
        if constexpr (vconn_capable) { // else the states are filtered out
            transient = transient || sm_.template is<pe::state::pe_snk_vcs_partner_on>() ||
                        sm_.template is<pe::state::pe_snk_vcs_ps_rdy_sent>();
        }
        if (transient) {
            sm_.process(pe::event::swap_done{});
        }
    }

    std::uint16_t makeHeader(std::uint8_t message_type, std::uint8_t data_objects) const
    {
        return pd_header{.message_type     = message_type,
                         .port_data_role   = sm_.template context<pe::pe_context>().data,
                         .revision         = pd_revision::rev_3_x,
                         .port_power_role  = power_role::sink,
                         .num_data_objects = data_objects}
            .encode();
    }

    pd_message makeControl(control_message_type type) const
    {
        return {.sop = sop_type::sop, .header = makeHeader(static_cast<std::uint8_t>(type), 0)};
    }

    static void putObject(pd_message& message, std::uint32_t object)
    {
        auto const offset = message.payload_size;
        message.payload[offset + 0] = static_cast<std::uint8_t>(object);
        message.payload[offset + 1] = static_cast<std::uint8_t>(object >> 8u);
        message.payload[offset + 2] = static_cast<std::uint8_t>(object >> 16u);
        message.payload[offset + 3] = static_cast<std::uint8_t>(object >> 24u);
        message.payload_size += 4;
    }

    static std::uint32_t getObject(pd_message const& message, std::uint8_t index)
    {
        auto const offset = static_cast<std::size_t>(index) * 4;
        return static_cast<std::uint32_t>(message.payload[offset + 0]) |
               (static_cast<std::uint32_t>(message.payload[offset + 1]) << 8u) |
               (static_cast<std::uint32_t>(message.payload[offset + 2]) << 16u) |
               (static_cast<std::uint32_t>(message.payload[offset + 3]) << 24u);
    }

    void dispatch(pd_message const& message)
    {
        if (bist_test_data_) {
            return; // BIST test data mode: deaf until hard reset/detach
        }
        auto const header = pd_header::decode(message.header);
        if (header.extended) {
            auto const extended = extended_header::decode(
                static_cast<std::uint16_t>(message.payload[0]) |
                (static_cast<std::uint16_t>(message.payload[1]) << 8u));
            if (extended.chunked) {
                sm_.process(pe::event::chunked_message{
                    makeControl(control_message_type::not_supported)});
            } else {
                sm_.process(pe::event::unsupported{
                    makeControl(control_message_type::not_supported)});
            }
            return;
        }
        if (isData(header, data_message_type::source_capabilities)) {
            evaluate(message, header.num_data_objects);
        } else if (isData(header, data_message_type::bist)) {
            if (header.num_data_objects >= 1) {
                enterBist(getObject(message, 0));
            }
        } else if (isControl(header, control_message_type::accept)) {
            sm_.process(pe::event::accept{});
            advanceTransients();
        } else if (isControl(header, control_message_type::reject)) {
            sm_.process(pe::event::reject{});
        } else if (isControl(header, control_message_type::wait)) {
            sm_.process(pe::event::wait{});
        } else if (isControl(header, control_message_type::ps_rdy)) {
            sm_.process(pe::event::ps_rdy{});
            advanceTransients(); // a VCONN hand-off completion
        } else if (isControl(header, control_message_type::dr_swap)) {
            answerSwap<data_role>(pe::event::dr_swap_accepted{
                makeControl(control_message_type::accept)});
        } else if (isControl(header, control_message_type::pr_swap)) {
            answerSwap<power_role>(pe::event::pr_swap_accepted{
                makeControl(control_message_type::accept)});
        } else if (isControl(header, control_message_type::vconn_swap)) {
            answerSwap<vconn_source_role>(pe::event::vconn_swap_accepted{
                makeControl(control_message_type::accept)});
        } else if (isControl(header, control_message_type::get_sink_cap)) {
            sendSinkCapabilities();
        } else if (isControl(header, control_message_type::get_source_cap)) {
            sendSourceCapabilities();
        } else if (isControl(header, control_message_type::soft_reset)) {
            sm_.process(pe::event::soft_reset_received{
                makeControl(control_message_type::accept)});
        } else if (!isControl(header, control_message_type::good_crc) &&
                   !isControl(header, control_message_type::ping)) {
            // answered from Ready only; ignored while negotiating
            sm_.process(pe::event::unsupported{makeControl(control_message_type::not_supported)});
        }
    }

    // The partner asks for a role swap, arbitrated by the injected
    // policy's optional allowSwap(role) - consulted with the role this
    // port would take. A policy without one keeps the non-DRP answer
    // (Not_Supported); a refusal answers Reject; a request outside
    // Ready is discarded (an AMS is running)
    template<typename ROLE, typename ACCEPTED>
    void answerSwap(ACCEPTED const& accepted)
    {
        if constexpr (requires(ROLE role) {
                          { policy_.allowSwap(role) } -> std::convertible_to<bool>;
                      }) {
            if (policy_.allowSwap(swapTarget<ROLE>())) {
                sm_.process(accepted);
            } else {
                sm_.process(
                    pe::event::unsupported{makeControl(control_message_type::reject)});
            }
        } else {
            sm_.process(
                pe::event::unsupported{makeControl(control_message_type::not_supported)});
        }
    }

    template<typename ROLE>
    ROLE swapTarget() const
    {
        if constexpr (std::is_same_v<ROLE, power_role>) {
            return power_role::source; // a sink swaps to sourcing
        } else if constexpr (std::is_same_v<ROLE, vconn_source_role>) {
            return {}; // the arbitration decides on the port's vconn role
        } else {
            auto const data = sm_.template context<pe::pe_context>().data;
            return data == data_role::ufp ? data_role::dfp : data_role::ufp;
        }
    }

    // BIST entry, honored only under an explicit vSafe5V contract
    // (spec): Carrier Mode 2 transmits the test carrier for
    // tBISTContMode; Test Data silences the engine until a hard reset
    // or detach while the TCPC keeps answering GoodCRC
    void enterBist(std::uint32_t bdo)
    {
        auto const& context = sm_.template context<pe::pe_context>();
        if (!context.explicit_contract || context.request.voltage != pe::v_safe_5v) {
            return;
        }
        switch (bist::modeOf(bdo)) {
        case bist::mode::carrier_mode_2:
            if (sm_.process(pe::event::bist_carrier{})) {
                tcpc_.transmit(transmit_signal::bist_carrier_mode_2);
            }
            break;
        case bist::mode::test_data:
            if (sm_.template is<pe::state::pe_snk_ready>()) {
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
            objects[index] = getObject(message, index);
        }
        auto const offered = std::span<std::uint32_t const>{objects.data(), n};

        auto terms = policy_.select(offered, capabilities_);
        if (!terms && n > 0) {
            terms = contract_request{.position          = 1,
                                     .voltage           = pdo::fixedVoltage(objects[0]),
                                     .operating_current = pdo::fixedMaxCurrent(objects[0]),
                                     .maximum_current   = pdo::fixedMaxCurrent(objects[0]),
                                     .mismatch          = true};
        }
        if (!terms) {
            return;
        }

        pd_message request{.sop    = sop_type::sop,
                           .header = makeHeader(
                               static_cast<std::uint8_t>(data_message_type::request), 1)};
        putObject(request, pdo::makeFixedRequest(terms->position, terms->operating_current,
                                                 terms->maximum_current, terms->mismatch));
        sm_.process(pe::event::capabilities_evaluated{request, *terms});
    }

    void sendSinkCapabilities()
    {
        auto const n = std::min<std::size_t>(capabilities_.size(), 7);
        pd_message caps{.sop    = sop_type::sop,
                        .header = makeHeader(
                            static_cast<std::uint8_t>(data_message_type::sink_capabilities),
                            static_cast<std::uint8_t>(n))};
        for (std::size_t index = 0; index < n; ++index) {
            putObject(caps, pdo::makeFixedSink(capabilities_[index].voltage,
                                               capabilities_[index].current));
        }
        sm_.process(pe::event::send_sink_caps{caps});
    }

    // PE_DR_SNK_Give_Source_Cap; a sink-only port answers Not_Supported
    void sendSourceCapabilities()
    {
        if (source_capabilities_.empty()) {
            sm_.process(pe::event::unsupported{makeControl(control_message_type::not_supported)});
            return;
        }
        auto const n = std::min<std::size_t>(source_capabilities_.size(), 7);
        pd_message caps{
            .sop    = sop_type::sop,
            .header = makeHeader(static_cast<std::uint8_t>(data_message_type::source_capabilities),
                                 static_cast<std::uint8_t>(n))};
        for (std::size_t index = 0; index < n; ++index) {
            putObject(caps, source_capabilities_[index]);
        }
        sm_.process(pe::event::send_source_caps{caps});
    }

    TCPC& tcpc_;
    std::span<sink_capability const> capabilities_;
    std::span<std::uint32_t const> source_capabilities_{}; // empty: not a DRP
    POLICY& policy_;
    enum class pending_ams : std::uint8_t { none, pr_swap, dr_swap };

    // The optional VCONN feature follows the injected policy: without
    // its arbitration hook, the VCS states are filtered from the table
    static constexpr bool vconn_capable = requires(POLICY p) {
        { p.allowSwap(vconn_source_role{}) } -> std::convertible_to<bool>;
    };

    void (*idle_hook_)(void*) = nullptr;
    void* idle_context_       = nullptr;
    bool bist_test_data_      = false;
    bool sink_tx_ok_          = true; // last Rp seen (SinkTxOk/NG)
    pending_ams pending_ams_  = pending_ams::none;
    PrlPort port_{*this};
    // both timers pumped: the PRL's HardResetCompleteTimer also drives
    // transitions whose port requests the facade must execute
    PumpedTimer prl_pumped_;
    ProtocolLayer<TCPC, PumpedTimer, PrlPort> prl_; // also an observer of sm_
    PumpedTimer pumped_;
    fsm::timed<PumpedTimer&> timed_;
    fsm::state_machine<pe::sink_table_for<vconn_capable>, fsm::timed<PumpedTimer&>,
                       ProtocolLayer<TCPC, PumpedTimer, PrlPort>, OBSERVERs...>
        sm_;
};

} // namespace usbc
