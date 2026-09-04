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
 * GotoMin are not sent, Get_Sink_Cap answers Not_Supported (source
 * only), the hard-reset VBUS off/on cycle is left to the supply
 * restore, lost regulation (at_target = false) is not yet handled, and
 * there is no NoResponseTimer/HardResetCounter. The supply-transition
 * choreography of PE_SRC_Transition_Supply is spelled out as _delay,
 * _settle and _ps_rdy sub-states so the diagram shows it.
 *
 * Integration: mirror image of the sink engine. The ProtocolLayer is
 * an observer of the machine (prl_action commands, txMessage()
 * transmissions); the application injects its own observers - derive
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
inline constexpr auto t_src_pr_swap_wait      = std::chrono::milliseconds{150}; // tPRSwapWait
inline constexpr auto t_src_dr_swap_wait      = std::chrono::milliseconds{150}; // tDRSwapWait

inline constexpr std::uint8_t n_caps_count = spec::n_caps_count;

// Engine-directed command: entering a state carrying it as src_action
// makes the engine transmit the Source_Capabilities message
struct send_capabilities_action {
    static constexpr std::string_view note = "sends Source_Capabilities";
    constexpr bool operator==(send_capabilities_action const&) const = default;
};

struct src_context {
    std::uint8_t caps_counter = 0; // CapsCounter
    bool attached              = false;
    bool pd_connected          = false; // a Source_Capabilities got its GoodCRC
    bool explicit_contract     = false;
    data_role data = data_role::dfp; // flipped by an agreed DR_Swap
    supply_target target{};
    pd_message reply{};
};

namespace event {

struct attached {};
struct detached {};
struct request {};
struct request_ok {
    pd_message accept;
    supply_target target;
};
struct request_bad {
    pd_message reject;
};
struct get_source_caps {};
struct give_sink_caps { // a DRP asked for its sink-role caps
    pd_message message;
};
struct supply_settled {};

} // namespace event

namespace state {

// PE_SRC_Startup: the protocol layer reset is mandatory; the default
// power restore covers the detach entry (after a hard reset,
// Transition_to_default already restored and suppression elides it)
struct pe_src_startup {
    static constexpr prl::reset_action prl_action{};
    static constexpr restore_default_action power_action{};
    static constexpr power_level power           = power_level::default_power;
    static constexpr pd_status pd                = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    explicit pe_src_startup(src_context& ctx) : context(ctx) { context = {}; }
    src_context& context;
};

// Advertises the capabilities and waits for the Request; the timer
// includes the transmission (deviation, see the file comment)
struct pe_src_send_capabilities {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr send_capabilities_action src_action{};
    static constexpr power_level power           = power_level::default_power;
    static constexpr pd_status pd                = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = send_capabilities_action::note;

    pe_src_send_capabilities(event::attached const&, src_context& ctx)
        : pe_src_send_capabilities(ctx)
    {
        context.attached = true;
    }
    explicit pe_src_send_capabilities(src_context& ctx) : context(ctx)
    {
        ++context.caps_counter;
    }

    // the GoodCRC on the capabilities means a PD sink is present
    void handle(pe::event::message_sent const&) { context.pd_connected = true; }

    src_context& context;
};

// Waits SourceCapabilityTimer between advertisement attempts
struct pe_src_discovery {
    static constexpr auto timeout = t_typec_send_source_cap; // SourceCapabilityTimer
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_discovery(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// nCapsCount advertisements went unanswered: the sink speaks no PD,
// the port stays a plain Type-C source until detach
struct pe_src_disabled {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::not_connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_disabled(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// The engine evaluates the Request through the injected policy and
// advances with request_ok or request_bad
struct pe_src_negotiate_capability {
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_negotiate_capability(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// PE_SRC_Transition_Supply: sends the Accept; the _delay, _settle and
// _ps_rdy sub-states spell out the supply choreography
struct pe_src_transition_supply {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_transition_supply(event::request_ok const& event, src_context& ctx) : context(ctx)
    {
        context.target = event.target;
        context.reply  = event.accept;
    }
    explicit pe_src_transition_supply(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

// The spec's tSrcTransition wait between the Accept and the change
struct pe_src_transition_supply_delay {
    static constexpr auto timeout = t_src_transition; // tSrcTransition
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_transition_supply_delay(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// Commands the supply to the new operating point and waits for the
// settled callback
struct pe_src_transition_supply_settle {
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "programs the supply";

    explicit pe_src_transition_supply_settle(src_context& ctx) : context(ctx) {}

    supply_target supplyTarget() const { return context.target; }

    src_context& context;
};

// The supply is at the target: PS_RDY tells the sink to draw
struct pe_src_transition_supply_ps_rdy {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_transition_supply_ps_rdy(src_context& ctx) : context(ctx)
    {
        context.reply = makeControlMessage(control_message_type::ps_rdy, power_role::source,
                                           context.data);
    }

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

struct pe_src_ready {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_ready(src_context& ctx) : context(ctx)
    {
        context.explicit_contract = true;
        context.pd_connected      = true;
    }

    active_contract report() const
    {
        return {context.target.voltage, context.target.current};
    }

    src_context& context;
};

// PE_SRC_Capability_Response: the policy refused, the Reject goes out
struct pe_src_capability_response {
    static constexpr power_level power          = power_level::contract_or_default;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_capability_response(event::request_bad const& event, src_context& ctx) : context(ctx)
    {
        context.reply = event.reject;
    }
    explicit pe_src_capability_response(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

// PE_DR_SRC_Give_Sink_Cap: a DRP answers Get_Sink_Cap with its
// sink-role capabilities, then returns to Ready
struct pe_dr_src_give_sink_cap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_dr_src_give_sink_cap(event::give_sink_caps const& event, src_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit pe_dr_src_give_sink_cap(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    src_context& context;

private:
    pd_message message_{};
};

// PE_SRC_Send_Not_Supported: answers a message the source does not
// support, then returns to Ready
struct pe_src_send_not_supported {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_send_not_supported(pe::event::unsupported const& event, src_context& ctx)
        : context(ctx)
    {
        context.reply = event.reply;
    }
    explicit pe_src_send_not_supported(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

// PE_SRC_Chunk_Received: a non-chunking device lets the sender run
// into its chunking timeout before answering Not_Supported
struct pe_src_chunk_received {
    static constexpr auto timeout = t_chunking_not_supported; // ChunkingNotSupportedTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_chunk_received(pe::event::chunked_message const& event, src_context& ctx)
        : context(ctx)
    {
        context.reply = event.reply;
    }
    explicit pe_src_chunk_received(src_context& ctx) : context(ctx) {}

    src_context& context;
};

// --- role swap messaging (PE_DRS / PE_PRS, source side) ----------------------

// PE_DRS_DFP_UFP/UFP_DFP_Send_Swap: our DR_Swap is out; no answer
// within tSenderResponse means the partner ignored it - stay Ready
struct pe_src_send_dr_swap {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends DR_Swap";

    pe_src_send_dr_swap(pe::event::send_dr_swap const& event, src_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    // re-entry from the tDRSwapWait retry rebuilds the request
    explicit pe_src_send_dr_swap(src_context& ctx)
        : context(ctx), message_(makeControlMessage(control_message_type::dr_swap,
                                                    power_role::source, ctx.data))
    {
    }

    pd_message const& txMessage() const { return message_; }

    src_context& context;

private:
    pd_message message_{};
};

// PE_DRS_*_Accept_Swap: the partner's DR_Swap passed the arbitration
struct pe_src_accept_dr_swap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_src_accept_dr_swap(pe::event::dr_swap_accepted const& event, src_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_src_accept_dr_swap(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    src_context& context;

private:
    pd_message message_{};
};

// PE_DRS_*_Change_to_*: the agreed swap flips the data role; the
// report lets the port update the TCPC header and the Type-C context
struct pe_src_dr_swap_change {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "flips the data role";

    pe_src_dr_swap_change(pe::event::accept const&, src_context& ctx)
        : pe_src_dr_swap_change(ctx)
    {
    }
    pe_src_dr_swap_change(pe::event::message_sent const&, src_context& ctx)
        : pe_src_dr_swap_change(ctx)
    {
    }
    explicit pe_src_dr_swap_change(src_context& ctx) : context(ctx)
    {
        context.data = context.data == data_role::ufp ? data_role::dfp : data_role::ufp;
    }

    data_role_changed swapReport() const { return {context.data}; }

    src_context& context;
};

// PE_PRS_SRC_SNK_Send_Swap: our PR_Swap is out
struct pe_src_send_pr_swap {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PR_Swap";

    pe_src_send_pr_swap(pe::event::send_pr_swap const& event, src_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    // re-entry from the tPRSwapWait retry rebuilds the request
    explicit pe_src_send_pr_swap(src_context& ctx)
        : context(ctx), message_(makeControlMessage(control_message_type::pr_swap,
                                                    power_role::source, ctx.data))
    {
    }

    pd_message const& txMessage() const { return message_; }

    src_context& context;

private:
    pd_message message_{};
};

// PE_PRS_SRC_SNK_Accept_Swap: the partner's PR_Swap passed arbitration
struct pe_src_accept_pr_swap {
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends Accept";

    pe_src_accept_pr_swap(pe::event::pr_swap_accepted const& event, src_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit pe_src_accept_pr_swap(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    src_context& context;

private:
    pd_message message_{};
};

// The partner answered Wait: the swap request is retried after the
// spec's pause (still Ready, spec-wise)
struct pe_src_dr_swap_wait {
    static constexpr auto timeout = t_src_dr_swap_wait; // tDRSwapWait
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_dr_swap_wait(src_context& ctx) : context(ctx) {}
    src_context& context;
};

struct pe_src_pr_swap_wait {
    static constexpr auto timeout = t_src_pr_swap_wait; // tPRSwapWait
    static constexpr power_level power          = power_level::explicit_contract;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_pr_swap_wait(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// PE_PRS_SRC_SNK_Transition_to_off, the spec's tSrcTransition wait
// between the agreement and removing power
struct pe_src_swap_transition_to_off {
    static constexpr auto timeout = t_src_transition; // tSrcTransition
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    pe_src_swap_transition_to_off(pe::event::accept const&, src_context& ctx)
        : pe_src_swap_transition_to_off(ctx)
    {
    }
    pe_src_swap_transition_to_off(pe::event::message_sent const&, src_context& ctx)
        : pe_src_swap_transition_to_off(ctx)
    {
    }
    explicit pe_src_swap_transition_to_off(src_context& ctx) : context(ctx) {}

    src_context& context;
};

// ... the supply is commanded off and its settled report awaited
struct pe_src_swap_supply_off {
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "turns the supply off";

    explicit pe_src_swap_supply_off(src_context& ctx) : context(ctx) {}

    supply_target supplyTarget() const { return {0, 0}; }

    src_context& context;
};

// PE_PRS_SRC_SNK_Assert_Rd: VBUS is off - the port flips its
// termination now; the sink engine then announces our PS_RDY
struct pe_src_swap_assert_rd {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_swap_assert_rd(src_context& ctx) : context(ctx) {}

    assert_new_role swapReport() const { return {power_role::sink}; }

    src_context& context;
};

// PE_PRS_SNK_SRC_Source_on, this engine's half: the port was the sink
// and asserted Rp - VBUS is driven to vSafe5V first
struct pe_src_swap_source_on {
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = "drives VBUS to vSafe5V";

    pe_src_swap_source_on(pe::event::attached_swap const& event, src_context& ctx)
        : context(ctx)
    {
        context.attached     = true;
        context.pd_connected = true;       // the swap was PD-negotiated
        context.data         = event.role; // a power swap preserves the data role
    }
    explicit pe_src_swap_source_on(src_context& ctx) : context(ctx) {}

    supply_target supplyTarget() const { return {v_safe_5v, i_default_current}; }

    src_context& context;
};

// ... at vSafe5V the PS_RDY completes the partner's wait
struct pe_src_swap_source_on_ps_rdy {
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);
    static constexpr std::string_view dot_action = "sends PS_RDY";

    explicit pe_src_swap_source_on_ps_rdy(src_context& ctx) : context(ctx)
    {
        context.reply = makeControlMessage(control_message_type::ps_rdy, power_role::source,
                                           context.data);
    }

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

// SwapSourceStartTimer: the new source pauses before its first
// Source_Capabilities
struct pe_src_swap_source_start {
    static constexpr auto timeout = t_source_start; // SwapSourceStartTimer
    static constexpr power_level power          = power_level::transition;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_src_swap_source_start(src_context& ctx) : context(ctx)
    {
        context.caps_counter = 0; // the advertisement starts over
    }

    src_context& context;
};

// Accepts a received Soft_Reset; the protocol layer resets before the
// Accept goes out (guaranteed hook order), then re-advertises
struct pe_src_soft_reset {
    static constexpr prl::reset_action prl_action{};
    static constexpr power_level power           = power_level::contract_or_default;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    pe_src_soft_reset(pe::event::soft_reset_received const& event, src_context& ctx)
        : context(ctx)
    {
        context.reply = event.accept;
    }
    explicit pe_src_soft_reset(src_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

// PE_SRC_Send_Soft_Reset: protocol errors first try a soft reset
struct pe_src_send_soft_reset {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer
    static constexpr prl::reset_action prl_action{};
    static constexpr power_level power           = power_level::contract_or_default;
    static constexpr pd_status pd                = pd_status::connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::reset_action::note;

    explicit pe_src_send_soft_reset(src_context& ctx) : context(ctx)
    {
        context.reply = makeControlMessage(control_message_type::soft_reset, power_role::source,
                                           context.data);
    }

    pd_message const& txMessage() const { return context.reply; }

    src_context& context;
};

struct pe_src_hard_reset {
    static constexpr prl::hard_reset_action prl_action{};
    static constexpr power_level power           = power_level::contract_or_default;
    static constexpr pd_status pd                = pd_status::connected_or_not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action = prl::hard_reset_action::note;

    explicit pe_src_hard_reset(src_context& ctx) : context(ctx) {}
    src_context& context;
};

// PE_SRC_Transition_to_default: supply back to vSafe5V, tSrcRecover,
// then advertise again (or rest in Startup after a detach)
struct pe_src_transition_to_default {
    static constexpr auto timeout = t_src_recover; // tSrcRecover
    static constexpr prl::reset_action prl_action{};
    static constexpr restore_default_action power_action{};
    static constexpr power_level power           = power_level::transition;
    static constexpr pd_status pd                = pd_status::not_connected;
    static constexpr std::string_view dot_note   = specNote(power, pd);
    static constexpr std::string_view dot_action =
        "resets the protocol layer, restores default power";

    explicit pe_src_transition_to_default(src_context& ctx) : context(ctx)
    {
        context.caps_counter      = 0;
        context.pd_connected      = false;
        context.explicit_contract = false;
        context.target            = {};
    }

    src_context& context;
};

} // namespace state

struct caps_count_allows {
    static bool check(state::pe_src_discovery const& state)
    {
        return state.context.caps_counter <= n_caps_count;
    }
};

struct pd_was_connected {
    static bool check(state::pe_src_send_capabilities const& state)
    {
        return state.context.pd_connected;
    }
};

struct src_explicit_contract {
    static bool check(state::pe_src_capability_response const& state)
    {
        return state.context.explicit_contract;
    }
};

struct still_attached {
    static bool check(state::pe_src_transition_to_default const& state)
    {
        return state.context.attached;
    }
};

// The spec timer range of every timed state, checked against the table
using source_timer_ranges = mtl::typelist<
    fsm::timed_by<state::pe_src_send_capabilities, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_discovery, spec::t_typec_send_source_cap>,
    fsm::timed_by<state::pe_src_transition_supply_delay, spec::t_src_transition>,
    fsm::timed_by<state::pe_src_chunk_received, spec::t_chunking_not_supported>,
    fsm::timed_by<state::pe_src_send_soft_reset, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_transition_to_default, spec::t_src_recover>,
    fsm::timed_by<state::pe_src_send_dr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_send_pr_swap, spec::t_sender_response>,
    fsm::timed_by<state::pe_src_swap_transition_to_off, spec::t_src_transition>,
    fsm::timed_by<state::pe_src_swap_source_start, spec::t_swap_source_start>,
    fsm::timed_by<state::pe_src_dr_swap_wait, spec::t_dr_swap_wait>,
    fsm::timed_by<state::pe_src_pr_swap_wait, spec::t_pr_swap_wait>>;

using source_table = fsm::transition_table<
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
    fsm::transition<fsm::from<state::pe_src_send_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_hard_reset>, fsm::guard<pd_was_connected>>,
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
                    fsm::guard<src_explicit_contract>>,
    fsm::transition<fsm::from<state::pe_src_capability_response>,
                    fsm::on<pe::event::message_sent>, fsm::to<state::pe_src_hard_reset>>,
    fsm::transition<fsm::from<state::pe_src_capability_response>,
                    fsm::on<pe::event::protocol_error>, fsm::to<state::pe_src_hard_reset>>,
    // Ready serves the sink
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::request>,
                    fsm::to<state::pe_src_negotiate_capability>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::get_source_caps>,
                    fsm::to<state::pe_src_send_capabilities>>,
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<event::give_sink_caps>,
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
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::chunked_message>,
                    fsm::to<state::pe_src_chunk_received>>,
    fsm::transition<fsm::from<state::pe_src_chunk_received>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_not_supported>>,
    // DR_Swap: sent from Ready, or accepted there; both sides flip on
    // the agreement, an ignored request falls back to Ready
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_dr_swap>,
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
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::dr_swap_accepted>,
                    fsm::to<state::pe_src_accept_dr_swap>>,
    fsm::transition<fsm::from<state::pe_src_accept_dr_swap>, fsm::on<pe::event::message_sent>,
                    fsm::to<state::pe_src_dr_swap_change>>,
    fsm::transition<fsm::from<state::pe_src_accept_dr_swap>, fsm::on<pe::event::protocol_error>,
                    fsm::to<state::pe_src_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_src_dr_swap_change>, fsm::on<pe::event::swap_done>,
                    fsm::to<state::pe_src_ready>>,
    // PR_Swap while sourcing: the agreement leads through tSrcTransition
    // into the supply-off wait, then the termination flip
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::send_pr_swap>,
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
    fsm::transition<fsm::from<state::pe_src_ready>, fsm::on<pe::event::pr_swap_accepted>,
                    fsm::to<state::pe_src_accept_pr_swap>>,
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
    // after tSrcRecover: advertise again, or rest in Startup when the
    // sink is gone
    fsm::transition<fsm::from<state::pe_src_transition_to_default>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_send_capabilities>, fsm::guard<still_attached>>,
    fsm::transition<fsm::from<state::pe_src_transition_to_default>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_src_startup>>>;
static_assert(fsm::timeouts_within_bounds_v<source_table, source_timer_ranges>);
static_assert(fsm::all_states_reachable_v<source_table>);

// The member observers behind SourcePower (POWER is
// SourcePower<DERIVED>); injected together as one fsm::observer_group

// Stores the contract terms the Ready state reports
template<typename POWER>
struct src_contract_store : fsm::observing<src_contract_store<POWER>> {
    explicit src_contract_store(POWER& power_ref) : power(power_ref) {}

    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert(concepts::source_power_client<typename POWER::derived_type>,
                      "SourcePower: the derived class must provide onContract(millivolt, "
                      "milliamp) and onContractLost()");
    }

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.report()))
    {
        return state.report();
    }
    void notifyEntry(active_contract contract) { power.contract_ = contract; }

    POWER& power;
};

// Reports the stored contract exactly when the diagram's Power column
// changes to Explicit Contract
template<typename POWER>
struct src_contract_apply : fsm::observing<src_contract_apply<POWER>> {
    explicit src_contract_apply(POWER& power_ref) : power(power_ref) {}

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

// Reports the contract loss on the states carrying a restore action
template<typename POWER>
struct src_contract_lost : fsm::observing<src_contract_lost<POWER>> {
    explicit src_contract_lost(POWER& power_ref) : power(power_ref) {}

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::power_action)
    {
        return STATE::power_action;
    }
    void notifyEntry(restore_default_action) { power.restoreDefaults(); }

    POWER& power;
};

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
class SourcePower : public fsm::observer_group<pe::src_contract_store<SourcePower<DERIVED>>,
                                               pe::src_contract_apply<SourcePower<DERIVED>>,
                                               pe::src_contract_lost<SourcePower<DERIVED>>> {
public:
    using derived_type = DERIVED;

    // store before apply: the contract terms must be fresh when the
    // power annotation edge fires on the same entry
    SourcePower()
        : fsm::observer_group<pe::src_contract_store<SourcePower>,
                              pe::src_contract_apply<SourcePower>,
                              pe::src_contract_lost<SourcePower>>(store_, apply_, lost_)
    {
    }

private:
    friend pe::src_contract_store<SourcePower>;
    friend pe::src_contract_apply<SourcePower>;
    friend pe::src_contract_lost<SourcePower>;

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

    pe::src_contract_store<SourcePower> store_{*this};
    pe::src_contract_apply<SourcePower> apply_{*this};
    pe::src_contract_lost<SourcePower> lost_{*this};
    pe::active_contract contract_{};
    bool contract_active_ = false;
};

template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER, concepts::source_policy POLICY,
         concepts::source_supply SUPPLY, typename... OBSERVERs>
class SourcePolicyEngine {
public:
    // The observers are injected into the engine's machine after the
    // protocol layer and the supply driver; a SourcePower-derived one
    // supplies the contract notifications
    SourcePolicyEngine(TCPC& tcpc, TIMER& prl_timer, TIMER& pe_timer,
                       std::span<std::uint32_t const> capabilities, POLICY& policy,
                       SUPPLY& supply, OBSERVERs&... observers)
        : tcpc_(tcpc),
          capabilities_(capabilities),
          policy_(policy),
          supply_(supply),
          prl_(tcpc, prl_timer, port_),
          timed_(pe_timer),
          sm_(timed_, prl_, caps_sender_, supply_driver_, observers...)
    {
        tcpc_.setMessageHeaderInfo(
            {power_role::source, data_role::dfp, pd_revision::rev_3_x});
        tcpc_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
        supply_.setReadyCallback(
            [](void* self, bool at_target) {
                auto& engine = *static_cast<SourcePolicyEngine*>(self);
                if (at_target) {
                    engine.sm_.process(pe::event::supply_settled{});
                }
                if (engine.idle_hook_ != nullptr) {
                    engine.idle_hook_(engine.idle_context_);
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
        prl_.resetRevision();
        sm_.process(pe::event::detached{});
    }

    // Feed the TCPC's PD alerts (message/transmit/hard reset bits)
    void onAlert(alert_status alerts) { prl_.onAlert(alerts); }

    // --- DRP integration: PD-negotiated role swaps ---------------------------

    // Sends the PR_Swap / DR_Swap; false while not Ready under an
    // explicit contract (the spec allows swaps only there)
    bool requestPowerSwap()
    {
        return sm_.process(
            pe::event::send_pr_swap{makeControl(control_message_type::pr_swap)});
    }

    bool requestDataSwap()
    {
        return sm_.process(
            pe::event::send_dr_swap{makeControl(control_message_type::dr_swap)});
    }

    // The port was the sink and asserted Rp mid PR_Swap: drive VBUS to
    // vSafe5V, announce PS_RDY, pause tSwapSourceStart, then advertise.
    // The event carries the preserved data role (the entered state
    // seeds the context with it); the negotiated revision holds for
    // the connection and is handed over from the retiring engine
    void attachedAfterSwap(data_role role, pd_revision revision)
    {
        prl_.seedRevision(revision);
        sm_.process(pe::event::attached_swap{role});
    }

    // The revision the protocol layer negotiated with this partner
    pd_revision negotiatedRevision() const { return prl_.revision(); }

    // A DRP announces its sink-role capabilities: Get_Sink_Cap is
    // answered with them instead of Not_Supported
    void provideSinkCapabilities(std::span<sink_capability const> capabilities)
    {
        sink_capabilities_ = capabilities;
    }

    // The facade's deferred port actions run through this hook once an
    // engine-internal event source (the supply settle callback) is done
    // processing - the machines are idle then
    void setIdleHook(void (*hook)(void*), void* hook_context)
    {
        idle_hook_ = hook;
        idle_context_ = hook_context;
    }

private:
    // The protocol layer's client, forwarding into the engine
    struct PrlPort {
        SourcePolicyEngine& pe;

        void onMessage(pd_message const& message) { pe.dispatch(message); }
        // adopted revision: the TCPC's GoodCRC header must follow
        void onRevision(pd_revision revision)
        {
            pe.tcpc_.setMessageHeaderInfo(
                {power_role::source, pe.sm_.template context<pe::src_context>().data,
                 revision});
        }
        void onTxDone()
        {
            pe.sm_.process(pe::event::message_sent{});
            pe.advanceTransients();
        }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { pe.sm_.process(pe::event::protocol_error{}); }
        void onHardReset() { pe.sm_.process(pe::event::hard_reset_received{}); }
        void onHardResetSent() { pe.sm_.process(pe::event::hard_reset_complete{}); }
    };

    // Transmits the Source_Capabilities on the states commanding it
    struct CapsSender : fsm::observing<CapsSender> {
        explicit CapsSender(SourcePolicyEngine& pe_ref) : pe(pe_ref) {}

        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::src_action)
        {
            return STATE::src_action;
        }
        void notifyEntry(pe::send_capabilities_action) { pe.transmitSourceCaps(); }

        SourcePolicyEngine& pe;
    };

    // Drives the source_supply: the settle state's target, and the
    // vSafe5V restore on the states carrying the restore action
    struct SupplyDriver : fsm::observing<SupplyDriver> {
        explicit SupplyDriver(SourcePolicyEngine& pe_ref) : pe(pe_ref) {}

        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.supplyTarget()))
        {
            return state.supplyTarget();
        }
        void notifyEntry(supply_target target)
        {
            pe.supply_.setOutput(target.voltage, target.current);
        }

        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::power_action)
        {
            return STATE::power_action;
        }
        void notifyEntry(pe::restore_default_action)
        {
            pe.supply_.setOutput(pe::v_safe_5v, pe::i_default_current);
        }

        SourcePolicyEngine& pe;
    };

    // A transient state left standing after its trigger was processed
    // is advanced here (the spec chains them without further input)
    void advanceTransients()
    {
        if (sm_.template is<pe::state::pe_src_dr_swap_change>()) {
            sm_.process(pe::event::swap_done{});
        }
    }

    std::uint16_t makeHeader(std::uint8_t message_type, std::uint8_t data_objects) const
    {
        return pd_header{.message_type     = message_type,
                         .port_data_role   = sm_.template context<pe::src_context>().data,
                         .revision         = pd_revision::rev_3_x,
                         .port_power_role  = power_role::source,
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

    // PE_DR_SRC_Give_Sink_Cap; a source-only port answers Not_Supported
    void sendSinkCapabilities()
    {
        if (sink_capabilities_.empty()) {
            sm_.process(pe::event::unsupported{makeControl(control_message_type::not_supported)});
            return;
        }
        auto const n = std::min<std::size_t>(sink_capabilities_.size(), 7);
        pd_message caps{
            .sop    = sop_type::sop,
            .header = makeHeader(static_cast<std::uint8_t>(data_message_type::sink_capabilities),
                                 static_cast<std::uint8_t>(n))};
        for (std::size_t index = 0; index < n; ++index) {
            putObject(caps, pdo::makeFixedSink(sink_capabilities_[index].voltage,
                                               sink_capabilities_[index].current));
        }
        sm_.process(pe::event::give_sink_caps{caps});
    }

    void transmitSourceCaps()
    {
        auto const n = std::min<std::size_t>(capabilities_.size(), 7);
        pd_message caps{
            .sop    = sop_type::sop,
            .header = makeHeader(static_cast<std::uint8_t>(data_message_type::source_capabilities),
                                 static_cast<std::uint8_t>(n))};
        for (std::size_t index = 0; index < n; ++index) {
            putObject(caps, capabilities_[index]);
        }
        prl_.transmit(caps);
    }

    void dispatch(pd_message const& message)
    {
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
        if (isData(header, data_message_type::request)) {
            negotiate(message);
        } else if (isControl(header, control_message_type::accept)) {
            sm_.process(pe::event::accept{});
            advanceTransients();
        } else if (isControl(header, control_message_type::reject)) {
            sm_.process(pe::event::reject{});
        } else if (isControl(header, control_message_type::wait)) {
            sm_.process(pe::event::wait{});
        } else if (isControl(header, control_message_type::ps_rdy)) {
            // the PS_RDY exchange of a PR_Swap runs on the sink engine
        } else if (isControl(header, control_message_type::dr_swap)) {
            answerSwap<data_role>(pe::event::dr_swap_accepted{
                makeControl(control_message_type::accept)});
        } else if (isControl(header, control_message_type::pr_swap)) {
            answerSwap<power_role>(pe::event::pr_swap_accepted{
                makeControl(control_message_type::accept)});
        } else if (isControl(header, control_message_type::get_source_cap)) {
            sm_.process(pe::event::get_source_caps{});
        } else if (isControl(header, control_message_type::get_sink_cap)) {
            sendSinkCapabilities();
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
            return power_role::sink; // a source swaps to sinking
        } else {
            auto const data = sm_.template context<pe::src_context>().data;
            return data == data_role::ufp ? data_role::dfp : data_role::ufp;
        }
    }

    // PE_SRC_Negotiate_Capability: the policy's verdict advances the
    // machine with request_ok or request_bad
    void negotiate(pd_message const& message)
    {
        if (!sm_.process(pe::event::request{})) {
            return; // not in a state that takes a Request
        }
        auto const rdo = getObject(message, 0);
        if (auto const target = policy_.evaluate(rdo, capabilities_)) {
            sm_.process(pe::event::request_ok{makeControl(control_message_type::accept), *target});
        } else {
            sm_.process(pe::event::request_bad{makeControl(control_message_type::reject)});
        }
    }

    TCPC& tcpc_;
    std::span<std::uint32_t const> capabilities_;
    std::span<sink_capability const> sink_capabilities_{}; // empty: not a DRP
    POLICY& policy_;
    SUPPLY& supply_;
    void (*idle_hook_)(void*) = nullptr;
    void* idle_context_       = nullptr;
    PrlPort port_{*this};
    ProtocolLayer<TCPC, TIMER, PrlPort> prl_; // also an observer of sm_
    fsm::timed<TIMER&> timed_;
    CapsSender caps_sender_{*this};
    SupplyDriver supply_driver_{*this};
    fsm::state_machine<pe::source_table, fsm::timed<TIMER&>, ProtocolLayer<TCPC, TIMER, PrlPort>,
                       CapsSender, SupplyDriver, OBSERVERs...>
        sm_;
};

} // namespace usbc
