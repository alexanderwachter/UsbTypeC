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
 * Deviations kept for later: Wait is treated as Reject (no
 * tSinkRequest retry), GotoMin and swaps answer Not_Supported, there
 * is no HardResetCounter - the sink retries indefinitely - and
 * Discovery does not verify VBUS itself: start() implies VBUS present,
 * and after a hard reset the SinkWaitCapTimer absorbs the VBUS gap.
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

namespace usbc {

// One contract the sink can accept, and the Sink_Capabilities content
struct sink_capability {
    millivolt voltage;
    milliamp current;
};

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

inline constexpr auto t_sink_wait_cap = std::chrono::milliseconds{465}; // 310 ms - 620 ms
inline constexpr auto t_ps_transition = std::chrono::milliseconds{500}; // 450 ms - 550 ms

inline constexpr milliamp i_snk_stdby = 500; // iSnkStdby at any voltage

struct pe_context {
    contract_request pending{}; // proposed by the last Request
    contract_request request{}; // accepted by the source
    pd_message reply{};         // pending Not_Supported answer
    bool explicit_contract = false;
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
struct reject {};
struct wait {};
struct ps_rdy {};
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

    explicit pe_snk_startup(pe_context& ctx) : context(ctx) { context = {}; }
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

// The engine evaluates through the injected policy and advances with
// capabilities_evaluated
struct pe_snk_evaluate_capability {
    static constexpr power_level power          = power_level::default_power;
    static constexpr pd_status pd               = pd_status::connected;
    static constexpr std::string_view dot_note  = specNote(power, pd);

    explicit pe_snk_evaluate_capability(pe_context& ctx) : context(ctx) {}
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
        context.pending = event.terms;
    }
    explicit pe_snk_select_capability(pe_context& ctx) : context(ctx) {}

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
                                           data_role::ufp);
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

    explicit pe_snk_hard_reset(pe_context& ctx) : context(ctx) {}
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

    explicit pe_snk_transition_to_default(pe_context& ctx) : context(ctx) { context = {}; }

    pe_context& context;
};

} // namespace state

struct has_explicit_contract {
    static bool check(state::pe_snk_select_capability const& state)
    {
        return state.context.explicit_contract;
    }
};

using sink_table = fsm::transition_table<
    fsm::initial<state::pe_snk_startup>,
    fsm::transition<fsm::from<state::pe_snk_startup>, fsm::on<event::started>,
                    fsm::to<state::pe_snk_discovery>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::vbus_removed>,
                    fsm::to<state::pe_snk_startup>>,
    fsm::transition<fsm::from<state::pe_snk_discovery>, fsm::on<event::vbus_present>,
                    fsm::to<state::pe_snk_wait_for_capabilities>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_hard_reset>>,
    fsm::transition<fsm::from<state::pe_snk_wait_for_capabilities>,
                    fsm::on<event::source_capabilities>,
                    fsm::to<state::pe_snk_evaluate_capability>>,
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
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_ready>, fsm::guard<has_explicit_contract>>,
    fsm::transition<fsm::from<state::pe_snk_select_capability>, fsm::on<event::wait>,
                    fsm::to<state::pe_snk_wait_for_capabilities>>,
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
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::unsupported>,
                    fsm::to<state::pe_snk_send_not_supported>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::message_sent>,
                    fsm::to<state::pe_snk_ready>>,
    fsm::transition<fsm::from<state::pe_snk_send_not_supported>, fsm::on<event::protocol_error>,
                    fsm::to<state::pe_snk_send_soft_reset>>,
    fsm::transition<fsm::from<state::pe_snk_ready>, fsm::on<event::chunked_message>,
                    fsm::to<state::pe_snk_chunk_received>>,
    fsm::transition<fsm::from<state::pe_snk_chunk_received>, fsm::on<fsm::timeout>,
                    fsm::to<state::pe_snk_send_not_supported>>,
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
          prl_(tcpc, prl_timer, port_),
          timed_(pe_timer),
          sm_(timed_, prl_, observers...)
    {
        tcpc_.setMessageHeaderInfo(
            {power_role::sink, data_role::ufp, pd_revision::rev_3_x});
        tcpc_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
        sm_.process(pe::event::started{}); // rest in Discovery until VBUS
    }

    // The Type-C layer reports attach: for a sink, VBUS is present
    void vbusPresent() { sm_.process(pe::event::vbus_present{}); }

    // ... and detach: negotiation state is gone, back to Discovery
    void vbusRemoved()
    {
        sm_.process(pe::event::vbus_removed{});
        sm_.process(pe::event::started{});
    }

    // Feed the TCPC's PD alerts (message/transmit/hard reset bits)
    void onAlert(alert_status alerts) { prl_.onAlert(alerts); }

private:
    // The protocol layer's client, forwarding into the engine
    struct PrlPort {
        SinkPolicyEngine& pe;

        void onMessage(pd_message const& message) { pe.dispatch(message); }
        void onTxDone() { pe.sm_.process(pe::event::message_sent{}); }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { pe.sm_.process(pe::event::protocol_error{}); }
        void onHardReset()
        {
            pe.sm_.process(pe::event::hard_reset_received{});
            pe.restart();
        }
        void onHardResetSent()
        {
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

    std::uint16_t makeHeader(std::uint8_t message_type, std::uint8_t data_objects) const
    {
        return pd_header{.message_type     = message_type,
                         .port_data_role   = data_role::ufp,
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
        } else if (isControl(header, control_message_type::accept)) {
            sm_.process(pe::event::accept{});
        } else if (isControl(header, control_message_type::reject)) {
            sm_.process(pe::event::reject{});
        } else if (isControl(header, control_message_type::wait)) {
            sm_.process(pe::event::wait{});
        } else if (isControl(header, control_message_type::ps_rdy)) {
            sm_.process(pe::event::ps_rdy{});
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

    TCPC& tcpc_;
    std::span<sink_capability const> capabilities_;
    POLICY& policy_;
    PrlPort port_{*this};
    ProtocolLayer<TCPC, TIMER, PrlPort> prl_; // also an observer of sm_
    fsm::timed<TIMER&> timed_;
    fsm::state_machine<pe::sink_table, fsm::timed<TIMER&>, ProtocolLayer<TCPC, TIMER, PrlPort>,
                       OBSERVERs...>
        sm_;
};

} // namespace usbc
