/*
 * USB PD sink policy engine (PE_SNK) on top of the protocol layer.
 *
 * The application injects the sink's possible contracts as a span of
 * sink_capability entries (also answered to Get_Sink_Cap) and a policy
 * that selects the contract from the source's capabilities; PowerPolicy
 * below is the power-based default. The engine negotiates: waits for
 * Source_Capabilities (SinkWaitCapTimer), evaluates through the policy,
 * requests (SenderResponseTimer), transitions the sink load to iSnkStdby
 * until PS_RDY (PSTransitionTimer), then applies the contract to the
 * sink_load and reports it to the client. Reject without an explicit
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
 * ProtocolLayer and runs from construction on, resting in
 * PE_SNK_Discovery until VBUS is reported. Feed TCPC alerts into
 * onAlert() and the Type-C layer's attach/detach into vbusPresent()
 * and vbusRemoved() - a sink's attach implies VBUS. After a hard reset
 * the engine waits in Discovery for the Type-C layer to report the
 * returning VBUS. Everything runs in the stack's serialized context;
 * client callbacks may originate from the timer context (mtl timer
 * contract).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Pdo.hpp>
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

template<typename T>
concept pe_sink_client = requires(T client, millivolt voltage, milliamp current) {
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

inline constexpr auto t_sink_wait_cap   = std::chrono::milliseconds{465}; // 310 ms - 620 ms
inline constexpr auto t_sender_response = std::chrono::milliseconds{27};  // 24 ms - 30 ms
inline constexpr auto t_ps_transition   = std::chrono::milliseconds{500}; // 450 ms - 550 ms
inline constexpr auto t_chunking_not_supported = std::chrono::milliseconds{45}; // 40 ms - 50 ms

inline constexpr milliamp i_snk_stdby       = 500; // iSnkStdby at any voltage
inline constexpr milliamp i_default_current = 500; // implicit vSafe5V contract

struct pe_context {
    contract_request request{};
    pd_message reply{}; // pending Not_Supported answer
    bool explicit_contract = false;
};

// Reporter observations; each type selects its notify hook
struct prl_reset_action {
    constexpr bool operator==(prl_reset_action const&) const = default;
};
struct hard_reset_action {
    constexpr bool operator==(hard_reset_action const&) const = default;
};
struct standby_limit {
    millivolt voltage;
};
struct active_contract {
    millivolt voltage;
    milliamp current;
};
struct lost_contract {
    bool lost;
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
struct soft_reset_received {
    pd_message accept;
};
struct unsupported {
    pd_message reply;
};
struct chunked_message {
    pd_message reply;
};
struct accept {};
struct reject {};
struct wait {};
struct ps_rdy {};
struct message_sent {};       // PRL: transmission confirmed
struct protocol_error {};     // PRL: transmission failed
struct hard_reset_complete {};
struct hard_reset_received {};
struct default_level_reached {};

} // namespace event

inline pd_message makeControlMessage(control_message_type type)
{
    return {.sop    = sop_type::sop,
            .header = pd_header{.message_type    = static_cast<std::uint8_t>(type),
                                .port_data_role  = data_role::ufp,
                                .revision        = pd_revision::rev_3_x,
                                .port_power_role = power_role::sink}
                          .encode()};
}

namespace state {

// PE_SNK_Startup: the protocol layer reset here is mandatory
struct pe_snk_startup {
    static constexpr prl_reset_action action{};

    explicit pe_snk_startup(pe_context& ctx) : context(ctx) { context = {}; }
    pe_context& context;
};

// Waits for the Type-C layer to report VBUS
struct pe_snk_discovery {
    explicit pe_snk_discovery(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_wait_for_capabilities {
    static constexpr auto timeout = t_sink_wait_cap; // SinkWaitCapTimer

    explicit pe_snk_wait_for_capabilities(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// The engine evaluates through the injected policy and advances with
// capabilities_evaluated
struct pe_snk_evaluate_capability {
    explicit pe_snk_evaluate_capability(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

struct pe_snk_select_capability {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    pe_snk_select_capability(event::capabilities_evaluated const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
        context.request = event.terms;
    }
    explicit pe_snk_select_capability(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

struct pe_snk_transition_sink {
    static constexpr auto timeout = t_ps_transition; // PSTransitionTimer

    explicit pe_snk_transition_sink(pe_context& ctx) : context(ctx) {}

    standby_limit report() const { return {context.request.voltage}; }

    pe_context& context;
};

struct pe_snk_ready {
    explicit pe_snk_ready(pe_context& ctx) : context(ctx) { context.explicit_contract = true; }

    active_contract report() const
    {
        return {context.request.voltage, context.request.operating_current};
    }

    pe_context& context;
};

struct pe_snk_give_sink_cap {
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
    static constexpr prl_reset_action action{};

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
    static constexpr prl_reset_action action{};

    explicit pe_snk_send_soft_reset(pe_context& ctx) : context(ctx)
    {
        context.reply = makeControlMessage(control_message_type::soft_reset);
    }

    pd_message const& txMessage() const { return context.reply; }

    pe_context& context;
};

struct pe_snk_hard_reset {
    static constexpr hard_reset_action action{};

    explicit pe_snk_hard_reset(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

// PE_SNK_Transition_to_default: back to vSafe5V defaults; the engine
// then advances through Startup and Discovery
struct pe_snk_transition_to_default {
    explicit pe_snk_transition_to_default(pe_context& ctx)
        : context(ctx), lost_(ctx.explicit_contract)
    {
        context = {};
    }

    lost_contract report() const { return {lost_}; }

    pe_context& context;

private:
    bool lost_ = false;
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

} // namespace pe

template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER, concepts::sink_policy POLICY,
         concepts::sink_load LOAD, concepts::pe_sink_client CLIENT>
class SinkPolicyEngine {
public:
    SinkPolicyEngine(TCPC& tcpc, TIMER& prl_timer, TIMER& pe_timer,
                     std::span<sink_capability const> capabilities, POLICY& policy, LOAD& load,
                     CLIENT& client)
        : tcpc_(tcpc),
          capabilities_(capabilities),
          policy_(policy),
          load_(load),
          client_(client),
          prl_(tcpc, prl_timer, port_),
          timed_(pe_timer)
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

    // Transmits a state's txMessage() through the protocol layer
    struct Sender : fsm::observing<Sender> {
        explicit Sender(SinkPolicyEngine& pe_ref) : pe(pe_ref) {}

        static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.txMessage()))
        {
            return state.txMessage();
        }
        void notifyEntry(pd_message const& message) { pe.prl_.transmit(message); }

        SinkPolicyEngine& pe;
    };

    // Applies the reporter observations to the load, client, and PRL
    struct Reporter : fsm::observing<Reporter> {
        explicit Reporter(SinkPolicyEngine& pe_ref) : pe(pe_ref) {}

        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::action)
        {
            return STATE::action;
        }
        void notifyEntry(pe::prl_reset_action) { pe.prl_.reset(sop_type::sop); }
        void notifyEntry(pe::hard_reset_action) { pe.prl_.transmitHardReset(); }

        static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.report()))
        {
            return state.report();
        }
        void notifyEntry(pe::standby_limit limit)
        {
            pe.load_.setLimit(limit.voltage, pe::i_snk_stdby);
        }
        void notifyEntry(pe::active_contract contract)
        {
            pe.load_.setLimit(contract.voltage, contract.current);
            pe.client_.onContract(contract.voltage, contract.current);
        }
        void notifyEntry(pe::lost_contract report)
        {
            if (report.lost) {
                pe.load_.setLimit(5000, pe::i_default_current);
                pe.client_.onContractLost();
            }
        }

        SinkPolicyEngine& pe;
    };

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
    LOAD& load_;
    CLIENT& client_;
    PrlPort port_{*this};
    ProtocolLayer<TCPC, TIMER, PrlPort> prl_;
    fsm::timed<TIMER&> timed_;
    Reporter reporter_{*this};
    Sender sender_{*this};
    fsm::state_machine<pe::sink_table, fsm::timed<TIMER&>, Reporter, Sender>
        sm_{timed_, reporter_, sender_};
};

} // namespace usbc
