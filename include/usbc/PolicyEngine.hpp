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
 * Deviations kept for later: PE_SNK_Evaluate_Capability runs inside
 * the message dispatch rather than as an own state, Wait is treated as
 * Reject (no tSinkRequest retry), GotoMin and swaps are ignored, and
 * there is no HardResetCounter - the sink retries indefinitely.
 *
 * Integration: the engine drives a pd_transport driver through its own
 * ProtocolLayer; feed TCPC alerts into onAlert(), call start() once the
 * Type-C layer reports attach and stop() on detach. Everything runs in
 * the stack's serialized context; client callbacks may originate from
 * the timer context (mtl timer contract).
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

inline constexpr milliamp i_snk_stdby       = 500; // iSnkStdby at any voltage
inline constexpr milliamp i_default_current = 500; // implicit vSafe5V contract

struct pe_context {
    contract_request request{};
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
struct stopped {};
struct send_request {
    pd_message message;
    contract_request terms;
};
struct send_sink_caps {
    pd_message message;
};
struct soft_reset_received {
    pd_message accept;
};
struct accept {};
struct reject_or_wait {};
struct ps_rdy {};
struct tx_done {};
struct tx_error {};
struct hard_reset_sent {};
struct hard_reset_received {};

} // namespace event

namespace state {

struct startup {
    explicit startup(pe_context& ctx) : context(ctx) { context = {}; }
    pe_context& context;
};

struct wait_for_capabilities {
    static constexpr auto timeout = t_sink_wait_cap; // SinkWaitCapTimer

    wait_for_capabilities(event::started const&, pe_context& ctx) : context(ctx) { context = {}; }
    wait_for_capabilities(event::hard_reset_sent const&, pe_context& ctx)
        : context(ctx), lost_(ctx.explicit_contract)
    {
        context = {};
    }
    wait_for_capabilities(event::hard_reset_received const&, pe_context& ctx)
        : context(ctx), lost_(ctx.explicit_contract)
    {
        context = {};
    }
    explicit wait_for_capabilities(pe_context& ctx) : context(ctx) {}

    lost_contract report() const { return {lost_}; }

    pe_context& context;

private:
    bool lost_ = false;
};

struct select_capability {
    static constexpr auto timeout = t_sender_response; // SenderResponseTimer

    select_capability(event::send_request const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
        context.request = event.terms;
    }
    explicit select_capability(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

struct transition_sink {
    static constexpr auto timeout = t_ps_transition; // PSTransitionTimer

    explicit transition_sink(pe_context& ctx) : context(ctx) {}

    standby_limit report() const { return {context.request.voltage}; }

    pe_context& context;
};

struct ready {
    explicit ready(pe_context& ctx) : context(ctx) { context.explicit_contract = true; }

    active_contract report() const
    {
        return {context.request.voltage, context.request.operating_current};
    }

    pe_context& context;
};

struct give_sink_cap {
    give_sink_cap(event::send_sink_caps const& event, pe_context& ctx)
        : context(ctx), message_(event.message)
    {
    }
    explicit give_sink_cap(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

// Accepts a received Soft_Reset; the reporter resets the protocol
// layer before the sender transmits the Accept
struct soft_reset {
    static constexpr prl_reset_action action{};

    soft_reset(event::soft_reset_received const& event, pe_context& ctx)
        : context(ctx), message_(event.accept)
    {
    }
    explicit soft_reset(pe_context& ctx) : context(ctx) {}

    pd_message const& txMessage() const { return message_; }

    pe_context& context;

private:
    pd_message message_{};
};

struct hard_reset {
    static constexpr hard_reset_action action{};

    explicit hard_reset(pe_context& ctx) : context(ctx) {}
    pe_context& context;
};

} // namespace state

struct has_explicit_contract {
    static bool check(state::select_capability const& state)
    {
        return state.context.explicit_contract;
    }
};

using sink_table = fsm::transition_table<
    fsm::initial<state::startup>,
    fsm::transition<fsm::from<state::startup>, fsm::on<event::started>,
                    fsm::to<state::wait_for_capabilities>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::stopped>, fsm::to<state::startup>>,
    // the evaluated source capabilities arrive as a prepared request
    fsm::transition<fsm::from<state::wait_for_capabilities>, fsm::on<event::send_request>,
                    fsm::to<state::select_capability>>,
    fsm::transition<fsm::from<state::ready>, fsm::on<event::send_request>,
                    fsm::to<state::select_capability>>,
    fsm::transition<fsm::from<state::wait_for_capabilities>, fsm::on<fsm::timeout>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<state::select_capability>, fsm::on<event::accept>,
                    fsm::to<state::transition_sink>>,
    fsm::transition<fsm::from<state::select_capability>, fsm::on<event::reject_or_wait>,
                    fsm::to<state::ready>, fsm::guard<has_explicit_contract>>,
    fsm::transition<fsm::from<state::select_capability>, fsm::on<event::reject_or_wait>,
                    fsm::to<state::wait_for_capabilities>>,
    fsm::transition<fsm::from<state::select_capability>, fsm::on<fsm::timeout>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<state::select_capability>, fsm::on<event::tx_error>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<state::transition_sink>, fsm::on<event::ps_rdy>,
                    fsm::to<state::ready>>,
    fsm::transition<fsm::from<state::transition_sink>, fsm::on<fsm::timeout>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<state::ready>, fsm::on<event::send_sink_caps>,
                    fsm::to<state::give_sink_cap>>,
    fsm::transition<fsm::from<state::give_sink_cap>, fsm::on<event::tx_done>,
                    fsm::to<state::ready>>,
    fsm::transition<fsm::from<state::give_sink_cap>, fsm::on<event::tx_error>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::soft_reset_received>,
                    fsm::to<state::soft_reset>>,
    fsm::transition<fsm::from<state::soft_reset>, fsm::on<event::tx_done>,
                    fsm::to<state::wait_for_capabilities>>,
    fsm::transition<fsm::from<state::soft_reset>, fsm::on<event::tx_error>,
                    fsm::to<state::hard_reset>>,
    fsm::transition<fsm::from<state::hard_reset>, fsm::on<event::hard_reset_sent>,
                    fsm::to<state::wait_for_capabilities>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_received>,
                    fsm::to<state::wait_for_capabilities>>>;

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
    }

    // Call once the Type-C layer reports attach
    void start()
    {
        tcpc_.setMessageHeaderInfo(
            {power_role::sink, data_role::ufp, pd_revision::rev_3_x});
        tcpc_.setReceiveDetect(receive_detect::sop | receive_detect::hard_reset);
        sm_.process(pe::event::started{});
    }

    void stop() { sm_.process(pe::event::stopped{}); }

    // Feed the TCPC's PD alerts (message/transmit/hard reset bits)
    void onAlert(alert_status alerts) { prl_.onAlert(alerts); }

private:
    // The protocol layer's client, forwarding into the engine
    struct PrlPort {
        SinkPolicyEngine& pe;

        void onMessage(pd_message const& message) { pe.dispatch(message); }
        void onTxDone() { pe.sm_.process(pe::event::tx_done{}); }
        void onTxDiscarded() {} // the preempting message drives the engine
        void onTxError() { pe.sm_.process(pe::event::tx_error{}); }
        void onHardReset() { pe.sm_.process(pe::event::hard_reset_received{}); }
        void onHardResetSent() { pe.sm_.process(pe::event::hard_reset_sent{}); }
    };

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
        if (isData(header, data_message_type::source_capabilities)) {
            evaluate(message, header.num_data_objects);
        } else if (isControl(header, control_message_type::accept)) {
            sm_.process(pe::event::accept{});
        } else if (isControl(header, control_message_type::reject) ||
                   isControl(header, control_message_type::wait)) {
            sm_.process(pe::event::reject_or_wait{});
        } else if (isControl(header, control_message_type::ps_rdy)) {
            sm_.process(pe::event::ps_rdy{});
        } else if (isControl(header, control_message_type::get_sink_cap)) {
            sendSinkCapabilities();
        } else if (isControl(header, control_message_type::soft_reset)) {
            sm_.process(pe::event::soft_reset_received{
                makeControl(control_message_type::accept)});
        }
        // everything else is ignored for now
    }

    // PE_SNK_Evaluate_Capability: ask the policy, fall back to the
    // first PDO with the Capability Mismatch flag
    void evaluate(pd_message const& message, std::uint8_t count)
    {
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
        sm_.process(pe::event::send_request{request, *terms});
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
