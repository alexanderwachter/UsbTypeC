/*
 * USB PD protocol layer (PRL), following the spec's PRL_Tx/PRL_Rx/PRL_HR
 * as closely as the driver split allows. The driver owns the bit level -
 * CRC checking, GoodCRC autoresponse, per-attempt outcome detection -
 * and everything above lives here: MessageID stamping, the RetryCounter
 * with the retransmission loop, the protocol timers, duplicate
 * rejection on receive, and the MessageID lifecycle across soft and
 * hard resets. Chunked extended messages are not handled yet.
 *
 * Transmit is a state machine with the pending message and RetryCounter
 * in machine-owned context. wait_for_phy_response runs CRCReceiveTimer
 * (tReceive): a failed attempt - reported by the driver or by the timer
 * - retransmits with the same MessageID while RetryCounter allows
 * (guarded self-transition), then lands in transmission_error, which
 * reports onTxError, increments the MessageIDCounter, and rests until
 * the policy engine transmits again or resets. Success and discard
 * outcomes arrive as alerts and report onTxDone/onTxDiscarded; the
 * MessageIDCounter increments on these completions only - never per
 * attempt. transmit() while a message is in flight is refused - the
 * policy engine serializes its requests.
 *
 * Hard reset: transmitHardReset() hands the signal to the driver and
 * waits in wait_for_hard_reset_complete, bounded by
 * HardResetCompleteTimer (tHardResetComplete); PHY confirmation or the
 * timer completes it and reports onHardResetSent.
 *
 * Execution contract: the transmit functions, reset, and onAlert run
 * in the stack's
 * context. The TIMER policy fires fsm::timeout from its own execution
 * context (mtl timer contract), and the integrator serializes it with
 * the stack's calls; client callbacks on timeout paths (onTxError,
 * onHardResetSent) originate from that serialized timer context.
 *
 * The PD revision is a build-time property: n_retry_count is the
 * PD rev 3.x value (a rev 2.0 build would use 3).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Tcpc.hpp>

#include <mtl/StateMachine.hpp>

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace usbc {

namespace concepts {

template<typename T>
concept prl_client = requires(T client, pd_message const& message) {
    client.onMessage(message);
    client.onTxDone();
    client.onTxDiscarded();
    client.onTxError();
    client.onHardReset();      // hard reset received from the partner
    client.onHardResetSent(); // own hard reset signal is on the wire
};

} // namespace concepts

namespace prl {

inline constexpr std::uint8_t n_retry_count = 2; // nRetryCount, PD rev 3.x

inline constexpr auto t_receive = std::chrono::milliseconds{1};             // 0.9 ms - 1.1 ms
inline constexpr auto t_hard_reset_complete = std::chrono::milliseconds{5}; // max 5 ms

// Shared by the transmitting states: the message in flight survives
// the timeout-driven retransmission transitions
struct tx_context {
    pd_message message{};
    std::uint8_t retry_counter = 0;
};

namespace event {

struct tx_request {
    pd_message message;
};
struct phy_success {};
struct phy_discarded {};
struct phy_failed {};
struct hard_reset_request {};
struct reset {};

} // namespace event

// Annotation tag: leaving a state carrying it completes a hard reset
struct hard_reset_sent {
    constexpr bool operator==(hard_reset_sent const&) const = default;
};

namespace state {

struct wait_for_message_request {};

struct wait_for_phy_response {
    static constexpr auto timeout = t_receive; // CRCReceiveTimer

    wait_for_phy_response(event::tx_request const& event, tx_context& ctx) : context(ctx)
    {
        context.message       = event.message;
        context.retry_counter = 0;
    }
    // Re-entry is the retransmission: same message, same MessageID
    explicit wait_for_phy_response(tx_context& ctx) : context(ctx) { ++context.retry_counter; }

    pd_message const& txMessage() const { return context.message; }

    tx_context& context;
};

// PRL_Tx_Transmission_Error folded with the idle wait: reported on
// entry, rests until the policy engine transmits again or resets
struct transmission_error {
    explicit transmission_error(tx_context& ctx) : context(ctx) {}

    // the failed message's SOP*, observed by the client reporter
    sop_type failedSop() const { return context.message.sop; }

    tx_context& context;
};

struct wait_for_hard_reset_complete {
    static constexpr auto timeout = t_hard_reset_complete; // HardResetCompleteTimer

    // leaving this state completes the hard reset, whichever edge takes
    // it out; observed by the client reporter
    static constexpr hard_reset_sent report_hard_reset_sent{};
};

} // namespace state

// PRL_Tx_Check_RetryCounter as a transition guard
struct retries_left {
    static bool check(state::wait_for_phy_response const& state)
    {
        return state.context.retry_counter < n_retry_count;
    }
};

using tx_table = fsm::transition_table<
    fsm::initial<state::wait_for_message_request>,
    fsm::transition<fsm::from<state::wait_for_message_request>, fsm::on<event::tx_request>,
                    fsm::to<state::wait_for_phy_response>>,
    fsm::transition<fsm::from<state::transmission_error>, fsm::on<event::tx_request>,
                    fsm::to<state::wait_for_phy_response>>,
    // no GoodCRC in time: retransmit while RetryCounter allows, else error
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::wait_for_phy_response>, fsm::guard<retries_left>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<fsm::timeout>,
                    fsm::to<state::transmission_error>>,
    // the driver may report a failed attempt before tReceive expires
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_failed>,
                    fsm::to<state::wait_for_phy_response>, fsm::guard<retries_left>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_failed>,
                    fsm::to<state::transmission_error>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_success>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_discarded>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_request>,
                    fsm::to<state::wait_for_hard_reset_complete>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<event::phy_success>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<fsm::timeout>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::reset>,
                    fsm::to<state::wait_for_message_request>>>;

// Hands a state's txMessage() to the TCPC on entry; the accessor is
// the marker that makes a state a transmitting one. A refused hand-off
// is not reported: CRCReceiveTimer turns it into a retry
template<concepts::pd_transport TCPC>
struct phy_driver : fsm::observing<phy_driver<TCPC>> {
    explicit phy_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.txMessage()))
    {
        return state.txMessage();
    }

    void notifyEntry(pd_message const& message) { tcpc.transmit(message); }

    TCPC& tcpc;
};

} // namespace prl

template<concepts::pd_transport TCPC, fsm::concepts::timer TIMER, concepts::prl_client CLIENT>
class ProtocolLayer {
public:
    ProtocolLayer(TCPC& tcpc, TIMER& timer, CLIENT& client)
        : tcpc_(tcpc), client_(client), timed_(timer)
    {
    }

    // Stamps the MessageID; the rest of the header is the caller's.
    // False when a message or hard reset is already in flight
    bool transmit(pd_message message)
    {
        auto header       = pd_header::decode(message.header);
        header.message_id = tx_counter_[index(message.sop)];
        message.header    = header.encode();
        return sm_.process(prl::event::tx_request{message});
    }

    bool transmitHardReset()
    {
        bool const accepted = tcpc_.transmit(transmit_signal::hard_reset);
        sm_.process(prl::event::hard_reset_request{});
        resetAll();
        return accepted;
    }

    // Soft reset scope: the MessageID lifecycle of one SOP* type
    void reset(sop_type sop)
    {
        tx_counter_[index(sop)] = 0;
        rx_id_[index(sop)].reset();
    }

    void onAlert(alert_status alerts)
    {
        if (any(alerts & alert_status::hard_reset_received)) {
            sm_.process(prl::event::reset{});
            resetAll();
            client_.onHardReset();
        }
        if (any(alerts & alert_status::transmit_success)) {
            if (auto const* pending = sm_.template getIf<prl::state::wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_success{});
                client_.onTxDone();
            } else {
                sm_.process(prl::event::phy_success{}); // hard reset confirmation
            }
        }
        if (any(alerts & alert_status::transmit_discarded)) {
            if (auto const* pending = sm_.template getIf<prl::state::wait_for_phy_response>()) {
                increment(pending->context.message.sop);
                sm_.process(prl::event::phy_discarded{});
                client_.onTxDiscarded();
            }
        }
        if (any(alerts & alert_status::transmit_failed)) {
            sm_.process(prl::event::phy_failed{}); // retry or transmission_error
        }
        if (any(alerts & alert_status::message_received)) {
            drainReceived();
        }
    }

private:
    // Reports the outcomes the state machine reaches on its own -
    // possibly from the serialized timer context. Each observation
    // delivers its own type, so the notify hooks cannot collide
    struct client_reporter : fsm::observing<client_reporter> {
        explicit client_reporter(ProtocolLayer& prl_ref) : prl(prl_ref) {}

        // entering a state with a failedSop() is the transmission error
        static constexpr auto observe_nonstatic(auto const& state)
            -> decltype((state.failedSop()))
        {
            return state.failedSop();
        }
        void notifyEntry(sop_type sop) { prl.giveUp(sop); }

        // leaving a report_hard_reset_sent state completes the hard
        // reset - via PHY confirmation, the timer, or a reset event
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::report_hard_reset_sent)
        {
            return STATE::report_hard_reset_sent;
        }
        void notifyExit(prl::hard_reset_sent) { prl.client_.onHardResetSent(); }

        ProtocolLayer& prl;
    };

    static constexpr std::size_t sop_count = 5;

    static constexpr std::size_t index(sop_type sop) { return static_cast<std::size_t>(sop); }

    void increment(sop_type sop)
    {
        auto& counter = tx_counter_[index(sop)];
        counter       = (counter + 1u) & 0x7u;
    }

    // PRL_Tx_Transmission_Error: MessageIDCounter increments, PE informed
    void giveUp(sop_type sop)
    {
        increment(sop);
        client_.onTxError();
    }

    void drainReceived()
    {
        pd_message message;
        while (tcpc_.receive(message)) {
            auto const id = pd_header::decode(message.header).message_id;
            auto& stored  = rx_id_[index(message.sop)];
            if (stored == id) {
                continue; // retransmission of a message already delivered
            }
            stored = id;
            client_.onMessage(message);
        }
    }

    void resetAll()
    {
        tx_counter_ = {};
        rx_id_      = {};
    }

    TCPC& tcpc_;
    CLIENT& client_;
    fsm::timed<TIMER&> timed_;
    prl::phy_driver<TCPC> driver_{tcpc_};
    client_reporter reporter_{*this};
    fsm::state_machine<prl::tx_table, fsm::timed<TIMER&>, prl::phy_driver<TCPC>, client_reporter>
        sm_{timed_, driver_, reporter_};
    std::array<std::uint8_t, sop_count> tx_counter_{};
    std::array<std::optional<std::uint8_t>, sop_count> rx_id_{};
};

} // namespace usbc
