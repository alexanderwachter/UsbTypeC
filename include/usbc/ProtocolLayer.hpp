/*
 * USB PD protocol layer (PRL), following the spec's PRL_Tx/PRL_Rx as
 * closely as the driver split allows. The driver owns the bit level -
 * CRC checking, GoodCRC autoresponse, per-attempt outcome detection -
 * and everything above lives here: MessageID stamping, the RetryCounter
 * with the retransmission loop, duplicate rejection on receive, and the
 * MessageID lifecycle across soft and hard resets. Chunked extended
 * messages are not handled yet.
 *
 * Transmit is a state machine: idle until the policy engine requests a
 * message, then waiting for the PHY outcome of one attempt. A failed
 * attempt is retransmitted with the same MessageID up to nRetryCount
 * times (PRL_Tx_Check_RetryCounter); then exactly one of on_tx_done /
 * on_tx_discarded / on_tx_error reaches the client, and the
 * MessageIDCounter increments on these completions only - never per
 * attempt, so a retransmission stays recognizable as the same message.
 * transmit() while a message is in flight is refused - the policy
 * engine serializes its requests.
 *
 * Everything here runs in the stack's context: the port feeds
 * read_alert() results into on_alert(), the policy engine sits on top
 * as the client (concepts::prl_client) and gets its notifications from
 * within these calls.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Message.hpp>
#include <usbc/Tcpc.hpp>

#include <mtl/StateMachine.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace usbc {

namespace concepts {

template<typename T>
concept prl_client = requires(T client, pd_message const& message) {
    client.on_message(message);
    client.on_tx_done();
    client.on_tx_discarded();
    client.on_tx_error();
    client.on_hard_reset();
};

} // namespace concepts

namespace prl {

namespace event {

struct tx_request {
    pd_message message;
};
// Retransmission after a failed attempt: same message, same MessageID
struct retry {
    pd_message message;
    std::uint8_t retry_counter;
};
struct phy_success {};
struct phy_discarded {};
struct phy_failed {};
struct reset {};

} // namespace event

namespace state {

struct wait_for_message_request {};

struct wait_for_phy_response {
    wait_for_phy_response() = default;
    explicit wait_for_phy_response(event::tx_request const& event) : tx_message(event.message) {}
    explicit wait_for_phy_response(event::retry const& event)
        : tx_message(event.message), retry_counter(event.retry_counter)
    {
    }
    pd_message tx_message{};
    std::uint8_t retry_counter = 0;
};

} // namespace state

using tx_table = fsm::transition_table<
    fsm::initial<state::wait_for_message_request>,
    fsm::transition<fsm::from<state::wait_for_message_request>, fsm::on<event::tx_request>,
                    fsm::to<state::wait_for_phy_response>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::retry>,
                    fsm::to<state::wait_for_phy_response>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_success>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_discarded>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_phy_response>, fsm::on<event::phy_failed>,
                    fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::reset>,
                    fsm::to<state::wait_for_message_request>>>;

// Hands a state's tx_message to the TCPC on entry; a tx_message member
// is the marker that makes a state a transmitting one
template<concepts::tcpc TCPC>
struct phy_driver : fsm::observing<phy_driver<TCPC>> {
    explicit phy_driver(TCPC& tcpc_ref) : tcpc(tcpc_ref) {}

    static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.tx_message))
    {
        return state.tx_message;
    }

    void notify_entry(pd_message const& message) { accepted = tcpc.transmit(message); }

    TCPC& tcpc;
    bool accepted = true;
};

} // namespace prl

template<concepts::tcpc TCPC, concepts::prl_client CLIENT>
class protocol_layer {
public:
    protocol_layer(TCPC& tcpc, CLIENT& client) : tcpc_(tcpc), client_(client) {}

    void set_revision(pd_revision revision)
    {
        n_retry_count_ = revision == pd_revision::rev_3_x ? 2 : 3;
    }

    // Stamps the MessageID; the rest of the header is the caller's.
    // False when a message is already in flight or the driver refused.
    bool transmit(pd_message message)
    {
        auto header       = pd_header::decode(message.header);
        header.message_id = tx_counter_[index(message.sop)];
        message.header    = header.encode();
        if (!sm_.process(prl::event::tx_request{message})) {
            return false;
        }
        if (!driver_.accepted) {
            sm_.process(prl::event::phy_failed{});
            return false;
        }
        return true;
    }

    bool transmit_hard_reset()
    {
        sm_.process(prl::event::reset{});
        reset_all();
        return tcpc_.transmit(transmit_signal::hard_reset);
    }

    // Soft reset scope: the MessageID lifecycle of one SOP* type
    void reset(sop_type sop)
    {
        tx_counter_[index(sop)] = 0;
        rx_id_[index(sop)].reset();
    }

    void on_alert(alert_status alerts)
    {
        if (any(alerts & alert_status::hard_reset_received)) {
            sm_.process(prl::event::reset{});
            reset_all();
            client_.on_hard_reset();
        }
        if (any(alerts & alert_status::transmit_success)) {
            if (complete(prl::event::phy_success{})) {
                client_.on_tx_done();
            }
        }
        if (any(alerts & alert_status::transmit_discarded)) {
            if (complete(prl::event::phy_discarded{})) {
                client_.on_tx_discarded();
            }
        }
        if (any(alerts & alert_status::transmit_failed)) {
            check_retry_counter();
        }
        if (any(alerts & alert_status::message_received)) {
            drain_received();
        }
    }

private:
    static constexpr std::size_t sop_count = 5;

    static constexpr std::size_t index(sop_type sop) { return static_cast<std::size_t>(sop); }

    // PRL_Tx_Check_RetryCounter: retransmit with the same MessageID
    // until nRetryCount is exhausted, then PRL_Tx_Transmission_Error
    void check_retry_counter()
    {
        auto const* pending = sm_.template get_if<prl::state::wait_for_phy_response>();
        if (pending == nullptr) {
            return; // spurious PHY alert, nothing in flight
        }
        if (pending->retry_counter < n_retry_count_) {
            auto const message = pending->tx_message; // re-entry destroys the state
            auto const attempt = static_cast<std::uint8_t>(pending->retry_counter + 1u);
            sm_.process(prl::event::retry{message, attempt});
            if (driver_.accepted) {
                return;
            }
            // the driver refused the retransmission: give up
        }
        if (complete(prl::event::phy_failed{})) {
            client_.on_tx_error();
        }
    }

    bool complete(auto event)
    {
        auto const* pending = sm_.template get_if<prl::state::wait_for_phy_response>();
        if (pending == nullptr) {
            return false; // spurious PHY alert, nothing in flight
        }
        auto& counter = tx_counter_[index(pending->tx_message.sop)];
        counter       = (counter + 1u) & 0x7u;
        sm_.process(event);
        return true;
    }

    void drain_received()
    {
        pd_message message;
        while (tcpc_.receive(message)) {
            auto const id = pd_header::decode(message.header).message_id;
            auto& stored  = rx_id_[index(message.sop)];
            if (stored == id) {
                continue; // retransmission of a message already delivered
            }
            stored = id;
            client_.on_message(message);
        }
    }

    void reset_all()
    {
        tx_counter_ = {};
        rx_id_      = {};
    }

    TCPC& tcpc_;
    CLIENT& client_;
    std::uint8_t n_retry_count_ = 2; // nRetryCount: 2 since PD rev 3.0, 3 before
    prl::phy_driver<TCPC> driver_{tcpc_};
    fsm::state_machine<prl::tx_table, prl::phy_driver<TCPC>> sm_{driver_};
    std::array<std::uint8_t, sop_count> tx_counter_{};
    std::array<std::optional<std::uint8_t>, sop_count> rx_id_{};
};

} // namespace usbc
