/*
 * The USB PD protocol layer's transmit machine: the spec's PRL_Tx with
 * the pending message and RetryCounter in machine-owned context, in
 * the form the PHY asks for (usbc::retransmission, interface/PdPhy.hpp).
 * By default wait_for_phy_response runs CRCReceiveTimer (tReceive): a
 * failed attempt - reported by the PHY or by the timer - retransmits
 * with the same MessageID while RetryCounter allows (guarded
 * self-transition), then lands in transmission_error. A PHY
 * retransmitting itself (a TCPC in the TCPCI sense) gets the table
 * without the retransmission rows: the state waits for the PHY's one
 * outcome under t_transmit_outcome, and its failure is the
 * transmission error. The hard reset waits in
 * wait_for_hard_reset_complete, bounded by HardResetCompleteTimer. The
 * layer running the table is usbc::ProtocolLayer (layer/ProtocolLayer.hpp).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/PdPhy.hpp>
#include <usbc/spec/Message.hpp>
#include <usbc/spec/Parameters.hpp>

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace usbc {

namespace prl {

inline constexpr std::size_t sop_count = 5; // SOP* types with own MessageID lifecycles

inline constexpr auto t_receive = std::chrono::milliseconds{1};             // tReceive
inline constexpr auto t_transmit_outcome = std::chrono::milliseconds{100};  // a PHY's outcome
inline constexpr auto t_hard_reset_complete = std::chrono::milliseconds{5}; // tHardResetComplete

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

// Annotation tag: a state carrying it accepts a tx_request - the
// layer's transmit() gate, tracked by the client reporter
struct tx_ready {
    constexpr bool operator==(tx_ready const&) const = default;
};

namespace state {

struct wait_for_message_request {
    static constexpr auto annotations = fsm::annotate(
        tx_ready{}
    );
};

// PRL_Tx_Wait_for_PHY_Response. With the protocol layer retransmitting
// the timeout is CRCReceiveTimer and re-entry the retransmission; with
// the PHY retransmitting it bounds a PHY that reports no outcome at
// all - its own CRCReceiveTimer and retries finish well within it
template<retransmission WHO_RETRANSMITS>
struct wait_for_phy_response {
    wait_for_phy_response(event::tx_request const& event, tx_context& ctx) : context(ctx)
    {
        context.message = event.message;
        context.retry_counter = 0;
    }
    // Re-entry is the retransmission: same message, same MessageID
    explicit wait_for_phy_response(tx_context& ctx) : context(ctx) { ++context.retry_counter; }

    // the message in flight, observed by the phy_driver
    pd_message const& values() const { return context.message; }

    using contexts = fsm::contexts<tx_context>;

    static constexpr auto timeout =
        WHO_RETRANSMITS == retransmission::protocol_layer ? t_receive : t_transmit_outcome;

    tx_context& context;
};

// PRL_Tx_Transmission_Error folded with the idle wait: reported on
// entry, rests until the policy engine transmits again or resets
struct transmission_error {
    explicit transmission_error(tx_context& ctx) : context(ctx) {}

    static constexpr auto annotations = fsm::annotate(
        tx_ready{}
    );

    // the failed message's SOP*, observed by the client reporter
    sop_type values() const { return context.message.sop; }

    using contexts = fsm::contexts<tx_context>;

    tx_context& context;
};

struct wait_for_hard_reset_complete {
    // leaving this state completes the hard reset, whichever edge takes
    // it out; observed on exit by the client reporter
    static constexpr auto annotations = fsm::annotate(
        hard_reset_sent{}
    );

    static constexpr auto timeout = t_hard_reset_complete; // HardResetCompleteTimer
};

} // namespace state

// PRL_Tx_Check_RetryCounter as a transition guard: the table's
// question, answered by the layer's client reporter - the limit
// follows the negotiated revision, which the layer holds
struct retries_left {};

// The timer range of every timed state, checked against the table
template<retransmission WHO_RETRANSMITS>
inline constexpr auto const& t_wait_for_phy_response =
    WHO_RETRANSMITS == retransmission::protocol_layer ? spec::t_receive : spec::t_transmit_outcome;

template<retransmission WHO_RETRANSMITS>
using prl_timer_ranges = fsm::timer_ranges<
    fsm::timed_by<
        state::wait_for_phy_response<WHO_RETRANSMITS>,
        t_wait_for_phy_response<WHO_RETRANSMITS>>,
    fsm::timed_by<state::wait_for_hard_reset_complete, spec::t_hard_reset_complete>>;

// The retransmission loop: no GoodCRC in time retransmits while
// RetryCounter allows - only where the protocol layer retransmits
template<retransmission WHO_RETRANSMITS>
using retransmission_transitions = std::conditional_t<
    WHO_RETRANSMITS == retransmission::protocol_layer,
    fsm::transition_table<
        fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<fsm::timeout>, fsm::guard<retries_left>, fsm::to<state::wait_for_phy_response<WHO_RETRANSMITS>>>,
        // the PHY may report a failed attempt before tReceive expires
        fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<event::phy_failed>, fsm::guard<retries_left>, fsm::to<state::wait_for_phy_response<WHO_RETRANSMITS>>>>,
    fsm::transition_table<>>;

template<retransmission WHO_RETRANSMITS>
using tx_transitions = fsm::transition_table<
    fsm::initial<state::wait_for_message_request>,
    fsm::transition<fsm::from<state::wait_for_message_request>, fsm::on<event::tx_request>, fsm::to<state::wait_for_phy_response<WHO_RETRANSMITS>>>,
    fsm::transition<fsm::from<state::transmission_error>, fsm::on<event::tx_request>, fsm::to<state::wait_for_phy_response<WHO_RETRANSMITS>>>,
    retransmission_transitions<WHO_RETRANSMITS>,
    // retries exhausted, the PHY's failure, or no outcome at all
    fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<fsm::timeout>, fsm::to<state::transmission_error>>,
    fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<event::phy_failed>, fsm::to<state::transmission_error>>,
    fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<event::phy_success>, fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_phy_response<WHO_RETRANSMITS>>, fsm::on<event::phy_discarded>, fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::hard_reset_request>, fsm::to<state::wait_for_hard_reset_complete>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<event::phy_success>, fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<state::wait_for_hard_reset_complete>, fsm::on<fsm::timeout>, fsm::to<state::wait_for_message_request>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<event::reset>, fsm::to<state::wait_for_message_request>>>;

// A named struct, not an alias: the short name replaces the fully
// spelled table type in every mangled symbol
template<retransmission WHO_RETRANSMITS>
struct tx_table : tx_transitions<WHO_RETRANSMITS> {};
// timeout bounds and reachability checked in test/compliance.cpp

} // namespace prl

} // namespace usbc
