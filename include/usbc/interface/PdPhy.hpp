/*
 * The USB PD Physical Layer (PHY Layer, PD spec chapter 5) as an
 * integration interface: the BMC transmitter and receiver on the CC
 * wire that the protocol layer drives. The contract is what the spec
 * asks of the PHY towards the protocol layer - transmit a packet and
 * report its outcome, deliver what was received for the configured
 * SOP* types, signal and report Hard Reset - kept implementable by a
 * driver that is not a bare PHY: a TCPCI port controller answers
 * GoodCRC and runs the retransmissions itself, which it declares
 * (retransmission below). A Type-C-only port needs none of it.
 *
 * Execution contract: the callbacks arrive in the stack's serialized
 * context (the Zephyr adapter delivers on the stack's workqueue), one
 * per event, in the order the events happened; a received message is
 * handed over in the callback. The functions are called from the
 * stack's context only. The protocol layer registers its callbacks
 * when its engine goes live and clears them (null pointers, nothing
 * reported) when it is done - a dual-role port runs one protocol layer
 * per role on the same PHY.
 *
 * Failure contract: bool functions report whether the driver accepted
 * the operation. transmit() reports its outcome through transmitted():
 * success on GoodCRC, discarded when an incoming message pre-empted it,
 * failed without a GoodCRC. Who retransmits is the PHY's declaration:
 * by default transmit() is one attempt and the protocol layer runs
 * CRCReceiveTimer and the nRetryCount retransmissions, as the spec's
 * PRL_Tx does; a PHY doing that itself - a TCPC in the TCPCI sense -
 * declares `static constexpr bool retransmits = true;` and reports one
 * outcome per message, failed once its retries are spent. A hard reset
 * signal is confirmed with transmitted(success) as well.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Cc.hpp>
#include <usbc/spec/Flags.hpp>
#include <usbc/spec/Message.hpp>

#include <concepts>
#include <cstdint>

namespace usbc {

// What the PHY shall detect and deliver; a driver without this
// granularity enables at least the requested classes
enum class receive_detect : std::uint8_t {
    none = 0,
    sop = 1u << 0,
    sop_prime = 1u << 1,
    sop_double_prime = 1u << 2,
    sop_prime_debug = 1u << 3,
    sop_double_prime_debug = 1u << 4,
    hard_reset = 1u << 5,
    cable_reset = 1u << 6,
};

template<>
struct is_flags<receive_detect> : std::true_type {};

// What the PHY needs to build GoodCRC answers
struct message_header_info {
    power_role power;
    data_role data;
    pd_revision revision;
};

// Transmissions without message content
enum class transmit_signal : std::uint8_t {
    hard_reset,
    cable_reset,
    bist_carrier_mode_2,
};

// The outcome of one transmit()
enum class transmit_outcome : std::uint8_t {
    success,   // GoodCRC received
    discarded, // dropped in favor of an incoming message
    failed,    // no GoodCRC within the retries
};

// The protocol layer's ears; a null pointer is an event nobody hears
struct pd_phy_callbacks {
    void (*transmitted)(void* context, transmit_outcome outcome) = nullptr;
    void (*received)(void* context, pd_message const& message) = nullptr;
    void (*hard_reset_received)(void* context) = nullptr;
};

namespace concepts {

template<typename T>
concept pd_phy = requires(
    T phy,
    pd_phy_callbacks const& callbacks,
    void* context,
    receive_detect detect,
    message_header_info header_info,
    plug_orientation line,
    pd_message const& message,
    transmit_signal signal
) {
    phy.setCallbacks(callbacks, context);
    { phy.setMessageHeaderInfo(header_info) } -> std::same_as<bool>;
    { phy.setReceiveDetect(detect) } -> std::same_as<bool>;
    { phy.selectCcLine(line) } -> std::same_as<bool>; // the wire carrying the messaging
    { phy.transmit(message) } -> std::same_as<bool>;  // outcome via transmitted()
    { phy.transmit(signal) } -> std::same_as<bool>;
};

} // namespace concepts

// Who runs CRCReceiveTimer and the nRetryCount retransmissions of a
// message: the protocol layer, as the spec's PRL_Tx does, or a PHY
// doing it itself - a TCPC in the TCPCI sense - which then reports one
// outcome per message
enum class retransmission : std::uint8_t {
    protocol_layer,
    phy,
};

namespace concepts {

// A PHY running CRCReceiveTimer and the retransmissions itself
template<typename T>
concept retransmitting_phy = pd_phy<T> && requires {
    { T::retransmits } -> std::same_as<bool const&>;
} && T::retransmits;

} // namespace concepts

template<concepts::pd_phy PHY>
inline constexpr retransmission retransmission_of =
    concepts::retransmitting_phy<PHY> ? retransmission::phy : retransmission::protocol_layer;

} // namespace usbc
