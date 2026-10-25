/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/Message.hpp>
#include <usbc/ProtocolLayer.hpp>

#include <chrono>
#include <print>
#include <source_location>
#include <vector>

// No timeout-value assertions in these tests: every state timeout is
// formally verified against the spec ranges at compile time by the
// fsm::timeouts_within_bounds check next to each transition table.

using namespace std::chrono_literals;

namespace {

// --- timer policy (host/test implementation) --------------------------------
struct manual_timer {
    std::chrono::milliseconds duration{};
    fsm::timer_callback callback = nullptr;
    void* context = nullptr;
    bool armed = false;

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context = ctx;
        armed = true;
    }
    void stop() { armed = false; }
    void expire()
    {
        if (armed) {
            armed = false;
            callback(context);
        }
    }
};
static_assert(fsm::concepts::timer<manual_timer>);

// --- policy engine test double ----------------------------------------------
struct mock_client {
    std::vector<usbc::pd_message> messages;
    int tx_done = 0;
    int tx_discarded = 0;
    int tx_error = 0;
    int hard_resets = 0;
    int hard_resets_sent = 0;

    void onMessage(usbc::pd_message const& message) { messages.push_back(message); }
    void onTxDone() { ++tx_done; }
    void onTxDiscarded() { ++tx_discarded; }
    void onTxError() { ++tx_error; }
    void onHardReset() { ++hard_resets; }
    void onHardResetSent() { ++hard_resets_sent; }
};
static_assert(usbc::concepts::prl_client<mock_client>);

// --- a PHY retransmitting itself ---------------------------------------------
struct retransmitting_phy : mock_phy {
    static constexpr bool retransmits = true;
};
static_assert(usbc::concepts::retransmitting_phy<retransmitting_phy>);
static_assert(!usbc::concepts::retransmitting_phy<mock_phy>);
static_assert(usbc::retransmission_of<mock_phy> == usbc::retransmission::protocol_layer);

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

// header codec round-trip, and MessageID stamping leaves the rest alone
constexpr usbc::pd_header request_header{
    .message_type = 0x02, // Request
    .port_data_role = usbc::data_role::ufp,
    .revision = usbc::pd_revision::rev_3_x,
    .port_power_role = usbc::power_role::sink,
    .message_id = 5,
    .num_data_objects = 1,
    .extended = false
};
static_assert(usbc::pd_header::decode(request_header.encode()) == request_header);
static_assert(
    usbc::pd_header::decode(0x0000u) == usbc::pd_header{
                                            .message_type = 0,
                                            .port_data_role = usbc::data_role::ufp,
                                            .revision = usbc::pd_revision::rev_1_0,
                                            .port_power_role = usbc::power_role::sink
                                        }
);
static_assert(usbc::pd_header::decode(request_header.encode()).message_id == 5);

struct no_hard_reset_hook : mock_client {
    void onHardReset(int) {}
};
static_assert(!usbc::concepts::prl_client<no_hard_reset_hook>);

} // namespace compile_time

// --- runtime checks ---------------------------------------------------------
int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        std::print("check failed at {}:{}\n", location.file_name(), location.line());
        ++failures;
    }
}

usbc::pd_message makeRequest(usbc::sop_type sop = usbc::sop_type::sop)
{
    usbc::pd_message message{.sop = sop, .payload_size = 4, .payload = {0x2c, 0x91, 0x01, 0x13}};
    message.header =
        usbc::pd_header{
            .message_type = 0x02,
            .port_power_role = usbc::power_role::sink,
            .num_data_objects = 1
        }
            .encode();
    return message;
}

std::uint8_t transmittedId(mock_phy const& phy)
{
    return usbc::pd_header::decode(phy.last_transmitted.header).message_id;
}

usbc::pd_message makeIncoming(std::uint8_t id)
{
    usbc::pd_message message{.sop = usbc::sop_type::sop, .payload_size = 0};
    message.header = usbc::pd_header{.message_type = 0x03, .message_id = id}.encode();
    return message;
}

} // namespace

int protocolLayerTests()
{
    mock_phy phy;
    mock_client client;
    manual_timer timer;
    usbc::ProtocolLayer<mock_phy, manual_timer> prl{phy, timer};

    // the client sets itself with the roles the header carries
    check(!prl.clientIs(client));
    prl.setClient(client, usbc::power_role::sink, usbc::data_role::ufp);
    check(prl.clientIs(client));
    check(phy.header_info.power == usbc::power_role::sink);
    check(phy.header_info.data == usbc::data_role::ufp);
    check(phy.header_info.revision == usbc::pd_revision::rev_3_x);
    prl.setDataRole(usbc::data_role::dfp);
    check(phy.header_info.data == usbc::data_role::dfp);

    // the layer hears the PHY only while it holds it, listening for
    // SOP messages and hard resets
    check(!phy.claimed());
    prl.claimPhy();
    check(phy.claimed());
    check(any(phy.detect & usbc::receive_detect::sop));

    // MessageID stamping; one PHY attempt per request, CRCReceiveTimer runs
    check(prl.transmit(makeRequest()));
    check(phy.transmit_count == 1 && transmittedId(phy) == 0);
    check(timer.armed);
    check(!prl.transmit(makeRequest())); // busy until the PHY reports
    check(phy.transmit_count == 1);

    phy.transmitted(usbc::transmit_outcome::success);
    check(client.tx_done == 1 && !timer.armed);

    // CRCReceiveTimer expiry retries with the same MessageID, then
    // transmission error after nRetryCount retries
    check(prl.transmit(makeRequest()) && transmittedId(phy) == 1);
    timer.expire(); // first retry
    check(phy.transmit_count == 3 && transmittedId(phy) == 1 && timer.armed);
    timer.expire(); // second retry
    check(phy.transmit_count == 4 && transmittedId(phy) == 1);
    timer.expire(); // retries exhausted
    check(phy.transmit_count == 4 && !timer.armed);
    check(client.tx_error == 1);

    // the error incremented the MessageIDCounter exactly once, and the
    // policy engine can transmit straight out of transmission_error
    check(prl.transmit(makeRequest()) && transmittedId(phy) == 2);

    // a PHY-reported failure retries immediately, before tReceive
    phy.transmitted(usbc::transmit_outcome::failed);
    check(phy.transmit_count == 6 && transmittedId(phy) == 2 && timer.armed);
    phy.transmitted(usbc::transmit_outcome::success);
    check(client.tx_done == 2 && client.tx_error == 1);

    check(prl.transmit(makeRequest()) && transmittedId(phy) == 3);
    phy.transmitted(usbc::transmit_outcome::discarded);
    check(client.tx_discarded == 1);

    // per-SOP* counters are independent
    check(prl.transmit(makeRequest(usbc::sop_type::sop_prime)));
    check(transmittedId(phy) == 0);
    phy.transmitted(usbc::transmit_outcome::success);

    // a spurious PHY report with nothing in flight reports nothing
    phy.transmitted(usbc::transmit_outcome::success);
    check(client.tx_done == 3);

    // receive: forward, drop the retransmission, accept the next id
    phy.injectMessage(makeIncoming(4));
    check(client.messages.size() == 1);
    phy.injectMessage(makeIncoming(4)); // GoodCRC got lost, source retransmits
    check(client.messages.size() == 1);
    phy.injectMessage(makeIncoming(5));
    check(client.messages.size() == 2);

    // soft reset scope: one SOP* type starts over, others keep their ids
    prl.reset(usbc::sop_type::sop);
    check(prl.transmit(makeRequest()) && transmittedId(phy) == 0);
    phy.transmitted(usbc::transmit_outcome::success);
    phy.injectMessage(makeIncoming(5)); // same id as before the reset: not a duplicate now
    check(client.messages.size() == 3);
    check(prl.transmit(makeRequest(usbc::sop_type::sop_prime)) && transmittedId(phy) == 1);
    phy.transmitted(usbc::transmit_outcome::success);

    // hard reset: aborts a pending tx, HardResetCompleteTimer bounds the
    // wait, PHY confirmation completes it
    check(prl.transmit(makeRequest()));
    check(prl.transmitHardReset());
    check(phy.last_signal == usbc::transmit_signal::hard_reset);
    check(timer.armed);
    check(!prl.transmit(makeRequest())); // busy until the hard reset is out
    phy.transmitted(usbc::transmit_outcome::success);
    check(client.hard_resets_sent == 1 && !timer.armed);
    check(prl.transmit(makeRequest()) && transmittedId(phy) == 0); // counters reset
    phy.transmitted(usbc::transmit_outcome::success);

    // ... and the timer completes it when the PHY never confirms
    check(prl.transmitHardReset());
    timer.expire();
    check(client.hard_resets_sent == 2);

    // received hard reset: counters reset, client informed
    phy.hardResetReceived();
    check(client.hard_resets == 1);
    check(prl.transmit(makeRequest()) && transmittedId(phy) == 0);
    phy.transmitted(usbc::transmit_outcome::success);

    // a refused PHY hand-off is recovered by the CRCReceiveTimer
    phy.accept_transmit = false;
    check(prl.transmit(makeRequest())); // accepted by the PRL, refused by the PHY
    phy.accept_transmit = true;
    timer.expire(); // retry succeeds at the PHY
    check(timer.armed);
    phy.transmitted(usbc::transmit_outcome::success);

    // nRetryCount follows the negotiated revision (the table's
    // retries_left question, answered by the layer): a partner at
    // revision 2.0 brings the link down to it, and a request without
    // GoodCRC now retries three times instead of two
    usbc::pd_message rev2{.sop = usbc::sop_type::sop, .payload_size = 0};
    rev2.header =
        usbc::pd_header{
            .message_type = 0x03,
            .revision = usbc::pd_revision::rev_2_0,
            .message_id = 6
        }
            .encode();
    phy.injectMessage(rev2);
    check(prl.revision() == usbc::pd_revision::rev_2_0);
    check(prl.transmit(makeRequest()));
    auto const attempts_before_retries = phy.transmit_count;
    timer.expire();
    timer.expire();
    timer.expire(); // the third retry PD2 grants
    check(phy.transmit_count == attempts_before_retries + 3 && timer.armed);
    timer.expire(); // retries exhausted
    check(!timer.armed && client.tx_error == 2);

    // A PHY retransmitting itself (a TCPC in the TCPCI sense): one
    // hand-off per request, its outcome is final, and the wait for it
    // is bounded
    {
        retransmitting_phy transport;
        mock_client transport_client;
        manual_timer outcome_timer;
        usbc::ProtocolLayer<retransmitting_phy, manual_timer> retransmitting_prl{
            transport,
            outcome_timer
        };
        static_assert(decltype(retransmitting_prl)::who_retransmits == usbc::retransmission::phy);
        retransmitting_prl
            .setClient(transport_client, usbc::power_role::sink, usbc::data_role::ufp);
        retransmitting_prl.claimPhy();

        check(retransmitting_prl.transmit(makeRequest()) && transmittedId(transport) == 0);
        check(outcome_timer.armed && outcome_timer.duration == usbc::prl::t_transmit_outcome);
        transport.transmitted(usbc::transmit_outcome::success);
        check(transport_client.tx_done == 1 && !outcome_timer.armed);

        // the PHY's failure is the transmission error, no retry here
        check(retransmitting_prl.transmit(makeRequest()) && transmittedId(transport) == 1);
        transport.transmitted(usbc::transmit_outcome::failed);
        check(transport.transmit_count == 2 && !outcome_timer.armed);
        check(transport_client.tx_error == 1);

        // a PHY that never reports: transmission error on the bound
        check(retransmitting_prl.transmit(makeRequest()) && transmittedId(transport) == 2);
        outcome_timer.expire();
        check(transport.transmit_count == 3 && !outcome_timer.armed);
        check(transport_client.tx_error == 2);
        check(retransmitting_prl.transmit(makeRequest()) && transmittedId(transport) == 3);
        transport.transmitted(usbc::transmit_outcome::success);
        check(transport_client.tx_done == 2);

        // released, the layer hears nothing more
        retransmitting_prl.releasePhy();
        check(!transport.claimed());
        check(retransmitting_prl.transmit(makeRequest()));
        transport.transmitted(usbc::transmit_outcome::success);
        check(transport_client.tx_done == 2);
    }

    return failures;
}
