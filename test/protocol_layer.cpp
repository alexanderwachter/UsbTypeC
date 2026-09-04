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
    void* context                = nullptr;
    bool armed                   = false;

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context  = ctx;
        armed    = true;
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
    int tx_done          = 0;
    int tx_discarded     = 0;
    int tx_error         = 0;
    int hard_resets      = 0;
    int hard_resets_sent = 0;

    void onMessage(usbc::pd_message const& message) { messages.push_back(message); }
    void onTxDone() { ++tx_done; }
    void onTxDiscarded() { ++tx_discarded; }
    void onTxError() { ++tx_error; }
    void onHardReset() { ++hard_resets; }
    void onHardResetSent() { ++hard_resets_sent; }
};
static_assert(usbc::concepts::prl_client<mock_client>);

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

// header codec round-trip, and MessageID stamping leaves the rest alone
constexpr usbc::pd_header request_header{.message_type     = 0x02, // Request
                                         .port_data_role   = usbc::data_role::ufp,
                                         .revision         = usbc::pd_revision::rev_3_x,
                                         .port_power_role  = usbc::power_role::sink,
                                         .message_id       = 5,
                                         .num_data_objects = 1,
                                         .extended         = false};
static_assert(usbc::pd_header::decode(request_header.encode()) == request_header);
static_assert(usbc::pd_header::decode(0x0000u) ==
              usbc::pd_header{.message_type    = 0,
                              .port_data_role  = usbc::data_role::ufp,
                              .revision        = usbc::pd_revision::rev_1_0,
                              .port_power_role = usbc::power_role::sink});
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
    message.header = usbc::pd_header{.message_type     = 0x02,
                                     .port_power_role  = usbc::power_role::sink,
                                     .num_data_objects = 1}
                         .encode();
    return message;
}

std::uint8_t transmittedId(mock_tcpc const& tcpc)
{
    return usbc::pd_header::decode(tcpc.last_transmitted.header).message_id;
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
    mock_tcpc tcpc;
    mock_client client;
    manual_timer timer;
    usbc::ProtocolLayer<mock_tcpc, manual_timer, mock_client> prl{tcpc, timer, client};

    // MessageID stamping; one PHY attempt per request, CRCReceiveTimer runs
    check(prl.transmit(makeRequest()));
    check(tcpc.transmit_count == 1 && transmittedId(tcpc) == 0);
    check(timer.armed);
    check(!prl.transmit(makeRequest())); // busy until the PHY reports
    check(tcpc.transmit_count == 1);

    prl.onAlert(usbc::alert_status::transmit_success);
    check(client.tx_done == 1 && !timer.armed);

    // CRCReceiveTimer expiry retries with the same MessageID, then
    // transmission error after nRetryCount retries
    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 1);
    timer.expire(); // first retry
    check(tcpc.transmit_count == 3 && transmittedId(tcpc) == 1 && timer.armed);
    timer.expire(); // second retry
    check(tcpc.transmit_count == 4 && transmittedId(tcpc) == 1);
    timer.expire(); // retries exhausted
    check(tcpc.transmit_count == 4 && !timer.armed);
    check(client.tx_error == 1);

    // the error incremented the MessageIDCounter exactly once, and the
    // policy engine can transmit straight out of transmission_error
    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 2);

    // a driver-reported failure retries immediately, before tReceive
    prl.onAlert(usbc::alert_status::transmit_failed);
    check(tcpc.transmit_count == 6 && transmittedId(tcpc) == 2 && timer.armed);
    prl.onAlert(usbc::alert_status::transmit_success);
    check(client.tx_done == 2 && client.tx_error == 1);

    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 3);
    prl.onAlert(usbc::alert_status::transmit_discarded);
    check(client.tx_discarded == 1);

    // per-SOP* counters are independent
    check(prl.transmit(makeRequest(usbc::sop_type::sop_prime)));
    check(transmittedId(tcpc) == 0);
    prl.onAlert(usbc::alert_status::transmit_success);

    // a spurious PHY alert with nothing in flight reports nothing
    prl.onAlert(usbc::alert_status::transmit_success);
    check(client.tx_done == 3);

    // receive: forward, drop the retransmission, accept the next id
    tcpc.injectMessage(makeIncoming(4));
    prl.onAlert(*tcpc.readAlert());
    check(client.messages.size() == 1);
    tcpc.injectMessage(makeIncoming(4)); // GoodCRC got lost, source retransmits
    prl.onAlert(*tcpc.readAlert());
    check(client.messages.size() == 1);
    tcpc.injectMessage(makeIncoming(5));
    prl.onAlert(*tcpc.readAlert());
    check(client.messages.size() == 2);

    // soft reset scope: one SOP* type starts over, others keep their ids
    prl.reset(usbc::sop_type::sop);
    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 0);
    prl.onAlert(usbc::alert_status::transmit_success);
    tcpc.injectMessage(makeIncoming(5)); // same id as before the reset: not a duplicate now
    prl.onAlert(*tcpc.readAlert());
    check(client.messages.size() == 3);
    check(prl.transmit(makeRequest(usbc::sop_type::sop_prime)) && transmittedId(tcpc) == 1);
    prl.onAlert(usbc::alert_status::transmit_success);

    // hard reset: aborts a pending tx, HardResetCompleteTimer bounds the
    // wait, PHY confirmation completes it
    check(prl.transmit(makeRequest()));
    check(prl.transmitHardReset());
    check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
    check(timer.armed);
    check(!prl.transmit(makeRequest())); // busy until the hard reset is out
    prl.onAlert(usbc::alert_status::transmit_success);
    check(client.hard_resets_sent == 1 && !timer.armed);
    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 0); // counters reset
    prl.onAlert(usbc::alert_status::transmit_success);

    // ... and the timer completes it when the PHY never confirms
    check(prl.transmitHardReset());
    timer.expire();
    check(client.hard_resets_sent == 2);

    // received hard reset: counters reset, client informed
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    prl.onAlert(*tcpc.readAlert());
    check(client.hard_resets == 1);
    check(prl.transmit(makeRequest()) && transmittedId(tcpc) == 0);
    prl.onAlert(usbc::alert_status::transmit_success);

    // a refused driver hand-off is recovered by the CRCReceiveTimer
    tcpc.accept_transmit = false;
    check(prl.transmit(makeRequest())); // accepted by the PRL, refused by the driver
    tcpc.accept_transmit = true;
    timer.expire(); // retry succeeds at the driver
    check(timer.armed);
    prl.onAlert(usbc::alert_status::transmit_success);

    return failures;
}
