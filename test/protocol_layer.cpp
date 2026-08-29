/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/Message.hpp>
#include <usbc/ProtocolLayer.hpp>

#include <print>
#include <source_location>
#include <vector>

namespace {

// --- policy engine test double ----------------------------------------------
struct mock_client {
    std::vector<usbc::pd_message> messages;
    int tx_done      = 0;
    int tx_discarded = 0;
    int tx_error     = 0;
    int hard_resets  = 0;

    void on_message(usbc::pd_message const& message) { messages.push_back(message); }
    void on_tx_done() { ++tx_done; }
    void on_tx_discarded() { ++tx_discarded; }
    void on_tx_error() { ++tx_error; }
    void on_hard_reset() { ++hard_resets; }
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
    void on_hard_reset(int) {}
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

usbc::pd_message make_request(usbc::sop_type sop = usbc::sop_type::sop)
{
    usbc::pd_message message{.sop = sop, .payload_size = 4, .payload = {0x2c, 0x91, 0x01, 0x13}};
    message.header = usbc::pd_header{.message_type     = 0x02,
                                     .port_power_role  = usbc::power_role::sink,
                                     .num_data_objects = 1}
                         .encode();
    return message;
}

std::uint8_t transmitted_id(mock_tcpc const& tcpc)
{
    return usbc::pd_header::decode(tcpc.last_transmitted.header).message_id;
}

usbc::pd_message make_incoming(std::uint8_t id)
{
    usbc::pd_message message{.sop = usbc::sop_type::sop, .payload_size = 0};
    message.header = usbc::pd_header{.message_type = 0x03, .message_id = id}.encode();
    return message;
}

} // namespace

int protocol_layer_tests()
{
    mock_tcpc tcpc;
    mock_client client;
    usbc::protocol_layer<mock_tcpc, mock_client> prl{tcpc, client};

    // MessageID stamping; one PHY attempt per request
    check(prl.transmit(make_request()));
    check(tcpc.transmit_count == 1 && transmitted_id(tcpc) == 0);
    check(!prl.transmit(make_request())); // busy until the PHY reports
    check(tcpc.transmit_count == 1);

    prl.on_alert(usbc::alert_status::transmit_success);
    check(client.tx_done == 1);

    // PRL_Tx_Check_RetryCounter, rev 3.x: nRetryCount = 2, every
    // retransmission keeps the MessageID, then transmission error
    check(prl.transmit(make_request()) && transmitted_id(tcpc) == 1);
    prl.on_alert(usbc::alert_status::transmit_failed); // first retry
    check(tcpc.transmit_count == 3 && transmitted_id(tcpc) == 1);
    check(client.tx_error == 0); // still trying
    prl.on_alert(usbc::alert_status::transmit_failed); // second retry
    check(tcpc.transmit_count == 4 && transmitted_id(tcpc) == 1);
    prl.on_alert(usbc::alert_status::transmit_failed); // retries exhausted
    check(tcpc.transmit_count == 4);
    check(client.tx_error == 1);

    // the error incremented the MessageIDCounter exactly once
    check(prl.transmit(make_request()) && transmitted_id(tcpc) == 2);
    prl.on_alert(usbc::alert_status::transmit_discarded);
    check(client.tx_discarded == 1);

    // per-SOP* counters are independent
    check(prl.transmit(make_request(usbc::sop_type::sop_prime)));
    check(transmitted_id(tcpc) == 0);
    prl.on_alert(usbc::alert_status::transmit_success);

    // a spurious PHY alert with nothing in flight reports nothing
    prl.on_alert(usbc::alert_status::transmit_success);
    check(client.tx_done == 2);

    // rev 2.0: nRetryCount = 3, four attempts in total
    prl.set_revision(usbc::pd_revision::rev_2_0);
    int const attempts_before = tcpc.transmit_count;
    check(prl.transmit(make_request()));
    prl.on_alert(usbc::alert_status::transmit_failed);
    prl.on_alert(usbc::alert_status::transmit_failed);
    prl.on_alert(usbc::alert_status::transmit_failed);
    check(tcpc.transmit_count == attempts_before + 4);
    prl.on_alert(usbc::alert_status::transmit_failed); // retries exhausted
    check(tcpc.transmit_count == attempts_before + 4 && client.tx_error == 2);
    prl.set_revision(usbc::pd_revision::rev_3_x);

    // receive: forward, drop the retransmission, accept the next id
    tcpc.inject_message(make_incoming(4));
    prl.on_alert(*tcpc.read_alert());
    check(client.messages.size() == 1);
    tcpc.inject_message(make_incoming(4)); // GoodCRC got lost, source retransmits
    prl.on_alert(*tcpc.read_alert());
    check(client.messages.size() == 1);
    tcpc.inject_message(make_incoming(5));
    prl.on_alert(*tcpc.read_alert());
    check(client.messages.size() == 2);

    // soft reset scope: one SOP* type starts over, others keep their ids
    prl.reset(usbc::sop_type::sop);
    check(prl.transmit(make_request()) && transmitted_id(tcpc) == 0);
    prl.on_alert(usbc::alert_status::transmit_success);
    tcpc.inject_message(make_incoming(5)); // same id as before the reset: not a duplicate now
    prl.on_alert(*tcpc.read_alert());
    check(client.messages.size() == 3);
    check(prl.transmit(make_request(usbc::sop_type::sop_prime)) && transmitted_id(tcpc) == 1);
    prl.on_alert(usbc::alert_status::transmit_success);

    // hard reset transmission resets every counter and aborts a pending tx
    check(prl.transmit(make_request()));
    check(prl.transmit_hard_reset());
    check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
    check(prl.transmit(make_request()) && transmitted_id(tcpc) == 0);
    prl.on_alert(usbc::alert_status::transmit_success);

    // received hard reset: counters reset, client informed
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    prl.on_alert(*tcpc.read_alert());
    check(client.hard_resets == 1);
    check(prl.transmit(make_request()) && transmitted_id(tcpc) == 0);

    // the driver refusing the hand-off surfaces as a failed request
    prl.on_alert(usbc::alert_status::transmit_success);
    tcpc.accept_transmit = false;
    check(!prl.transmit(make_request()));
    tcpc.accept_transmit = true;
    check(prl.transmit(make_request())); // machine is idle again afterwards

    return failures;
}
