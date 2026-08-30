/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/Pdo.hpp>
#include <usbc/PolicyEngine.hpp>

#include <array>
#include <chrono>
#include <print>
#include <source_location>
#include <span>
#include <vector>

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

// --- application test doubles ------------------------------------------------
struct mock_load {
    usbc::millivolt voltage = 5000;
    usbc::milliamp current  = 0;
    int changes             = 0;

    bool setLimit(usbc::millivolt expected_voltage, usbc::milliamp max_current)
    {
        voltage = expected_voltage;
        current = max_current;
        ++changes;
        return true;
    }
};
static_assert(usbc::concepts::sink_load<mock_load>);

struct mock_pe_client {
    usbc::millivolt voltage = 0;
    usbc::milliamp current  = 0;
    int contracts           = 0;
    int lost                = 0;

    void onContract(usbc::millivolt v, usbc::milliamp i)
    {
        voltage = v;
        current = i;
        ++contracts;
    }
    void onContractLost() { ++lost; }
};
static_assert(usbc::concepts::pe_sink_client<mock_pe_client>);

// --- compile-time checks: codec and the default policy -----------------------
namespace compile_time {

using usbc::pdo::makeFixedSink;

// 9 V / 3 A fixed supply PDO
constexpr std::uint32_t nine_volt = makeFixedSink(9000, 3000);
static_assert(usbc::pdo::kindOf(nine_volt) == usbc::pdo::kind::fixed_supply);
static_assert(usbc::pdo::fixedVoltage(nine_volt) == 9000);
static_assert(usbc::pdo::fixedMaxCurrent(nine_volt) == 3000);

constexpr std::array offered{makeFixedSink(5000, 3000),   // 15 W
                             makeFixedSink(9000, 3000),   // 27 W
                             makeFixedSink(12000, 1500),  // 18 W
                             makeFixedSink(20000, 2250)}; // 45 W

constexpr std::array sink_all{usbc::sink_capability{5000, 3000},
                              usbc::sink_capability{9000, 3000},
                              usbc::sink_capability{12000, 3000},
                              usbc::sink_capability{20000, 3000}};

// no PDO reaches 100 W: the most powerful one wins
constexpr usbc::PowerPolicy wide{5000, 100000};
static_assert(wide.select(offered, sink_all)->position == 4);
static_assert(wide.select(offered, sink_all)->operating_current == 2250);

// the lowest voltage delivering max power wins: 9 V and 20 V both give
// 27 W, 9 V is chosen and the current capped to the power budget
constexpr usbc::PowerPolicy capped{5000, 27000};
static_assert(capped.select(offered, sink_all)->position == 2);
static_assert(capped.select(offered, sink_all)->operating_current == 3000);
constexpr usbc::PowerPolicy fifteen{5000, 15000};
static_assert(fifteen.select(offered, sink_all)->position == 1); // equal power: lower voltage
static_assert(fifteen.select(offered, sink_all)->operating_current == 3000);

// min power must be fulfilled
constexpr usbc::PowerPolicy greedy{50000, 100000};
static_assert(!greedy.select(offered, sink_all).has_value());
constexpr usbc::PowerPolicy min20{20000, 100000};
static_assert(min20.select(offered, sink_all)->position == 4);

// voltages the sink does not list are off limits, and the sink
// capability also caps the current
constexpr std::array sink_low{usbc::sink_capability{5000, 3000},
                              usbc::sink_capability{9000, 2000}};
static_assert(wide.select(offered, sink_low)->position == 2); // 20 V excluded
static_assert(wide.select(offered, sink_low)->operating_current == 2000);
static_assert(wide.select(offered, sink_low)->voltage == 9000);

// non-fixed PDOs are skipped
constexpr std::array augmented_only{std::uint32_t{0b11u << 30u}};
static_assert(!wide.select(augmented_only, sink_all).has_value());

} // namespace compile_time

// --- runtime checks ----------------------------------------------------------
int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        std::print("check failed at {}:{}\n", location.file_name(), location.line());
        ++failures;
    }
}

std::uint8_t next_id = 0;

std::uint16_t makeHeader(std::uint8_t type, std::uint8_t objects)
{
    return usbc::pd_header{.message_type     = type,
                           .port_data_role   = usbc::data_role::dfp,
                           .revision         = usbc::pd_revision::rev_3_x,
                           .port_power_role  = usbc::power_role::source,
                           .message_id       = static_cast<std::uint8_t>(next_id++ & 0x7u),
                           .num_data_objects = objects}
        .encode();
}

usbc::pd_message makeControl(usbc::control_message_type type)
{
    return {.sop = usbc::sop_type::sop, .header = makeHeader(static_cast<std::uint8_t>(type), 0)};
}

usbc::pd_message makeSourceCaps(std::span<std::uint32_t const> objects)
{
    usbc::pd_message message{
        .sop    = usbc::sop_type::sop,
        .header = makeHeader(static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities),
                             static_cast<std::uint8_t>(objects.size()))};
    for (auto const object : objects) {
        message.payload[message.payload_size + 0] = static_cast<std::uint8_t>(object);
        message.payload[message.payload_size + 1] = static_cast<std::uint8_t>(object >> 8u);
        message.payload[message.payload_size + 2] = static_cast<std::uint8_t>(object >> 16u);
        message.payload[message.payload_size + 3] = static_cast<std::uint8_t>(object >> 24u);
        message.payload_size += 4;
    }
    return message;
}

std::uint32_t transmittedObject(mock_tcpc const& tcpc, std::uint8_t index = 0)
{
    auto const& payload = tcpc.last_transmitted.payload;
    auto const offset   = static_cast<std::size_t>(index) * 4;
    return static_cast<std::uint32_t>(payload[offset + 0]) |
           (static_cast<std::uint32_t>(payload[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(payload[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(payload[offset + 3]) << 24u);
}

std::uint8_t transmittedType(mock_tcpc const& tcpc)
{
    return usbc::pd_header::decode(tcpc.last_transmitted.header).message_type;
}

} // namespace

int policyEngineTests()
{
    constexpr std::array sink_caps{usbc::sink_capability{5000, 3000},
                                   usbc::sink_capability{9000, 3000}};

    mock_tcpc tcpc;
    manual_timer prl_timer;
    manual_timer pe_timer;
    usbc::PowerPolicy policy{5000, 27000};
    mock_load load;
    mock_pe_client client;
    usbc::SinkPolicyEngine<mock_tcpc, manual_timer, usbc::PowerPolicy, mock_load, mock_pe_client>
        pe{tcpc, prl_timer, pe_timer, sink_caps, policy, load, client};

    auto deliver = [&](usbc::pd_message const& message) {
        tcpc.injectMessage(message);
        pe.onAlert(*tcpc.readAlert());
    };
    auto txSuccess = [&] { pe.onAlert(usbc::alert_status::transmit_success); };

    // start(): roles configured, SinkWaitCapTimer running
    pe.start();
    check(tcpc.header_info.power == usbc::power_role::sink);
    check(any(tcpc.detect & usbc::receive_detect::sop));
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_sink_wait_cap);

    // source capabilities: the policy picks 9 V / 3 A (position 2)
    constexpr std::array offered{usbc::pdo::makeFixedSink(5000, 3000),
                                 usbc::pdo::makeFixedSink(9000, 3000),
                                 usbc::pdo::makeFixedSink(20000, 2250)};
    deliver(makeSourceCaps(offered));
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::data_message_type::request));
    check(transmittedObject(tcpc) == usbc::pdo::makeFixedRequest(2, 3000, 3000, false));
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_sender_response);
    txSuccess(); // GoodCRC for the request

    // Accept: sink drops to standby for the transition
    deliver(makeControl(usbc::control_message_type::accept));
    check(load.voltage == 9000 && load.current == usbc::pe::i_snk_stdby);
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_ps_transition);

    // PS_RDY: the contract is active
    deliver(makeControl(usbc::control_message_type::ps_rdy));
    check(load.voltage == 9000 && load.current == 3000);
    check(client.contracts == 1 && client.voltage == 9000 && client.current == 3000);
    check(!pe_timer.armed);

    // Get_Sink_Cap answered with the injected capabilities span
    deliver(makeControl(usbc::control_message_type::get_sink_cap));
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::sink_capabilities));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).num_data_objects == 2);
    check(transmittedObject(tcpc, 1) == usbc::pdo::makeFixedSink(9000, 3000));
    txSuccess();

    // new capabilities renegotiate; Reject with a contract returns to ready
    deliver(makeSourceCaps(offered));
    txSuccess();
    deliver(makeControl(usbc::control_message_type::reject));
    check(client.lost == 0);

    // Soft_Reset: protocol layer resets, Accept goes out with MessageID 0
    deliver(makeControl(usbc::control_message_type::soft_reset));
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).message_id == 0);
    txSuccess();
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_sink_wait_cap);
    check(client.lost == 0); // a soft reset does not end the contract

    // negotiate again, then SenderResponse timeout escalates to hard reset
    deliver(makeSourceCaps(offered));
    txSuccess();
    pe_timer.expire(); // no Accept in time
    check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
    txSuccess(); // PHY confirms the hard reset
    check(client.lost == 1);
    check(load.voltage == 5000 && load.current == usbc::pe::i_default_current);
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_sink_wait_cap);

    // SinkWaitCap timeout also escalates to hard reset
    pe_timer.expire();
    check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
    txSuccess();

    // a received hard reset falls back to waiting for capabilities
    deliver(makeSourceCaps(offered));
    txSuccess();
    deliver(makeControl(usbc::control_message_type::accept));
    deliver(makeControl(usbc::control_message_type::ps_rdy));
    check(client.contracts >= 2);
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    pe.onAlert(*tcpc.readAlert());
    check(client.lost == 2);
    check(pe_timer.armed && pe_timer.duration == usbc::pe::t_sink_wait_cap);

    // stop() returns to startup: no timer running
    pe.stop();
    check(!pe_timer.armed);

    return failures;
}
