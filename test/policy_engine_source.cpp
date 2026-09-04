/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/SourcePolicyEngine.hpp>

#include <array>
#include <chrono>
#include <print>
#include <source_location>

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

// --- application test doubles ------------------------------------------------
struct mock_supply {
    usbc::supply_callback callback = nullptr;
    void* context                  = nullptr;
    usbc::millivolt voltage        = 5000;
    usbc::milliamp current         = 0;
    int sets                       = 0;

    void setCallback(usbc::supply_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt v, usbc::milliamp i)
    {
        voltage = v;
        current = i;
        ++sets;
        return true;
    }

    // test helper: the supply reports the target reached
    void settle() { callback(context, true); }
};
static_assert(usbc::concepts::source_supply<mock_supply>);

// the contract-notification side as one observer
struct mock_power : usbc::SourcePower<mock_power> {
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
static_assert(usbc::concepts::source_power_client<mock_power>);

// --- compile-time checks: codec, policy, spec notation -----------------------
namespace compile_time {

constexpr std::array offered{usbc::pdo::makeFixedSource(5000, 3000),
                             usbc::pdo::makeFixedSource(9000, 3000)};

static_assert(usbc::pdo::fixedVoltage(offered[1]) == 9000);
static_assert(usbc::pdo::requestPosition(usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 2);
static_assert(usbc::pdo::requestOperatingCurrent(
                  usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 1500);
static_assert(usbc::pdo::requestMaximumCurrent(
                  usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 3000);

// the default policy grants within the PDO, refuses beyond it
constexpr usbc::RequestPolicy policy;
static_assert(policy.evaluate(usbc::pdo::makeFixedRequest(2, 3000, 3000, false),
                              offered)->voltage == 9000);
static_assert(policy.evaluate(usbc::pdo::makeFixedRequest(1, 1000, 1000, false),
                              offered)->current == 1000);
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(2, 3100, 3100, false), offered));
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(5, 1000, 1000, false), offered));
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(0, 1000, 1000, false), offered));

namespace spec_notation {

using namespace usbc::pe;
using namespace usbc::pe::state;

static_assert(pe_src_startup::power == power_level::default_power &&
              pe_src_startup::pd == pd_status::connected_or_not_connected);
static_assert(pe_src_send_capabilities::power == power_level::default_power &&
              pe_src_send_capabilities::pd == pd_status::connected_or_not_connected);
static_assert(pe_src_discovery::power == power_level::default_power &&
              pe_src_discovery::pd == pd_status::not_connected);
static_assert(pe_src_disabled::power == power_level::default_power &&
              pe_src_disabled::pd == pd_status::not_connected);
static_assert(pe_src_negotiate_capability::power == power_level::contract_or_default &&
              pe_src_negotiate_capability::pd == pd_status::connected);
static_assert(pe_src_transition_supply::power == power_level::transition &&
              pe_src_transition_supply::pd == pd_status::connected);
static_assert(pe_src_ready::power == power_level::explicit_contract &&
              pe_src_ready::pd == pd_status::connected);
static_assert(pe_src_capability_response::power == power_level::contract_or_default &&
              pe_src_capability_response::pd == pd_status::connected);
static_assert(pe_src_hard_reset::power == power_level::contract_or_default &&
              pe_src_hard_reset::pd == pd_status::connected_or_not_connected);
static_assert(pe_src_transition_to_default::power == power_level::transition &&
              pe_src_transition_to_default::pd == pd_status::not_connected);
static_assert(pe_src_ready::dot_note == "Power: Explicit Contract | PD: Connected");

} // namespace spec_notation

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

int next_id = 0;

std::uint16_t makeHeader(std::uint8_t message_type, std::uint8_t data_objects)
{
    return usbc::pd_header{.message_type     = message_type,
                           .port_data_role   = usbc::data_role::ufp,
                           .revision         = usbc::pd_revision::rev_3_x,
                           .port_power_role  = usbc::power_role::sink,
                           .message_id       = static_cast<std::uint8_t>(next_id++ & 0x7u),
                           .num_data_objects = data_objects}
        .encode();
}

usbc::pd_message makeControl(usbc::control_message_type type)
{
    return {.sop = usbc::sop_type::sop, .header = makeHeader(static_cast<std::uint8_t>(type), 0)};
}

usbc::pd_message makeRequest(std::uint32_t rdo)
{
    usbc::pd_message message{
        .sop    = usbc::sop_type::sop,
        .header = makeHeader(static_cast<std::uint8_t>(usbc::data_message_type::request), 1)};
    message.payload[0]   = static_cast<std::uint8_t>(rdo);
    message.payload[1]   = static_cast<std::uint8_t>(rdo >> 8u);
    message.payload[2]   = static_cast<std::uint8_t>(rdo >> 16u);
    message.payload[3]   = static_cast<std::uint8_t>(rdo >> 24u);
    message.payload_size = 4;
    return message;
}

std::uint8_t transmittedType(mock_tcpc const& tcpc)
{
    return usbc::pd_header::decode(tcpc.last_transmitted.header).message_type;
}

} // namespace

int policyEngineSourceTests()
{
    constexpr std::array source_caps{usbc::pdo::makeFixedSource(5000, 3000),
                                     usbc::pdo::makeFixedSource(9000, 3000)};

    mock_tcpc tcpc;
    manual_timer prl_timer;
    manual_timer pe_timer;
    usbc::RequestPolicy policy;
    mock_supply supply;
    mock_power power;
    usbc::SourcePolicyEngine<mock_tcpc, manual_timer, usbc::RequestPolicy, mock_supply,
                             mock_power>
        pe{tcpc, prl_timer, pe_timer, source_caps, policy, supply, power};

    auto deliver = [&](usbc::pd_message const& message) {
        tcpc.injectMessage(message);
        pe.onAlert(*tcpc.readAlert());
    };
    auto txSuccess = [&] { pe.onAlert(usbc::alert_status::transmit_success); };
    auto txFailedAttempts = [&] { // PRL: initial attempt plus retries exhausted
        pe.onAlert(usbc::alert_status::transmit_failed);
        pe.onAlert(usbc::alert_status::transmit_failed);
        pe.onAlert(usbc::alert_status::transmit_failed);
    };

    // construction configured the roles and rests in Startup with the
    // supply at vSafe5V defaults
    check(tcpc.header_info.power == usbc::power_role::source);
    check(any(tcpc.detect & usbc::receive_detect::sop));
    check(!pe_timer.armed && tcpc.transmit_count == 0);
    check(supply.sets == 1 && supply.voltage == usbc::pe::v_safe_5v);

    // the Type-C layer reports the sink: capabilities go out
    pe.attached();
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).num_data_objects == 2);
    check(pe_timer.armed);
    txSuccess(); // GoodCRC: the sink speaks PD

    // the Request for 9 V / 3 A: Accept, tSrcTransition, supply, PS_RDY
    deliver(makeRequest(usbc::pdo::makeFixedRequest(2, 3000, 3000, false)));
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    txSuccess();
    check(pe_timer.armed);
    check(supply.sets == 1); // not before tSrcTransition
    pe_timer.expire();
    check(supply.sets == 2 && supply.voltage == 9000 && supply.current == 3000);
    supply.settle();
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::ps_rdy));
    txSuccess();
    check(power.contracts == 1 && power.voltage == 9000 && power.current == 3000);
    check(!pe_timer.armed);

    // Get_Source_Cap re-advertises, then a new Request renegotiates
    deliver(makeControl(usbc::control_message_type::get_source_cap));
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    txSuccess();
    deliver(makeRequest(usbc::pdo::makeFixedRequest(1, 1000, 1000, false)));
    txSuccess(); // Accept
    pe_timer.expire();
    check(supply.voltage == 5000 && supply.current == 1000);
    supply.settle();
    txSuccess(); // PS_RDY
    check(power.contracts == 2 && power.voltage == 5000 && power.current == 1000);

    // an unacceptable Request is rejected; the contract stands
    deliver(makeRequest(usbc::pdo::makeFixedRequest(2, 3100, 3100, false)));
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::reject));
    txSuccess();
    check(power.lost == 0);
    check(supply.voltage == 5000 && supply.current == 1000); // untouched

    // unsupported control answered with Not_Supported from Ready
    deliver(makeControl(usbc::control_message_type::get_sink_cap));
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::control_message_type::not_supported));
    txSuccess();

    // Soft_Reset: protocol layer resets, Accept goes out with MessageID
    // 0, then the capabilities again
    deliver(makeControl(usbc::control_message_type::soft_reset));
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).message_id == 0);
    txSuccess();
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    txSuccess();

    // no Request in tSenderResponse with a PD sink: hard reset, default
    // restored, tSrcRecover, then advertise again
    pe_timer.expire();
    check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
    txSuccess(); // PHY confirms the hard reset
    check(power.lost == 1);
    check(supply.voltage == usbc::pe::v_safe_5v &&
          supply.current == usbc::pe::i_default_current);
    check(pe_timer.armed);
    pe_timer.expire();
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    txSuccess();

    // detach clears everything and rests in Startup
    deliver(makeRequest(usbc::pdo::makeFixedRequest(2, 3000, 3000, false)));
    txSuccess();
    pe_timer.expire();
    supply.settle();
    txSuccess();
    check(power.contracts == 4); // re-advertise cycle negotiated again
    pe.detached();
    check(power.lost == 2);
    check(!pe_timer.armed);

    // a PD-incapable sink: the advertisement fails nCapsCount times,
    // then the engine rests in Disabled without a hard reset
    tcpc.last_signal.reset();
    auto const attempts_before = tcpc.transmit_count;
    pe.attached();
    for (int i = 0; i < 55 && pe_timer.armed; ++i) {
        txFailedAttempts(); // no GoodCRC: the PRL gives up
        if (pe_timer.armed) {
            pe_timer.expire(); // Discovery: next attempt
        }
    }
    check(!pe_timer.armed); // Disabled: no timer running
    check(!tcpc.last_signal); // never escalated to a hard reset
    // nCapsCount + 1 advertisements, each with the PRL's three attempts
    check(tcpc.transmit_count - attempts_before == (usbc::pe::n_caps_count + 1) * 3);
    pe.detached(); // back to Startup for the next sink

    return failures;
}
