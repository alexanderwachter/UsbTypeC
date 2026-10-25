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

// --- application test doubles ------------------------------------------------
struct mock_supply : usbc::SourceSupply<mock_supply> {
    usbc::supply_ready_callback callback = nullptr;
    void* context = nullptr;
    usbc::millivolt voltage = 5000;
    usbc::milliamp current = 0;
    int sets = 0;

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
    {
        callback = cb;
        context = ctx;
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
    usbc::milliamp current = 0;
    int contracts = 0;
    int lost = 0;

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

constexpr std::array offered{
    usbc::pdo::makeFixedSource(5000, 3000),
    usbc::pdo::makeFixedSource(9000, 3000)
};

static_assert(usbc::pdo::fixedVoltage(offered[1]) == 9000);
static_assert(usbc::pdo::requestPosition(usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 2);
static_assert(
    usbc::pdo::requestOperatingCurrent(usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 1500
);
static_assert(
    usbc::pdo::requestMaximumCurrent(usbc::pdo::makeFixedRequest(2, 1500, 3000, false)) == 3000
);

// the default policy grants within the PDO, refuses beyond it
constexpr usbc::RequestPolicy policy;
static_assert(
    policy.evaluate(usbc::pdo::makeFixedRequest(2, 3000, 3000, false), offered)->voltage == 9000
);
static_assert(
    policy.evaluate(usbc::pdo::makeFixedRequest(1, 1000, 1000, false), offered)->current == 1000
);
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(2, 3100, 3100, false), offered));
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(5, 1000, 1000, false), offered));
static_assert(!policy.evaluate(usbc::pdo::makeFixedRequest(0, 1000, 1000, false), offered));

namespace spec_notation {

using namespace usbc::pe;
using namespace usbc::pe::state;

static_assert(
    pe_src_startup::power == power_level::default_power &&
    !pe_src_startup::annotations.has<pd_status>
);
static_assert(
    pe_src_send_capabilities::power == power_level::default_power &&
    !pe_src_send_capabilities::annotations.has<pd_status>
);
static_assert(
    pe_src_discovery::power == power_level::default_power &&
    pe_src_discovery::pd == pd_status::not_connected
);
static_assert(
    pe_src_disabled::power == power_level::default_power &&
    pe_src_disabled::pd == pd_status::not_connected
);
static_assert(
    pe_src_negotiate_capability::power == power_level::contract_or_default &&
    pe_src_negotiate_capability::pd == pd_status::connected
);
static_assert(
    pe_src_transition_supply::power == power_level::transition &&
    pe_src_transition_supply::pd == pd_status::connected
);
static_assert(
    pe_src_ready::power == power_level::explicit_contract &&
    pe_src_ready::pd == pd_status::connected
);
static_assert(
    pe_src_capability_response::power == power_level::contract_or_default &&
    pe_src_capability_response::pd == pd_status::connected
);
static_assert(
    pe_src_hard_reset::power == power_level::contract_or_default &&
    !pe_src_hard_reset::annotations.has<pd_status>
);
static_assert(
    pe_src_transition_to_default::power == power_level::transition &&
    pe_src_transition_to_default::pd == pd_status::not_connected
);
static_assert(
    pe_src_ready::annotations.get<power_level>() == power_level::explicit_contract &&
    pe_src_ready::annotations.get<pd_status>() == pd_status::connected
);
static_assert(pe_src_discovery::annotations.get<pd_status>() == pd_status::not_connected);

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
    return usbc::pd_header{
        .message_type = message_type,
        .port_data_role = usbc::data_role::ufp,
        .revision = usbc::pd_revision::rev_3_x,
        .port_power_role = usbc::power_role::sink,
        .message_id = static_cast<std::uint8_t>(next_id++ & 0x7u),
        .num_data_objects = data_objects
    }
        .encode();
}

usbc::pd_message makeControl(usbc::control_message_type type)
{
    return {.sop = usbc::sop_type::sop, .header = makeHeader(static_cast<std::uint8_t>(type), 0)};
}

usbc::pd_message makeRequest(std::uint32_t rdo)
{
    usbc::pd_message message{
        .sop = usbc::sop_type::sop,
        .header = makeHeader(static_cast<std::uint8_t>(usbc::data_message_type::request), 1)
    };
    message.payload[0] = static_cast<std::uint8_t>(rdo);
    message.payload[1] = static_cast<std::uint8_t>(rdo >> 8u);
    message.payload[2] = static_cast<std::uint8_t>(rdo >> 16u);
    message.payload[3] = static_cast<std::uint8_t>(rdo >> 24u);
    message.payload_size = 4;
    return message;
}

std::uint8_t transmittedType(mock_phy const& phy)
{
    return usbc::pd_header::decode(phy.last_transmitted.header).message_type;
}

// Observes the engine's port requests (the facade's role in a DRP)
struct recovery_watch : fsm::observing<recovery_watch> {
    int requests = 0;

    void notifyEntry(usbc::tc::request_error_recovery) { ++requests; }
};

// The engine's optional swap features follow the injected policy
// answering the tables' swap questions; this test drives a PR_Swap,
// so extend the stock policy with the answers (the enabled corner of
// the detection - the facade's proxy answers in real use)
struct swap_capable_policy : usbc::RequestPolicy {
    bool allow = true; // the partner's requests: Accept or Reject
    bool check(usbc::pe::pr_swap_allowed) const { return allow; }
    bool check(usbc::pe::dr_swap_allowed) const { return allow; }
};

} // namespace

int policyEngineSourceTests()
{
    constexpr std::array source_caps{
        usbc::pdo::makeFixedSource(5000, 3000),
        usbc::pdo::makeFixedSource(9000, 3000)
    };

    using Layer = usbc::ProtocolLayer<mock_phy, manual_timer>;

    mock_cc cc;
    mock_phy phy;
    manual_timer prl_timer;
    manual_timer pe_timer;
    manual_timer swap_timer; // the timeouts inside a swap exchange
    swap_capable_policy policy;
    mock_supply supply;
    mock_power power;
    Layer prl{phy, prl_timer};
    usbc::SourcePolicyEngine<
        mock_cc,
        Layer,
        manual_timer,
        swap_capable_policy,
        mock_supply,
        mock_power>
        pe{cc,
           prl,
           pe_timer,
           swap_timer,
           source_caps,
           usbc::rp_value::usb_default,
           policy,
           supply,
           power};
    prl.claimPhy(); // the connection layer's part: the PHY reports to the layer

    auto deliver = [&](usbc::pd_message const& message) { phy.injectMessage(message); };
    auto txSuccess = [&] { phy.transmitted(usbc::transmit_outcome::success); };
    auto txFailedAttempts = [&] { // PRL: initial attempt plus retries exhausted
        phy.transmitted(usbc::transmit_outcome::failed);
        phy.transmitted(usbc::transmit_outcome::failed);
        phy.transmitted(usbc::transmit_outcome::failed);
    };

    // construction rests in Startup with the supply at vSafe5V defaults
    check(!pe_timer.armed && phy.transmit_count == 0);
    check(supply.sets == 1 && supply.voltage == usbc::pe::v_safe_5v);
    check(cc.rp == usbc::rp_value::usb_default); // the advertisement, no contract

    // the Type-C layer reports the sink: the engine becomes the
    // layer's client - the header says source - and the capabilities
    // go out
    check(!pe.isLive());
    pe.attached();
    check(pe.isLive());
    check(phy.header_info.power == usbc::power_role::source);
    check(phy.header_info.data == usbc::data_role::dfp);
    check(
        transmittedType(phy) ==
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities)
    );
    check(usbc::pd_header::decode(phy.last_transmitted.header).num_data_objects == 2);
    check(pe_timer.armed);
    txSuccess(); // GoodCRC: the sink speaks PD

    // the Request for 9 V / 3 A: Accept, tSrcTransition, supply, PS_RDY
    deliver(makeRequest(usbc::pdo::makeFixedRequest(2, 3000, 3000, false)));
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    txSuccess();
    check(pe_timer.armed);
    check(supply.sets == 1); // not before tSrcTransition
    pe_timer.expire();
    check(supply.sets == 2 && supply.voltage == 9000 && supply.current == 3000);
    supply.settle();
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::ps_rdy));
    txSuccess();
    check(power.contracts == 1 && power.voltage == 9000 && power.current == 3000);
    check(!pe_timer.armed);

    // PD3 collision avoidance: Ready under the contract advertises
    // SinkTxOk; a source-initiated swap signals SinkTxNG and holds
    // tSinkTx before the request goes out
    check(cc.rp == usbc::rp_value::p_3a0); // SinkTxOk
    check(pe.requestPowerSwap());
    check(cc.rp == usbc::rp_value::p_1a5); // SinkTxNG
    check(pe_timer.armed);                 // tSinkTx
    pe_timer.expire();
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::pr_swap));
    txSuccess();
    deliver(makeControl(usbc::control_message_type::reject));
    check(cc.rp == usbc::rp_value::p_3a0); // back to SinkTxOk in Ready

    // Get_Source_Cap re-advertises, then a new Request renegotiates
    deliver(makeControl(usbc::control_message_type::get_source_cap));
    check(
        transmittedType(phy) ==
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities)
    );
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
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::reject));
    txSuccess();
    check(power.lost == 0);
    check(supply.voltage == 5000 && supply.current == 1000); // untouched

    // unsupported control answered with Not_Supported from Ready
    deliver(makeControl(usbc::control_message_type::get_sink_cap));
    check(
        transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::not_supported)
    );
    txSuccess();

    // the partner's swap requests are the table's questions: the
    // policy's no answers Reject from Ready, the supply stands
    policy.allow = false;
    deliver(makeControl(usbc::control_message_type::pr_swap));
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::reject));
    txSuccess();
    deliver(makeControl(usbc::control_message_type::dr_swap));
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::reject));
    txSuccess();
    check(supply.voltage == 5000 && supply.current == 1000 && power.lost == 0);
    policy.allow = true;

    // Soft_Reset: protocol layer resets, Accept goes out with MessageID
    // 0, then the capabilities again
    deliver(makeControl(usbc::control_message_type::soft_reset));
    check(transmittedType(phy) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    check(usbc::pd_header::decode(phy.last_transmitted.header).message_id == 0);
    txSuccess();
    check(
        transmittedType(phy) ==
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities)
    );
    txSuccess();

    // no Request in tSenderResponse with a PD sink: hard reset, VBUS
    // removed first, tSrcRecover at vSafe0V, defaults restored, then
    // advertise again
    pe_timer.expire();
    check(phy.last_signal == usbc::transmit_signal::hard_reset);
    txSuccess();                // PHY confirms the hard reset
    check(supply.voltage == 0); // VBUS removed before the recovery
    supply.settle();            // at vSafe0V
    check(pe_timer.armed);      // tSrcRecover
    pe_timer.expire();
    check(supply.voltage == usbc::pe::v_safe_5v && supply.current == usbc::pe::i_default_current);
    check(power.lost == 1); // reported with the restored defaults
    supply.settle();        // at vSafe5V: the advertisement resumes
    check(
        transmittedType(phy) ==
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities)
    );
    txSuccess();

    // detach clears everything and rests in Startup
    deliver(makeRequest(usbc::pdo::makeFixedRequest(2, 3000, 3000, false)));
    txSuccess();
    pe_timer.expire();
    supply.settle();
    txSuccess();
    check(power.contracts == 4);           // re-advertise cycle negotiated again
    check(cc.rp == usbc::rp_value::p_3a0); // SinkTxOk under the contract
    pe.detached();
    check(power.lost == 2);
    check(!pe_timer.armed);
    check(!pe.isLive());                         // the layer has no client
    check(cc.rp == usbc::rp_value::usb_default); // the advertisement again

    // a PD-incapable sink: the advertisement fails nCapsCount times,
    // then the engine rests in Disabled without a hard reset
    phy.last_signal.reset();
    auto const attempts_before = phy.transmit_count;
    pe.attached();
    for (int i = 0; i < 55 && pe_timer.armed; ++i) {
        txFailedAttempts(); // no GoodCRC: the PRL gives up
        if (pe_timer.armed) {
            pe_timer.expire(); // Discovery: next attempt
        }
    }
    check(!pe_timer.armed);  // Disabled: no timer running
    check(!phy.last_signal); // never escalated to a hard reset
    // nCapsCount + 1 advertisements, each with the PRL's three attempts
    check(phy.transmit_count - attempts_before == (usbc::pe::n_caps_count + 1) * 3);
    pe.detached(); // back to Startup for the next sink

    // HardResetCounter: a PD-capable sink that never requests gets
    // hard-reset nHardResetCount+1 times, then the engine requests
    // Type-C Error Recovery instead of resetting forever
    {
        mock_cc silent_cc;
        mock_phy silent_phy;
        manual_timer silent_prl_timer;
        manual_timer silent_pe_timer;
        manual_timer silent_swap_timer;
        mock_supply silent_supply;
        mock_power silent_power;
        recovery_watch watch;
        Layer silent_prl{silent_phy, silent_prl_timer};
        usbc::SourcePolicyEngine<
            mock_cc,
            Layer,
            manual_timer,
            usbc::RequestPolicy,
            mock_supply,
            mock_power,
            recovery_watch>
            silent{
                silent_cc,
                silent_prl,
                silent_pe_timer,
                silent_swap_timer,
                source_caps,
                usbc::rp_value::usb_default,
                policy,
                silent_supply,
                silent_power,
                watch
            };
        silent_prl.claimPhy();
        auto confirm = [&] { silent_phy.transmitted(usbc::transmit_outcome::success); };

        silent.attached();
        confirm(); // GoodCRC on the capabilities: a PD sink is present
        for (int reset = 0; reset <= usbc::spec::n_hard_reset_count; ++reset) {
            silent_pe_timer.expire(); // SenderResponseTimer, no Request
            check(silent_phy.last_signal == usbc::transmit_signal::hard_reset);
            silent_phy.last_signal.reset();
            confirm();                // the PHY confirms the hard reset
            silent_supply.settle();   // VBUS removed
            silent_pe_timer.expire(); // tSrcRecover
            silent_supply.settle();   // defaults back: the caps go out
            confirm();                // GoodCRC again
            check(watch.requests == 0);
        }
        silent_pe_timer.expire();                   // the counter is spent
        check(!silent_phy.last_signal.has_value()); // no further hard reset
        check(watch.requests == 1);                 // Error Recovery requested
    }

    return failures;
}
