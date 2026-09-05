/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/PdDrp.hpp>

// No timeout-value assertions in these tests: every state timeout is
// formally verified against the spec ranges at compile time by the
// fsm::timeouts_within_bounds check next to each transition table.

#include <array>
#include <chrono>
#include <cstdint>
#include <print>
#include <source_location>

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

// --- the user's domain pieces, as test doubles ------------------------------
constexpr std::array sink_capabilities{usbc::sink_capability{5000, 3000}};
constexpr std::array source_caps{usbc::pdo::makeFixedSource(5000, 1500)};

struct mock_sink_power : usbc::SinkPower<mock_sink_power> {
    int limits = 0;
    usbc::milliamp last_limit = 0;
    bool setLimit(usbc::millivolt, usbc::milliamp current)
    {
        ++limits;
        last_limit = current;
        return true;
    }
    void onContract(usbc::millivolt, usbc::milliamp) {}
    void onContractLost() {}
};

struct mock_supply {
    usbc::supply_ready_callback callback = nullptr;
    void* context                        = nullptr;
    usbc::millivolt voltage              = 5000;
    int outputs                          = 0;

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt v, usbc::milliamp)
    {
        voltage = v;
        ++outputs;
        return true;
    }

    // test helper: the regulator reaches the commanded target
    void settle() { callback(context, true); }
};
static_assert(usbc::concepts::source_supply<mock_supply>);

struct mock_source_power : usbc::SourcePower<mock_source_power> {
    void onContract(usbc::millivolt, usbc::milliamp) {}
    void onContractLost() {}
};

// --- partner messages --------------------------------------------------------
int next_id = 0;

usbc::pd_message partnerMessage(std::uint8_t message_type, std::uint8_t data_objects,
                                usbc::power_role power, usbc::data_role data,
                                usbc::pd_revision revision = usbc::pd_revision::rev_2_0)
{
    return {.sop    = usbc::sop_type::sop,
            .header = usbc::pd_header{.message_type     = message_type,
                                      .port_data_role   = data,
                                      .revision         = revision,
                                      .port_power_role  = power,
                                      .message_id = static_cast<std::uint8_t>(next_id++ & 0x7u),
                                      .num_data_objects = data_objects}
                          .encode()};
}

usbc::pd_message partnerControl(usbc::control_message_type type, usbc::power_role power,
                                usbc::data_role data)
{
    return partnerMessage(static_cast<std::uint8_t>(type), 0, power, data);
}

void putObject(usbc::pd_message& message, std::uint32_t object)
{
    auto const offset           = message.payload_size;
    message.payload[offset + 0] = static_cast<std::uint8_t>(object);
    message.payload[offset + 1] = static_cast<std::uint8_t>(object >> 8u);
    message.payload[offset + 2] = static_cast<std::uint8_t>(object >> 16u);
    message.payload[offset + 3] = static_cast<std::uint8_t>(object >> 24u);
    message.payload_size += 4;
}

std::uint8_t transmittedType(mock_tcpc const& tcpc)
{
    return usbc::pd_header::decode(tcpc.last_transmitted.header).message_type;
}

bool transmittedControl(mock_tcpc const& tcpc, usbc::control_message_type type)
{
    return transmittedType(tcpc) == static_cast<std::uint8_t>(type);
}

// --- runtime checks ---------------------------------------------------------
int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        std::print("check failed at {}:{}\n", location.file_name(), location.line());
        ++failures;
    }
}

} // namespace

// The facade wires the port from domain pieces alone; the role swaps
// are full PD exchanges: PR_Swap/DR_Swap, Accept, and the PS_RDY
// hand-off, with the Type-C terminations flipped at the spec's
// Assert_Rd/Assert_Rp moments
int pdDrpTests()
{
    using Port = usbc::PdDrp<mock_tcpc, mock_vbus, manual_timer, usbc::PowerPolicy,
                             mock_sink_power, usbc::RequestPolicy, mock_supply,
                             mock_source_power>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    usbc::pd_drp_timers<manual_timer> timers;
    usbc::PowerPolicy sink_policy{2500, 15000};
    mock_sink_power sink_power;
    usbc::RequestPolicy source_policy;
    mock_supply supply;
    mock_source_power source_power;

    Port port{tcpc,        vbus,          timers, sink_capabilities, sink_policy, sink_power,
              source_caps, source_policy, supply, source_power,      usbc::rp_value::p_1a5};

    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };
    auto const deliver   = [&](usbc::pd_message const& message) { tcpc.injectMessage(message); };
    auto const txSuccess = [&] {
        tcpc.alerts |= usbc::alert_status::transmit_success;
        tcpc.callback(tcpc.context);
    };

    port.start();
    check(tcpc.pull == usbc::cc_pull::rd); // toggling, sink phase first
    check(!port.powerRole() && !port.dataRole());

    // a charger appears: the port resolves to sink, the sink engine
    // takes over, the header says sink/UFP
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    vbus.setVoltage(5000);
    timers.tc.expire(); // tCCDebounce
    check(tcpc.sinking && port.powerRole() == usbc::power_role::sink);
    check(port.dataRole() == usbc::data_role::ufp);
    check(tcpc.header_info.power == usbc::power_role::sink);
    check(tcpc.header_info.data == usbc::data_role::ufp);
    check(timers.sink_pe.armed); // SinkWaitCapTimer runs

    // negotiation: the partner's capabilities, our Request, Accept,
    // PS_RDY - an explicit contract (swaps are allowed only under one).
    // The partner is a PD 2.0 device: the lowest common revision is
    // adopted, stamped into our headers, and the GoodCRC header follows
    auto caps = partnerMessage(
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities), 1,
        usbc::power_role::source, usbc::data_role::dfp, usbc::pd_revision::rev_2_0);
    putObject(caps, usbc::pdo::makeFixedSource(5000, 3000));
    deliver(caps);
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::request));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).revision ==
          usbc::pd_revision::rev_2_0);
    check(tcpc.header_info.revision == usbc::pd_revision::rev_2_0);
    txSuccess();
    deliver(partnerControl(usbc::control_message_type::accept, usbc::power_role::source,
                           usbc::data_role::dfp));
    deliver(partnerControl(usbc::control_message_type::ps_rdy, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(sink_power.limits > 0); // the contract reached the load

    // a DRP answers Get_Source_Cap while sinking with its source-role
    // capabilities (PE_DR_SNK_Give_Source_Cap)
    deliver(partnerControl(usbc::control_message_type::get_source_cap,
                           usbc::power_role::source, usbc::data_role::dfp));
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    txSuccess();

    // DR_Swap, our request: DR_Swap out; the partner's Wait retries it
    // after tDRSwapWait, the Accept then flips both sides
    check(port.swapDataRole());
    check(transmittedControl(tcpc, usbc::control_message_type::dr_swap));
    txSuccess();
    deliver(partnerControl(usbc::control_message_type::wait, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(port.dataRole() == usbc::data_role::ufp); // nothing flipped
    check(timers.sink_pe.armed);                    // tDRSwapWait runs
    timers.sink_pe.expire();
    check(transmittedControl(tcpc, usbc::control_message_type::dr_swap)); // retried
    txSuccess();
    deliver(partnerControl(usbc::control_message_type::accept, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(port.dataRole() == usbc::data_role::dfp);
    check(tcpc.header_info.data == usbc::data_role::dfp);
    check(tcpc.sinking); // power roles untouched

    // BIST Carrier Mode 2 (the contract is vSafe5V): the carrier goes
    // out for tBISTContMode, then normal operation resumes
    auto bist_carrier = partnerMessage(
        static_cast<std::uint8_t>(usbc::data_message_type::bist), 1,
        usbc::power_role::source, usbc::data_role::dfp);
    putObject(bist_carrier, 5u << 28); // Carrier Mode 2 BDO
    deliver(bist_carrier);
    check(tcpc.last_signal == usbc::transmit_signal::bist_carrier_mode_2);
    check(timers.sink_pe.armed); // BISTContModeTimer
    timers.sink_pe.expire();     // back to Ready

    // BIST Test Data: the engine goes deaf to messages (the TCPC keeps
    // answering GoodCRC) until a hard reset ends the test mode
    auto bist_test = partnerMessage(static_cast<std::uint8_t>(usbc::data_message_type::bist),
                                    1, usbc::power_role::source, usbc::data_role::dfp);
    putObject(bist_test, 8u << 28); // Test Data BDO
    deliver(bist_test);
    auto const tx_before_bist = tcpc.transmit_count;
    deliver(partnerControl(usbc::control_message_type::get_sink_cap,
                           usbc::power_role::source, usbc::data_role::dfp));
    check(tcpc.transmit_count == tx_before_bist); // silenced

    // the partner hard-resets: the connection layer holds the attach
    // through the legitimate VBUS cycle instead of detaching, and the
    // swapped data role survives (a hard reset does not change it)
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    tcpc.callback(tcpc.context);
    check(!tcpc.sinking && !port.powerRole()); // window open, not detached
    check(tcpc.pull == usbc::cc_pull::rd);
    vbus.setVoltage(0); // the source removes VBUS - not a detach
    vbus.setVoltage(5000); // ... and restores it
    check(tcpc.sinking && port.powerRole() == usbc::power_role::sink);
    check(port.dataRole() == usbc::data_role::dfp); // preserved
    check(tcpc.header_info.data == usbc::data_role::dfp);
    check(timers.sink_pe.armed); // NoResponseTimer awaits the capabilities

    // the source re-advertises: the contract re-establishes (and the
    // PD 2.0 revision is re-adopted after the hard reset's reset)
    auto caps_again = partnerMessage(
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities), 1,
        usbc::power_role::source, usbc::data_role::dfp, usbc::pd_revision::rev_2_0);
    putObject(caps_again, usbc::pdo::makeFixedSource(5000, 3000));
    deliver(caps_again);
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::request));
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).revision ==
          usbc::pd_revision::rev_2_0);
    txSuccess();
    deliver(partnerControl(usbc::control_message_type::accept, usbc::power_role::source,
                           usbc::data_role::dfp));
    deliver(partnerControl(usbc::control_message_type::ps_rdy, usbc::power_role::source,
                           usbc::data_role::dfp));

    // DR_Swap, the partner's request: we Accept and flip back
    deliver(partnerControl(usbc::control_message_type::dr_swap, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(transmittedControl(tcpc, usbc::control_message_type::accept));
    txSuccess();
    check(port.dataRole() == usbc::data_role::ufp);
    check(tcpc.header_info.data == usbc::data_role::ufp);

    // PR_Swap, our request (sink -> source): the agreement enters the
    // swap standby - draw stops, detach detection is suspended
    check(port.swapPowerRole());
    check(transmittedControl(tcpc, usbc::control_message_type::pr_swap));
    txSuccess();
    deliver(partnerControl(usbc::control_message_type::accept, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(sink_power.last_limit == usbc::spec::i_snk_stdby); // standby draw
    check(!tcpc.sinking && !port.powerRole());               // standby holds
    vbus.setVoltage(0); // the old source collapses VBUS - not a detach

    // the old source's PS_RDY: Assert_Rp, VBUS on, our PS_RDY, then
    // the capabilities after tSwapSourceStart
    deliver(partnerControl(usbc::control_message_type::ps_rdy, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(tcpc.sourcing && port.powerRole() == usbc::power_role::source);
    check(tcpc.header_info.power == usbc::power_role::source);
    check(port.dataRole() == usbc::data_role::ufp); // preserved across the swap
    check(supply.voltage == 5000 && supply.outputs > 0);
    supply.settle();
    check(transmittedControl(tcpc, usbc::control_message_type::ps_rdy));
    txSuccess();
    check(timers.source_pe.armed); // SwapSourceStartTimer
    timers.source_pe.expire();
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities));
    // the negotiated revision survived the engine handover
    check(usbc::pd_header::decode(tcpc.last_transmitted.header).revision ==
          usbc::pd_revision::rev_2_0);
    txSuccess();

    // the new sink requests 5 V: the negotiation completes in source
    // role - only then is the engine Ready for further swaps
    auto request = partnerMessage(static_cast<std::uint8_t>(usbc::data_message_type::request),
                                  1, usbc::power_role::sink, usbc::data_role::ufp);
    putObject(request, usbc::pdo::makeFixedRequest(1, 1000, 1500, false));
    deliver(request);
    check(transmittedControl(tcpc, usbc::control_message_type::accept));
    txSuccess();
    timers.source_pe.expire(); // tSrcTransition
    supply.settle();
    check(transmittedControl(tcpc, usbc::control_message_type::ps_rdy));
    txSuccess();

    // ... and Get_Sink_Cap while sourcing with its sink-role
    // capabilities (PE_DR_SRC_Give_Sink_Cap)
    deliver(partnerControl(usbc::control_message_type::get_sink_cap, usbc::power_role::sink,
                           usbc::data_role::ufp));
    check(transmittedType(tcpc) ==
          static_cast<std::uint8_t>(usbc::data_message_type::sink_capabilities));
    txSuccess();

    // PR_Swap, the partner's request (source -> sink): Accept,
    // tSrcTransition, supply off, Assert_Rd, our PS_RDY
    deliver(partnerControl(usbc::control_message_type::pr_swap, usbc::power_role::sink,
                           usbc::data_role::ufp));
    check(transmittedControl(tcpc, usbc::control_message_type::accept));
    txSuccess();
    check(timers.source_pe.armed); // tSrcTransition
    timers.source_pe.expire();
    check(supply.voltage == 0); // the supply is commanded off
    supply.settle();
    check(tcpc.pull == usbc::cc_pull::rd && !tcpc.sourcing); // Rd asserted
    check(transmittedControl(tcpc, usbc::control_message_type::ps_rdy));
    txSuccess();

    // the new source drives VBUS and reports PS_RDY: Attached.SNK, the
    // sink engine awaits the capabilities
    vbus.setVoltage(5000);
    deliver(partnerControl(usbc::control_message_type::ps_rdy, usbc::power_role::source,
                           usbc::data_role::dfp));
    check(tcpc.sinking && port.powerRole() == usbc::power_role::sink);
    check(tcpc.header_info.power == usbc::power_role::sink);
    check(port.dataRole() == usbc::data_role::ufp);
    check(timers.sink_pe.armed); // SinkWaitCapTimer runs again

    // detach as sink: Rp gone, both roles end
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    vbus.setVoltage(0);
    check(!port.powerRole());

    return failures;
}
