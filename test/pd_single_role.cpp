/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The single-role facades at message level: the paths only a full
 * port integration exercises - the hard-reset window holding the
 * attach through the VBUS cycle, and the Error Recovery escalation
 * after an exhausted HardResetCounter.
 */

#include "mocks.hpp"

#include <usbc/PdSink.hpp>
#include <usbc/PdSource.hpp>

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

// --- the user's domain pieces, as test doubles ------------------------------
constexpr std::array sink_capabilities{usbc::sink_capability{5000, 3000}};
constexpr std::array source_caps{usbc::pdo::makeFixedSource(5000, 1500)};

struct mock_sink_power : usbc::SinkPower<mock_sink_power> {
    int contracts = 0;
    int losses = 0;
    bool setLimit(usbc::millivolt, usbc::milliamp) { return true; }
    void onContract(usbc::millivolt, usbc::milliamp) { ++contracts; }
    void onContractLost() { ++losses; }
};

struct mock_supply {
    usbc::supply_ready_callback callback = nullptr;
    void* context = nullptr;
    usbc::millivolt voltage = 5000;

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
    {
        callback = cb;
        context = ctx;
    }
    bool setOutput(usbc::millivolt v, usbc::milliamp)
    {
        voltage = v;
        return true;
    }
    void settle() { callback(context, true); } // the regulator is in tolerance
};

struct mock_source_power : usbc::SourcePower<mock_source_power> {
    int contracts = 0;
    void onContract(usbc::millivolt, usbc::milliamp) { ++contracts; }
    void onContractLost() {}
};

// --- partner messages --------------------------------------------------------
int next_id = 0;

usbc::pd_message partnerMessage(
    std::uint8_t message_type,
    std::uint8_t data_objects,
    usbc::power_role power,
    usbc::data_role data
)
{
    return {
        .sop = usbc::sop_type::sop,
        .header =
            usbc::pd_header{
                .message_type = message_type,
                .port_data_role = data,
                .revision = usbc::pd_revision::rev_3_x,
                .port_power_role = power,
                .message_id = static_cast<std::uint8_t>(next_id++ & 0x7u),
                .num_data_objects = data_objects
            }
                .encode()
    };
}

usbc::pd_message
partnerControl(usbc::control_message_type type, usbc::power_role power, usbc::data_role data)
{
    return partnerMessage(static_cast<std::uint8_t>(type), 0, power, data);
}

void putObject(usbc::pd_message& message, std::uint32_t object)
{
    auto const offset = message.payload_size;
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

int pdSinkFacadeTests()
{
    using Port =
        usbc::PdSink<mock_tcpc, mock_vbus, manual_timer, usbc::PowerPolicy, mock_sink_power>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    usbc::pd_sink_timers<manual_timer> timers;
    usbc::PowerPolicy policy{2500, 15000};
    mock_sink_power power;

    Port port{tcpc, vbus, timers, sink_capabilities, policy, power};

    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };
    auto const deliver = [&](usbc::pd_message const& message) { tcpc.injectMessage(message); };
    auto const txSuccess = [&] {
        tcpc.alerts |= usbc::alert_status::transmit_success;
        tcpc.callback(tcpc.context);
    };
    auto const negotiate = [&] { // the source's caps through to PS_RDY
        auto caps = partnerMessage(
            static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities),
            1,
            usbc::power_role::source,
            usbc::data_role::dfp
        );
        putObject(caps, usbc::pdo::makeFixedSource(5000, 3000));
        deliver(caps);
        check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::data_message_type::request));
        txSuccess();
        deliver(partnerControl(
            usbc::control_message_type::accept,
            usbc::power_role::source,
            usbc::data_role::dfp
        ));
        deliver(partnerControl(
            usbc::control_message_type::ps_rdy,
            usbc::power_role::source,
            usbc::data_role::dfp
        ));
    };

    port.start();
    check(tcpc.pull == usbc::cc_pull::rd);

    // a source appears: attach, negotiate, explicit contract
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    vbus.setVoltage(5000);
    timers.tc.expire(); // tCCDebounce
    check(tcpc.sinking);
    check(timers.pe.armed); // SinkWaitCapTimer runs
    negotiate();
    check(power.contracts == 1);

    // a sink-only port's policy answers none of the swap questions:
    // the partner's PR_Swap is Not_Supported, the contract stands
    deliver(partnerControl(
        usbc::control_message_type::pr_swap,
        usbc::power_role::source,
        usbc::data_role::dfp
    ));
    check(
        transmittedType(tcpc) ==
        static_cast<std::uint8_t>(usbc::control_message_type::not_supported)
    );
    txSuccess();
    check(power.contracts == 1);

    // the source hard-resets: the window holds the attach while VBUS
    // legitimately cycles - no detach, the connection resumes and
    // renegotiates instead of resolving from scratch
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    tcpc.callback(tcpc.context);
    check(power.losses == 1);              // back to vSafe5V defaults
    check(tcpc.pull == usbc::cc_pull::rd); // Rd stays presented
    vbus.setVoltage(0);                    // the source cycles VBUS ...
    check(tcpc.pull == usbc::cc_pull::rd); // ... still not a detach
    vbus.setVoltage(5000);                 // ... and it returns
    check(timers.pe.armed);                // the engine awaits the caps
    negotiate();
    check(power.contracts == 2);

    // the source turns silent but keeps cycling VBUS on every hard
    // reset: the counter runs out and the port escalates to Type-C
    // Error Recovery instead of resetting forever. A received hard
    // reset first puts the engine back to awaiting capabilities
    tcpc.alerts |= usbc::alert_status::hard_reset_received;
    tcpc.callback(tcpc.context);
    vbus.setVoltage(0);
    vbus.setVoltage(5000);
    check(timers.pe.armed); // SinkWaitCapTimer runs, the source is mute
    for (int reset = 0; reset <= usbc::spec::n_hard_reset_count; ++reset) {
        timers.pe.expire(); // SinkWaitCap / NoResponse: hard reset
        check(tcpc.last_signal == usbc::transmit_signal::hard_reset);
        tcpc.last_signal.reset();
        txSuccess();           // the PHY confirms: the window opens
        vbus.setVoltage(0);    // the source cycles VBUS
        vbus.setVoltage(5000); // the attach resumes, caps awaited
        check(timers.pe.armed);
    }
    timers.pe.expire();                      // the counter is spent
    check(!tcpc.last_signal.has_value());    // no further hard reset
    check(tcpc.pull == usbc::cc_pull::open); // Error Recovery: open terminations
    timers.tc.expire();                      // tErrorRecovery over
    check(tcpc.pull == usbc::cc_pull::rd);   // resolution restarts

    return failures;
}

int pdSourceFacadeTests()
{
    using Port = usbc::PdSource<
        mock_tcpc,
        mock_vbus,
        manual_timer,
        usbc::RequestPolicy,
        mock_supply,
        mock_source_power>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    usbc::pd_source_timers<manual_timer> timers;
    usbc::RequestPolicy policy;
    mock_supply supply;
    mock_source_power power;

    Port port{tcpc, vbus, timers, source_caps, policy, supply, power, usbc::rp_value::p_1a5};

    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };
    auto const deliver = [&](usbc::pd_message const& message) { tcpc.injectMessage(message); };
    auto const txSuccess = [&] {
        tcpc.alerts |= usbc::alert_status::transmit_success;
        tcpc.callback(tcpc.context);
    };

    port.start();
    check(tcpc.pull == usbc::cc_pull::rp && tcpc.rp == usbc::rp_value::p_1a5);

    // a sink appears: attach, advertise, grant its Request
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    timers.tc.expire(); // tCCDebounce (VBUS at vSafe0V)
    check(tcpc.sourcing);
    check(
        transmittedType(tcpc) ==
        static_cast<std::uint8_t>(usbc::data_message_type::source_capabilities)
    );
    txSuccess();

    auto request = partnerMessage(
        static_cast<std::uint8_t>(usbc::data_message_type::request),
        1,
        usbc::power_role::sink,
        usbc::data_role::ufp
    );
    putObject(request, usbc::pdo::makeFixedRequest(1, 1000, 1000, false));
    deliver(request);
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::accept));
    txSuccess();
    timers.pe.expire(); // tSrcTransition: the supply is driven now
    supply.settle();    // the output is in tolerance: PS_RDY goes out
    check(transmittedType(tcpc) == static_cast<std::uint8_t>(usbc::control_message_type::ps_rdy));
    txSuccess();
    check(power.contracts == 1);

    // a source-only port's policy answers none of the swap questions:
    // the partner's DR_Swap is Not_Supported
    deliver(partnerControl(
        usbc::control_message_type::dr_swap,
        usbc::power_role::sink,
        usbc::data_role::ufp
    ));
    check(
        transmittedType(tcpc) ==
        static_cast<std::uint8_t>(usbc::control_message_type::not_supported)
    );
    txSuccess();

    // a mute PD sink (GoodCRCs, never Requests): hard resets until the
    // counter is spent, then Type-C Error Recovery - a fresh fixture,
    // since the loop starts from the awaiting-Request state
    {
        mock_tcpc silent_tcpc;
        mock_vbus silent_vbus;
        usbc::pd_source_timers<manual_timer> silent_timers;
        mock_supply silent_supply;
        mock_source_power silent_power;
        Port silent{
            silent_tcpc,
            silent_vbus,
            silent_timers,
            source_caps,
            policy,
            silent_supply,
            silent_power,
            usbc::rp_value::p_1a5
        };
        auto const confirm = [&] {
            silent_tcpc.alerts |= usbc::alert_status::transmit_success;
            silent_tcpc.callback(silent_tcpc.context);
        };

        silent.start();
        silent_tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        silent_tcpc.alerts |= usbc::alert_status::cc_status_changed;
        silent_tcpc.callback(silent_tcpc.context);
        silent_timers.tc.expire(); // attach: the capabilities go out
        confirm();                 // GoodCRC: a PD sink is present

        for (int reset = 0; reset <= usbc::spec::n_hard_reset_count; ++reset) {
            silent_timers.pe.expire(); // SenderResponse, no Request
            check(silent_tcpc.last_signal == usbc::transmit_signal::hard_reset);
            silent_tcpc.last_signal.reset();
            confirm();                 // the PHY confirms the hard reset
            silent_supply.settle();    // VBUS removed
            silent_timers.pe.expire(); // tSrcRecover
            silent_supply.settle();    // defaults back: the caps go out
            confirm();                 // GoodCRC again
        }
        silent_timers.pe.expire();                      // the counter is spent
        check(!silent_tcpc.last_signal.has_value());    // no further hard reset
        check(silent_tcpc.pull == usbc::cc_pull::open); // Error Recovery
        silent_timers.tc.expire();                      // tErrorRecovery over
        check(silent_tcpc.pull == usbc::cc_pull::rp);   // resolution restarts
    }

    return failures;
}
