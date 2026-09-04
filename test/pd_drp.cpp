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
    bool setLimit(usbc::millivolt, usbc::milliamp)
    {
        ++limits;
        return true;
    }
    void onContract(usbc::millivolt, usbc::milliamp) {}
    void onContractLost() {}
};

struct mock_supply {
    usbc::supply_callback callback = nullptr;
    void* context                  = nullptr;
    int outputs                    = 0;

    void setCallback(usbc::supply_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt, usbc::milliamp)
    {
        ++outputs;
        return true;
    }
};
static_assert(usbc::concepts::source_supply<mock_supply>);

struct mock_source_power : usbc::SourcePower<mock_source_power> {
    void onContract(usbc::millivolt, usbc::milliamp) {}
    void onContractLost() {}
};

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

// The facade wires the port from domain pieces alone: the routing,
// engine activation, and message-header bookkeeping are internal
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

    port.start();
    check(tcpc.pull == usbc::cc_pull::rd); // toggling, sink phase first
    check(!port.powerRole() && !port.dataRole());

    // a charger appears: the port resolves to sink, the sink engine
    // takes over (its SinkWaitCap runs), the header says sink/UFP
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    vbus.setVoltage(5000);
    timers.tc.expire(); // tCCDebounce
    check(tcpc.sinking && port.powerRole() == usbc::power_role::sink);
    check(port.dataRole() == usbc::data_role::ufp);
    check(tcpc.header_info.power == usbc::power_role::sink);
    check(tcpc.header_info.data == usbc::data_role::ufp);
    check(timers.sink_pe.armed); // the sink engine is live

    // data role swap: header follows, nothing else changes
    check(port.swapDataRole());
    check(port.dataRole() == usbc::data_role::dfp);
    check(tcpc.header_info.data == usbc::data_role::dfp);
    check(tcpc.sinking);

    // power role swap: the source engine takes over and advertises,
    // the swapped data role survives
    check(port.beginSwapToSource());
    vbus.setVoltage(0); // the old source turns off
    check(port.completeSwap());
    check(tcpc.sourcing && port.powerRole() == usbc::power_role::source);
    check(tcpc.header_info.power == usbc::power_role::source);
    check(port.dataRole() == usbc::data_role::dfp);
    check(tcpc.header_info.data == usbc::data_role::dfp);
    check(tcpc.transmit_count > 0); // Source_Capabilities on the wire

    // detach as source: Rd gone, both roles end
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!tcpc.sourcing && !port.powerRole());

    return failures;
}
