/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/TypeC.hpp>

#include <chrono>
#include <print>
#include <source_location>

using namespace std::chrono_literals;

namespace {

// --- timer policy (host/test implementation) --------------------------------
struct manual_timer {
    std::chrono::milliseconds duration{};
    fsm::timer_callback callback = nullptr;
    void* context                = nullptr;
    bool armed                   = false;
    int starts                   = 0;

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context  = ctx;
        armed    = true;
        ++starts;
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

// --- user notification test double ------------------------------------------
struct mock_tc_client {
    int attached                       = 0;
    int detached                       = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    usbc::rp_value advertisement       = usbc::rp_value::usb_default;

    void on_attached(usbc::plug_orientation o, usbc::rp_value rp)
    {
        ++attached;
        orientation   = o;
        advertisement = rp;
    }
    void on_detached() { ++detached; }
};
static_assert(usbc::concepts::tc_sink_client<mock_tc_client>);

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

using usbc::cc_state;
using usbc::cc_status;

static_assert(usbc::tc::single_rp(cc_status{cc_state::snk_power_1a5, cc_state::snk_open}));
static_assert(!usbc::tc::single_rp(cc_status{cc_state::snk_open, cc_state::snk_open}));
static_assert(!usbc::tc::single_rp(cc_status{cc_state::snk_default, cc_state::snk_default}));
static_assert(usbc::tc::orientation_of(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
              usbc::plug_orientation::cc2);
static_assert(usbc::tc::advertisement_of(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
              usbc::rp_value::p_3a0);
static_assert(usbc::tc::advertisement_of(cc_status{cc_state::snk_default, cc_state::snk_open}) ==
              usbc::rp_value::usb_default);

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

} // namespace

int type_c_tests()
{
    using sink = usbc::type_c_sink<mock_tcpc, mock_vbus, manual_timer, mock_tc_client>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    mock_tc_client client;
    sink tc{tcpc, vbus, client};
    vbus.set_callback([](void* ctx, bool met) { static_cast<sink*>(ctx)->vbus_event(met); }, &tc);
    auto& timer = tc.timer();

    // construction applied Unattached.SNK: Rd presented, sink path off,
    // vSafe5V monitored
    check(tcpc.pull == usbc::cc_pull::rd && !tcpc.sinking);
    check(vbus.monitored == usbc::vbus_level::safe5v);

    // source attaches: Rp seen, debounce runs, VBUS arrives mid-debounce
    // without restarting it
    tcpc.line_state = {usbc::cc_state::snk_power_1a5, usbc::cc_state::snk_open};
    tc.cc_alert();
    check(timer.armed && timer.duration == usbc::tc::t_cc_debounce);
    auto const starts_before = timer.starts;
    vbus.set_voltage(5000);
    check(timer.starts == starts_before && timer.armed);

    timer.expire(); // debounce complete: attach
    check(tcpc.sinking);
    check(tcpc.orientation == usbc::plug_orientation::cc1);
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc1);
    check(client.advertisement == usbc::rp_value::p_1a5);
    check(vbus.monitored == usbc::vbus_level::sink_disconnect);

    // detach: VBUS drops below vSinkDisconnect
    vbus.set_voltage(0);
    check(!tcpc.sinking);
    check(client.detached == 1);
    check(vbus.monitored == usbc::vbus_level::safe5v);

    // attach with VBUS arriving after the debounce, flipped orientation
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
    tc.cc_alert();
    timer.expire();
    check(!tcpc.sinking && client.attached == 1); // debounced, waiting for VBUS
    vbus.set_voltage(5000);
    check(tcpc.sinking);
    check(client.attached == 2 && client.orientation == usbc::plug_orientation::cc2);
    check(client.advertisement == usbc::rp_value::p_3a0);
    vbus.set_voltage(0);
    check(client.detached == 2);

    // a CC change during the debounce restarts it
    tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
    tc.cc_alert();
    auto const restart_before = timer.starts;
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    tc.cc_alert();
    check(timer.starts == restart_before + 1);

    // ... and a line gone open at the timeout returns to Unattached.SNK
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    tc.cc_alert();
    timer.expire();
    check(!tcpc.sinking && client.attached == 2 && client.detached == 2);

    // both lines with Rp (debug accessory) is not an attach
    tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_default};
    tc.cc_alert();
    vbus.set_voltage(5000);
    timer.expire();
    check(!tcpc.sinking && client.attached == 2);
    vbus.set_voltage(0);

    return failures;
}
