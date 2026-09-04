/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/TypeCSink.hpp>
#include <usbc/TypeCSource.hpp>

#include <chrono>
#include <print>
#include <source_location>

// No timeout-value assertions in these tests: every state timeout is
// formally verified against the spec ranges at compile time by the
// fsm::timeoutsWithinBounds check next to each transition table.

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

// --- user observer test doubles ---------------------------------------------
// Injected into the machine, watching the attached state's attachedInfo()
struct mock_tc_client : fsm::observing<mock_tc_client> {
    int attached                       = 0;
    int detached                       = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    usbc::rp_value advertisement       = usbc::rp_value::usb_default;

    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::tc::attach_info info)
    {
        ++attached;
        orientation   = info.orientation;
        advertisement = info.advertisement;
    }
    void notifyExit(usbc::tc::attach_info) { ++detached; }
};

// an observer may additionally take the PD alert bits the layer ignores
struct mock_pd_forwarding_client : mock_tc_client {
    usbc::alert_status forwarded = usbc::alert_status::none;
    void onPdAlert(usbc::alert_status alerts) { forwarded |= alerts; }
};

struct mock_src_client : fsm::observing<mock_src_client> {
    int attached                       = 0;
    int detached                       = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;

    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::plug_orientation o)
    {
        ++attached;
        orientation = o;
    }
    void notifyExit(usbc::plug_orientation) { ++detached; }
};

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

using usbc::cc_state;
using usbc::cc_status;

static_assert(usbc::tc::singleRp(cc_status{cc_state::snk_power_1a5, cc_state::snk_open}));
static_assert(!usbc::tc::singleRp(cc_status{cc_state::snk_open, cc_state::snk_open}));
static_assert(!usbc::tc::singleRp(cc_status{cc_state::snk_default, cc_state::snk_default}));
static_assert(usbc::tc::orientationOf(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
              usbc::plug_orientation::cc2);
static_assert(usbc::tc::advertisementOf(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
              usbc::rp_value::p_3a0);
static_assert(usbc::tc::advertisementOf(cc_status{cc_state::snk_default, cc_state::snk_open}) ==
              usbc::rp_value::usb_default);

static_assert(usbc::tc::singleRd(cc_status{cc_state::src_rd, cc_state::src_open}));
static_assert(usbc::tc::singleRd(cc_status{cc_state::src_ra, cc_state::src_rd}));
static_assert(!usbc::tc::singleRd(cc_status{cc_state::src_rd, cc_state::src_rd}));
static_assert(!usbc::tc::singleRd(cc_status{cc_state::src_open, cc_state::src_ra}));
static_assert(usbc::tc::srcOrientationOf(cc_status{cc_state::src_open, cc_state::src_rd}) ==
              usbc::plug_orientation::cc2);

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

int typeCTests()
{
    using sink = usbc::TypeCSink<mock_tcpc, mock_vbus, manual_timer, mock_tc_client>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    mock_tc_client client;
    manual_timer timer;
    sink tc{tcpc, vbus, timer, client};

    // construction rests in Disabled: nothing registered, nothing driven
    check(tcpc.callback == nullptr && !vbus.monitored);

    // start() applies Unattached.SNK: Rd presented, sink path off,
    // vSafe5V monitored, callbacks registered
    tc.start();
    check(tcpc.pull == usbc::cc_pull::rd && !tcpc.sinking);
    check(vbus.monitored == usbc::vbus_level::safe5v);
    check(tcpc.callback != nullptr);

    // a CC alert travels through the alert handler the sink registered
    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };

    // source attaches: Rp seen, debounce runs, VBUS arrives mid-debounce
    // without restarting it
    tcpc.line_state = {usbc::cc_state::snk_power_1a5, usbc::cc_state::snk_open};
    ccAlert();
    check(timer.armed);
    auto const starts_before = timer.starts;
    vbus.setVoltage(5000);
    check(timer.starts == starts_before && timer.armed);

    timer.expire(); // debounce complete: attach
    check(tcpc.sinking);
    check(tcpc.orientation == usbc::plug_orientation::cc1);
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc1);
    check(client.advertisement == usbc::rp_value::p_1a5);
    check(vbus.monitored == usbc::vbus_level::sink_disconnect);

    // detach: VBUS drops below vSinkDisconnect
    vbus.setVoltage(0);
    check(!tcpc.sinking);
    check(client.detached == 1);
    check(vbus.monitored == usbc::vbus_level::safe5v);

    // attach with VBUS arriving after the debounce, flipped orientation
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
    ccAlert();
    timer.expire();
    check(!tcpc.sinking && client.attached == 1); // debounced, waiting for VBUS
    vbus.setVoltage(5000);
    check(tcpc.sinking);
    check(client.attached == 2 && client.orientation == usbc::plug_orientation::cc2);
    check(client.advertisement == usbc::rp_value::p_3a0);
    vbus.setVoltage(0);
    check(client.detached == 2);

    // a CC change during the debounce restarts it
    tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
    ccAlert();
    auto const restart_before = timer.starts;
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    check(timer.starts == restart_before + 1);

    // ... and a line gone open at the timeout returns to Unattached.SNK
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    timer.expire();
    check(!tcpc.sinking && client.attached == 2 && client.detached == 2);

    // both lines with Rp (debug accessory) is not an attach
    tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_default};
    ccAlert();
    vbus.setVoltage(5000);
    timer.expire();
    check(!tcpc.sinking && client.attached == 2);
    vbus.setVoltage(0);

    // PD alert bits reach a client providing onPdAlert(); CC bits do not
    {
        mock_tcpc pd_tcpc;
        mock_vbus pd_vbus;
        mock_pd_forwarding_client pd_client;
        manual_timer pd_timer;
        usbc::TypeCSink<mock_tcpc, mock_vbus, manual_timer, mock_pd_forwarding_client> pd_tc{
            pd_tcpc, pd_vbus, pd_timer, pd_client};
        pd_tc.start();

        pd_tcpc.alerts |= usbc::alert_status::message_received |
                          usbc::alert_status::cc_status_changed;
        pd_tcpc.callback(pd_tcpc.context);
        check(pd_client.forwarded == usbc::alert_status::message_received);
    }

    return failures;
}

int typeCSourceTests()
{
    using source = usbc::TypeCSource<mock_tcpc, mock_vbus, manual_timer, mock_src_client>;

    mock_tcpc tcpc;
    mock_vbus vbus;
    mock_src_client client;
    manual_timer timer;
    source tc{tcpc, vbus, timer, usbc::rp_value::p_3a0, client};

    // construction rests in Disabled: nothing registered, nothing driven
    check(tcpc.callback == nullptr && !vbus.monitored);

    // start() applies Unattached.SRC: Rp with the configured
    // advertisement, source path off, vSafe0V monitored and reported
    tc.start();
    check(tcpc.pull == usbc::cc_pull::rp && tcpc.rp == usbc::rp_value::p_3a0);
    check(!tcpc.sourcing && !vbus.discharging);
    check(vbus.monitored == usbc::vbus_level::safe0v);

    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };

    // sink attaches on CC2: debounce, then VBUS applied (already at vSafe0V)
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    check(timer.armed);
    check(!tcpc.sourcing); // not before the debounce completes
    timer.expire();
    check(tcpc.sourcing);
    check(tcpc.orientation == usbc::plug_orientation::cc2);
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc2);
    vbus.setVoltage(5000); // the supply raises VBUS

    // detach: Rd removed, discharge to vSafe0V before re-presenting
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!tcpc.sourcing && vbus.discharging);
    check(client.detached == 1);
    vbus.setVoltage(0); // discharge complete
    check(!vbus.discharging); // Unattached.SRC again

    // attach with Ra on the other pin: still a single Rd, on CC1
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_ra};
    ccAlert();
    timer.expire();
    check(tcpc.sourcing && client.attached == 2);
    check(client.orientation == usbc::plug_orientation::cc1);

    // detach with VBUS already at vSafe0V skips UnattachedWait.SRC
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!tcpc.sourcing && !vbus.discharging);
    check(client.detached == 2);

    // Rd stable but VBUS not yet at vSafe0V: attach waits for it
    vbus.setVoltage(5000);
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    timer.expire();
    check(!tcpc.sourcing && client.attached == 2); // debounced, waiting
    vbus.setVoltage(0);
    check(tcpc.sourcing && client.attached == 3);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(client.detached == 3);

    // both lines with Rd (debug accessory) is not an attach
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_rd};
    ccAlert();
    timer.expire();
    check(!tcpc.sourcing && client.attached == 3);

    // a CC change during the debounce restarts it
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    auto const restart_before = timer.starts;
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    check(timer.starts == restart_before + 1);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    timer.expire();
    check(!tcpc.sourcing && client.attached == 3);

    // a sink already present at start() is seeded from CC status
    {
        mock_tcpc seeded_tcpc;
        mock_vbus seeded_vbus;
        mock_src_client seeded_client;
        manual_timer seeded_timer;
        seeded_tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        source seeded{seeded_tcpc, seeded_vbus, seeded_timer, seeded_client};
        seeded.start();
        check(seeded_timer.armed); // debounce started right away
        seeded_timer.expire();
        check(seeded_tcpc.sourcing && seeded_client.attached == 1);
        check(seeded_tcpc.rp == usbc::rp_value::usb_default);
    }

    return failures;
}
