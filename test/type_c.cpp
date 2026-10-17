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
// fsm::timeouts_within_bounds check next to each transition table.

using namespace std::chrono_literals;

namespace {

// --- timer policy (host/test implementation) --------------------------------
struct manual_timer {
    std::chrono::milliseconds duration{};
    fsm::timer_callback callback = nullptr;
    void* context = nullptr;
    bool armed = false;
    int starts = 0;

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context = ctx;
        armed = true;
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
    int attached = 0;
    int detached = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    usbc::rp_value advertisement = usbc::rp_value::usb_default;

    void notifyEntry(usbc::tc::attach_info info)
    {
        ++attached;
        orientation = info.orientation;
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
    int attached = 0;
    int detached = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;

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
static_assert(
    usbc::tc::orientationOf(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
    usbc::plug_orientation::cc2
);
static_assert(
    usbc::tc::advertisementOf(cc_status{cc_state::snk_open, cc_state::snk_power_3a0}) ==
    usbc::rp_value::p_3a0
);
static_assert(
    usbc::tc::advertisementOf(cc_status{cc_state::snk_default, cc_state::snk_open}) ==
    usbc::rp_value::usb_default
);

static_assert(usbc::tc::singleRd(cc_status{cc_state::src_rd, cc_state::src_open}));
static_assert(usbc::tc::singleRd(cc_status{cc_state::src_ra, cc_state::src_rd}));
static_assert(!usbc::tc::singleRd(cc_status{cc_state::src_rd, cc_state::src_rd}));
static_assert(!usbc::tc::singleRd(cc_status{cc_state::src_open, cc_state::src_ra}));
static_assert(
    usbc::tc::srcOrientationOf(cc_status{cc_state::src_open, cc_state::src_rd}) ==
    usbc::plug_orientation::cc2
);

// every watching state consumes its level's vbus event family
// the positive checks moved to test/compliance.cpp with the other
// table verifications; the negative probe below stays a unit test

// a watching state whose table drops a family event is rejected
namespace inconsistent {
struct deaf {
    static constexpr auto annotations = fsm::annotate(
        usbc::vbus_level{usbc::vbus_level::safe5v}
    );
};
struct poke {};
using table = fsm::transition_table<fsm::transition<fsm::from<deaf>, fsm::on<poke>, fsm::to<deaf>>>;
static_assert(!usbc::tc::watch_events_consistent_v<table>);
} // namespace inconsistent

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
    check(tcpc.callback == nullptr && vbus.monitored == usbc::vbus_level::unwatched);

    // start() applies Unattached.SNK: Rd presented, sink path off,
    // nothing measured (a source is detected on CC), callbacks registered
    tc.start();
    check(tcpc.pull == usbc::cc_pull::rd && !tcpc.sinking);
    check(vbus.monitored == usbc::vbus_level::unwatched);
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
    check(vbus.monitored == usbc::vbus_level::unwatched);

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

    // ... and the line going open restarts the shorter tPDDebounce:
    // the Rp returning in time resumes the tCCDebounce afresh
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    check(timer.starts == restart_before + 2);
    tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    check(timer.starts == restart_before + 3);
    // ... while open for tPDDebounce returns to Unattached.SNK (a
    // second open report keeps the running tPDDebounce)
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    ccAlert();
    check(timer.starts == restart_before + 4 && timer.armed);
    timer.expire();
    check(!tcpc.sinking && client.attached == 2 && client.detached == 2);

    // open lines reported in Unattached.SNK are not a source: nothing
    // starts
    ccAlert();
    check(!timer.armed);

    // both lines with Rp (debug accessory) is not an attach: the port
    // waits in AttachWait.SNK for the line to change, and the lines
    // going open leave after tPDDebounce
    tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_default};
    ccAlert();
    vbus.setVoltage(5000);
    timer.expire();
    check(!tcpc.sinking && client.attached == 2 && !timer.armed);
    vbus.setVoltage(0);
    tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    check(timer.armed);
    timer.expire();
    check(client.attached == 2 && client.detached == 2);

    // PD alert bits reach a client providing onPdAlert(); CC bits do not
    {
        mock_tcpc pd_tcpc;
        mock_vbus pd_vbus;
        mock_pd_forwarding_client pd_client;
        manual_timer pd_timer;
        usbc::TypeCSink<mock_tcpc, mock_vbus, manual_timer, mock_pd_forwarding_client>
            pd_tc{pd_tcpc, pd_vbus, pd_timer, pd_client};
        pd_tc.start();

        pd_tcpc.alerts |=
            usbc::alert_status::message_received | usbc::alert_status::cc_status_changed;
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
    check(tcpc.callback == nullptr && vbus.monitored == usbc::vbus_level::unwatched);

    // start() applies Unattached.SRC: Rp with the configured
    // advertisement, source path off, nothing monitored while resting
    // (AttachWait re-arms vSafe0V and gets the fresh arm-report)
    tc.start();
    check(tcpc.pull == usbc::cc_pull::rp && tcpc.rp == usbc::rp_value::p_3a0);
    check(!tcpc.sourcing && !vbus.discharging);
    check(vbus.monitored == usbc::vbus_level::unwatched);

    auto const ccAlert = [&] {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    };

    // sink attaches on CC2: debounce with a load on VBUS, then VBUS
    // applied (already at vSafe0V)
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    check(timer.armed && vbus.discharging);
    check(!tcpc.sourcing); // not before the debounce completes
    timer.expire();
    check(tcpc.sourcing && !vbus.discharging);
    check(tcpc.orientation == usbc::plug_orientation::cc2);
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc2);
    vbus.setVoltage(5000); // the supply raises VBUS

    // detach: Rd removed, discharge to vSafe0V before re-presenting
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!tcpc.sourcing && vbus.discharging);
    check(client.detached == 1);
    vbus.setVoltage(0);       // discharge complete
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

    // Rd stable but VBUS not yet at vSafe0V: attach waits for it, the
    // load still applied
    vbus.setVoltage(5000);
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    timer.expire();
    check(!tcpc.sourcing && client.attached == 2); // debounced, waiting
    check(vbus.discharging);
    vbus.setVoltage(0);
    check(tcpc.sourcing && client.attached == 3);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(client.detached == 3);

    // both lines with Rd (debug accessory) is not an attach: the port
    // waits in AttachWait.SRC for the line to change
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_rd};
    ccAlert();
    timer.expire();
    check(!tcpc.sourcing && client.attached == 3 && !timer.armed);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();

    // a CC change during the debounce restarts it
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    auto const restart_before = timer.starts;
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    check(timer.starts == restart_before + 1);
    // ... and the sink going away leaves at once, no debounce
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!timer.armed && !tcpc.sourcing && client.attached == 3);

    // the same after the debounce, while waiting for vSafe0V: open
    // and Ra on the other pin is a cable without a sink
    vbus.setVoltage(5000);
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    timer.expire();
    check(vbus.monitored == usbc::vbus_level::safe0v);
    tcpc.line_state = {usbc::cc_state::src_ra, usbc::cc_state::src_open};
    ccAlert();
    check(vbus.monitored == usbc::vbus_level::unwatched && client.attached == 3);
    vbus.setVoltage(0);

    // a sink arriving while VBUS is still discharging after a detach is
    // debounced as soon as vSafe0V is reached
    tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    timer.expire();
    check(tcpc.sourcing && client.attached == 4);
    vbus.setVoltage(5000);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!tcpc.sourcing && vbus.discharging && client.detached == 4);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert(); // the next sink, during the discharge
    check(!timer.armed);
    vbus.setVoltage(0);
    check(vbus.discharging && timer.armed); // AttachWait.SRC, tCCDebounce
    timer.expire();
    check(tcpc.sourcing && client.attached == 5);
    check(client.orientation == usbc::plug_orientation::cc2);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    vbus.setVoltage(0);
    check(client.detached == 5);

    // a cable without a sink (Ra on one pin) does not enter
    // AttachWait.SRC; Ra on both pins (audio accessory) does, and
    // waits there for the line to change
    ccAlert();
    check(!timer.armed);
    tcpc.line_state = {usbc::cc_state::src_ra, usbc::cc_state::src_ra};
    ccAlert();
    check(timer.armed);
    timer.expire();
    check(!tcpc.sourcing && client.attached == 5 && !timer.armed);
    tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();

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
