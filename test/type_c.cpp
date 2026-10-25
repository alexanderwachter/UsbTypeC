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
// The CC debounce runs on the cc_timer (the CcWatcher's), the
// connection states' timeouts on the timer.

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
    using sink = usbc::TypeCSink<mock_cc, mock_vbus, manual_timer, mock_tc_client>;

    mock_cc cc;
    mock_vbus vbus;
    mock_tc_client client;
    manual_timer timer;
    manual_timer cc_timer;
    sink tc{cc, vbus, timer, cc_timer, client};

    // construction rests in Disabled: nothing registered, nothing driven
    check(cc.callback == nullptr && vbus.monitored == usbc::vbus_level::unwatched);

    // start() applies Unattached.SNK: Rd presented, sink path off,
    // nothing measured (a source is detected on CC), callbacks
    // registered; the open line is detected at once - nothing to
    // debounce
    tc.start();
    check(cc.pull == usbc::cc_pull::rd && !vbus.sinking());
    check(vbus.monitored == usbc::vbus_level::unwatched);
    check(cc.callback != nullptr);
    check(!timer.armed && !cc_timer.armed);

    // the CC driver reports the line through the callback the sink
    // registered
    auto const ccAlert = [&] { cc.report(); };

    // source attaches: Rp seen at once, AttachWait.SNK debounces it for
    // tCCDebounce on the CC timer, VBUS arriving mid-debounce does not
    // restart it
    cc.line_state = {usbc::cc_state::snk_power_1a5, usbc::cc_state::snk_open};
    ccAlert();
    check(cc_timer.armed && !timer.armed);
    auto const starts_before = cc_timer.starts;
    vbus.setVoltage(5000);
    check(cc_timer.starts == starts_before && cc_timer.armed);

    cc_timer.expire(); // the Rp is detected, VBUS present: attach
    check(vbus.sinking());
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc1);
    check(client.advertisement == usbc::rp_value::p_1a5);
    check(vbus.monitored == usbc::vbus_level::sink_disconnect);

    // detach: VBUS drops below vSinkDisconnect - and the Rp still
    // there is debounced again by AttachWait.SNK
    vbus.setVoltage(0);
    check(!vbus.sinking());
    check(client.detached == 1);
    check(vbus.monitored == usbc::vbus_level::unwatched);
    check(cc_timer.armed);

    // attach with VBUS arriving after the debounce, flipped orientation
    cc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
    ccAlert();
    cc_timer.expire();
    check(!vbus.sinking() && client.attached == 1); // detected, waiting for VBUS
    check(vbus.monitored == usbc::vbus_level::safe5v);
    vbus.setVoltage(5000);
    check(vbus.sinking());
    check(client.attached == 2 && client.orientation == usbc::plug_orientation::cc2);
    check(client.advertisement == usbc::rp_value::p_3a0);
    vbus.setVoltage(0);
    check(client.detached == 2);

    // a CC change during the debounce restarts it
    cc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
    ccAlert();
    auto const restart_before = cc_timer.starts;
    cc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    check(cc_timer.starts == restart_before + 1);

    // ... and the line going open starts the shorter tPDDebounce: the
    // Rp returning in time resumes the tCCDebounce afresh
    cc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    check(cc_timer.starts == restart_before + 2);
    cc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    ccAlert();
    check(cc_timer.starts == restart_before + 3);
    // ... while open for tPDDebounce returns to Unattached.SNK (a
    // second open report is no change and keeps the running
    // tPDDebounce)
    cc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    ccAlert();
    check(cc_timer.starts == restart_before + 4 && cc_timer.armed);
    cc_timer.expire();
    check(!vbus.sinking() && client.attached == 2 && client.detached == 2);
    check(!cc_timer.armed);

    // open lines reported in Unattached.SNK are not a source: nothing
    // starts
    ccAlert();
    check(!cc_timer.armed);

    // both lines with Rp (debug accessory) is not an attach: detected
    // after tCCDebounce, the port waits in AttachWait.SNK for the line
    // to change, and the lines going open leave after tPDDebounce
    cc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_default};
    ccAlert();
    vbus.setVoltage(5000);
    cc_timer.expire();
    check(!vbus.sinking() && client.attached == 2 && !cc_timer.armed);
    vbus.setVoltage(0);
    cc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    ccAlert();
    check(cc_timer.armed);
    cc_timer.expire();
    check(client.attached == 2 && client.detached == 2);

    return failures;
}

int typeCSourceTests()
{
    using source = usbc::TypeCSource<mock_cc, mock_vbus, manual_timer, mock_src_client>;

    mock_cc cc;
    mock_vbus vbus;
    mock_src_client client;
    manual_timer timer;
    manual_timer cc_timer;
    source tc{cc, vbus, timer, cc_timer, usbc::rp_value::p_3a0, client};

    // construction rests in Disabled: nothing registered, nothing driven
    check(cc.callback == nullptr && vbus.monitored == usbc::vbus_level::unwatched);

    // start() applies Unattached.SRC: Rp with the configured
    // advertisement, source path off, nothing monitored while resting
    tc.start();
    check(cc.pull == usbc::cc_pull::rp && cc.rp == usbc::rp_value::p_3a0);
    check(!vbus.sourcing() && !vbus.discharging());
    check(vbus.monitored == usbc::vbus_level::unwatched);
    check(!cc_timer.armed);

    auto const ccAlert = [&] { cc.report(); };

    // sink attaches on CC2: tCCDebounce on the CC timer with a load on
    // VBUS, then VBUS applied (already at vSafe0V)
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    check(cc_timer.armed && vbus.discharging());
    check(!vbus.sourcing()); // not before the debounce completes
    cc_timer.expire();
    check(vbus.sourcing() && !vbus.discharging());
    check(client.attached == 1 && client.orientation == usbc::plug_orientation::cc2);
    vbus.setVoltage(5000); // the supply raises VBUS

    // the Rd gone is debounced within tSRCDisconnect: a dead-battery
    // sink's Rd gaps while it hands over from its port protector to
    // its own pull-down, and the attach survives the gap
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(cc_timer.armed && vbus.sourcing() && client.detached == 0);
    check(cc_timer.duration == usbc::tc::t_src_disconnect);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert(); // the Rd is back before the debounce ends
    check(!cc_timer.armed && vbus.sourcing() && client.detached == 0 && client.attached == 1);

    // detach: Rd removed for tSRCDisconnect, discharge to vSafe0V
    // before re-presenting
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(vbus.sourcing()); // not before the debounce completes
    cc_timer.expire();
    check(!vbus.sourcing() && vbus.discharging());
    check(client.detached == 1);
    vbus.setVoltage(0);         // discharge complete
    check(!vbus.discharging()); // Unattached.SRC again

    // attach with Ra on the other pin: still a single Rd, on CC1
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_ra};
    ccAlert();
    cc_timer.expire();
    check(vbus.sourcing() && client.attached == 2);
    check(client.orientation == usbc::plug_orientation::cc1);

    // detach with VBUS already at vSafe0V skips UnattachedWait.SRC
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire(); // tSRCDisconnect
    check(!vbus.sourcing() && !vbus.discharging());
    check(client.detached == 2);

    // Rd detected but VBUS not yet at vSafe0V: attach waits for it, the
    // load still applied
    vbus.setVoltage(5000);
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire();
    check(!vbus.sourcing() && client.attached == 2); // detected, waiting
    check(vbus.discharging());
    vbus.setVoltage(0);
    check(vbus.sourcing() && client.attached == 3);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire(); // tSRCDisconnect
    check(client.detached == 3);

    // both lines with Rd (debug accessory) is not an attach: detected
    // after tCCDebounce, the port waits in AttachWait.SRC for the line
    // to change
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_rd};
    ccAlert();
    cc_timer.expire();
    check(!vbus.sourcing() && client.attached == 3 && !cc_timer.armed);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();

    // a CC change during the debounce restarts it
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert();
    auto const restart_before = cc_timer.starts;
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    check(cc_timer.starts == restart_before + 1);
    // ... and the sink going away leaves at once, no debounce
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    check(!cc_timer.armed && !vbus.sourcing() && client.attached == 3);

    // the same after the detection, while waiting for vSafe0V: open
    // and Ra on the other pin is a cable without a sink
    vbus.setVoltage(5000);
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire();
    check(vbus.monitored == usbc::vbus_level::safe0v);
    cc.line_state = {usbc::cc_state::src_ra, usbc::cc_state::src_open};
    ccAlert();
    check(vbus.monitored == usbc::vbus_level::unwatched && client.attached == 3);
    vbus.setVoltage(0);

    // a sink arriving while VBUS is still discharging after a detach
    // is detected on reaching vSafe0V (UnattachedWait.SRC does not
    // monitor the pins) and debounced with the load applied
    cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire();
    check(vbus.sourcing() && client.attached == 4);
    vbus.setVoltage(5000);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire(); // tSRCDisconnect
    check(!vbus.sourcing() && vbus.discharging() && client.detached == 4);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    ccAlert(); // the next sink, during the discharge
    check(!cc_timer.armed);
    vbus.setVoltage(0);
    check(vbus.discharging() && cc_timer.armed); // AttachWait.SRC, tCCDebounce
    cc_timer.expire();
    check(vbus.sourcing() && client.attached == 5);
    check(client.orientation == usbc::plug_orientation::cc2);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();
    cc_timer.expire(); // tSRCDisconnect
    vbus.setVoltage(0);
    check(client.detached == 5);

    // a cable without a sink (Ra on one pin) does not enter
    // AttachWait.SRC; Ra on both pins (audio accessory) does, and
    // waits there for the line to change
    cc.line_state = {usbc::cc_state::src_ra, usbc::cc_state::src_open};
    ccAlert();
    check(!cc_timer.armed);
    cc.line_state = {usbc::cc_state::src_ra, usbc::cc_state::src_ra};
    ccAlert();
    check(cc_timer.armed);
    cc_timer.expire();
    check(!vbus.sourcing() && client.attached == 5 && !cc_timer.armed);
    cc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    ccAlert();

    // a sink already present at start() is reported from the CC status
    {
        mock_cc seeded_cc;
        mock_vbus seeded_vbus;
        mock_src_client seeded_client;
        manual_timer seeded_timer;
        manual_timer seeded_cc_timer;
        seeded_cc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        source seeded{seeded_cc, seeded_vbus, seeded_timer, seeded_cc_timer, seeded_client};
        seeded.start();
        check(seeded_cc_timer.armed); // debounce started right away
        seeded_cc_timer.expire();
        check(seeded_vbus.sourcing() && seeded_client.attached == 1);
        check(seeded_cc.rp == usbc::rp_value::usb_default);
    }

    return failures;
}
