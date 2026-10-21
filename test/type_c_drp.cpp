/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/TypeCDrp.hpp>

#include <chrono>
#include <optional>
#include <print>
#include <source_location>

// No timeout-value assertions in these tests: every state timeout is
// formally verified against the spec ranges at compile time by the
// fsm::timeouts_within_bounds check next to each transition table.
// The CC debounce runs on the cc_timer (the CcWatcher's), the
// connection states' timeouts - the toggle, the Try walls - on the
// timer.

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

// --- user observer test double ----------------------------------------------
// Watches both attached states' attachedInfo(); the info type encodes
// the attached role
struct mock_drp_client : fsm::observing<mock_drp_client> {
    int attached_snk = 0;
    int attached_src = 0;
    int detached = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    usbc::rp_value advertisement = usbc::rp_value::usb_default;

    void notifyEntry(usbc::tc::attach_info info)
    {
        ++attached_snk;
        orientation = info.orientation;
        advertisement = info.advertisement;
    }
    void notifyExit(usbc::tc::attach_info) { ++detached; }
    void notifyEntry(usbc::plug_orientation o)
    {
        ++attached_src;
        orientation = o;
    }
    void notifyExit(usbc::plug_orientation) { ++detached; }
};

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

constexpr auto& timing = usbc::default_drp_timing;

// the toggle slices split tDRP by dcSRC.DRP
static_assert(usbc::tc::drp::t_src_slice<timing> == timing.t_drp * timing.dc_src / 100);
static_assert(
    usbc::tc::drp::unattached_src<timing>::timeout +
        usbc::tc::drp::unattached_snk<timing>::timeout ==
    timing.t_drp
);

// a custom timing overrides the defaults with designated initializers
inline constexpr usbc::drp_timing sink_heavy_timing{
    .t_drp = std::chrono::milliseconds{100},
    .dc_src = 30
};
static_assert(usbc::tc::drp::unattached_src<sink_heavy_timing>::timeout == 30ms);
static_assert(usbc::tc::drp::unattached_snk<sink_heavy_timing>::timeout == 70ms);
static_assert(usbc::tc::drp::timingWithinSpec<sink_heavy_timing>());

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

struct fixture {
    mock_tcpc tcpc;
    mock_vbus vbus;
    mock_drp_client client;
    manual_timer timer;    // the connection states: the toggle, the Try walls, the standbys
    manual_timer cc_timer; // the CC debounce

    void ccAlert()
    {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    }
};

// An observer hearing the data role flips
struct mock_data_role_listener {
    std::optional<usbc::data_role> data_seen{};

    void onDataRole(usbc::data_role role) { data_seen = role; }
};

// The role lock: answers the table's sourcing_allowed question
struct mock_role_lock {
    bool sourcing = true;
    bool check(usbc::sourcing_allowed) const { return sourcing; }
};
static_assert(fsm::concepts::answers_stateless_guard<mock_role_lock, usbc::sourcing_allowed>);
static_assert(
    !fsm::concepts::answers_stateless_guard<mock_data_role_listener, usbc::sourcing_allowed>
);
static_assert(usbc::sourcing_allowed::check()); // the default without a lock

} // namespace

int typeCDrpTests()
{
    constexpr auto& timing = usbc::default_drp_timing;
    using drp = usbc::TypeCDrp<
        mock_tcpc,
        mock_vbus,
        manual_timer,
        timing,
        usbc::drp_preference::none,
        mock_drp_client>;

    fixture f;
    drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, usbc::rp_value::p_1a5, f.client};

    // construction rests in Disabled: nothing registered, nothing driven
    check(f.tcpc.callback == nullptr && f.vbus.monitored == usbc::vbus_level::unwatched);

    // start() goes live toggling at Rd with the sink slice armed; the
    // open line is detected at once, nothing to debounce
    tc.start();
    check(f.tcpc.pull == usbc::cc_pull::rd && !f.tcpc.sinking && !f.tcpc.sourcing);
    check(f.vbus.monitored == usbc::vbus_level::unwatched); // toggling: nothing measured
    check(f.timer.armed && !f.cc_timer.armed);

    // the toggle alternates Rd and Rp with the configured advertisement
    f.timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rp && f.tcpc.rp == usbc::rp_value::p_1a5);
    check(f.vbus.monitored == usbc::vbus_level::unwatched); // resting Rp: nothing measured
    check(f.timer.armed);
    f.timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rd);
    check(f.timer.armed);

    // a source appears during the Rd phase: AttachWait.SNK stops the
    // toggle and debounces the Rp
    f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    f.ccAlert();
    check(!f.timer.armed && f.cc_timer.armed);
    f.vbus.setVoltage(5000);
    f.cc_timer.expire();
    check(f.tcpc.sinking && !f.tcpc.sourcing);
    check(f.client.attached_snk == 1 && f.client.attached_src == 0);
    check(f.client.orientation == usbc::plug_orientation::cc1);
    check(f.client.advertisement == usbc::rp_value::p_3a0);
    check(!f.timer.armed); // attached: no toggle running

    // detach: the source's Rp still there is debounced again by
    // AttachWait.SNK; gone for tPDDebounce, a DRP goes on with the Rp
    // phase (4.5.2.2.4.2)
    f.vbus.setVoltage(0);
    check(!f.tcpc.sinking && f.client.detached == 1);
    check(f.tcpc.pull == usbc::cc_pull::rd && f.cc_timer.armed && !f.timer.armed);
    f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    f.ccAlert();
    f.cc_timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed);

    // a sink appears during the Rp phase: source attach flow
    f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    f.ccAlert();
    f.cc_timer.expire();
    check(f.tcpc.sourcing && !f.tcpc.sinking);
    check(f.client.attached_src == 1);
    check(f.client.orientation == usbc::plug_orientation::cc2);
    f.vbus.setVoltage(5000); // the supply raises VBUS

    // the sink leaves: Rd at once while VBUS is removed, then toggling
    // resumes at Rd (Unattached.SNK)
    f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    f.ccAlert();
    check(!f.tcpc.sourcing && f.vbus.discharging);
    check(f.tcpc.pull == usbc::cc_pull::rd && !f.timer.armed);
    check(f.client.detached == 2);
    f.vbus.setVoltage(0);
    check(!f.vbus.discharging);
    check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);

    // open lines reported during the Rd phase are not a source: the
    // toggle runs on undisturbed
    auto const slice_started = f.timer.starts;
    f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    f.ccAlert();
    check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.starts == slice_started);

    // an Rp that vanishes during its debounce: tPDDebounce, then the
    // DRP continues with the Rp phase (4.5.2.2.4.2)
    f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
    f.ccAlert();
    f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    f.ccAlert();
    f.cc_timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed);
    check(f.client.attached_snk == 1 && f.client.attached_src == 1);

    // an Rd that vanishes during its debounce: the DRP continues with
    // the Rd phase at once
    f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
    f.ccAlert();
    f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    f.ccAlert();
    check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
    check(f.client.attached_snk == 1 && f.client.attached_src == 1);

    // the sink that left turned into a source while VBUS was being
    // removed: not monitored meanwhile, detected once VBUS is down and
    // debounced as the sink flow would, no toggle
    {
        fixture g;
        drp tc2{g.tcpc, g.vbus, g.timer, g.cc_timer, usbc::rp_value::p_1a5, g.client};
        tc2.start();
        g.timer.expire(); // Rp phase
        g.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        g.ccAlert();
        g.cc_timer.expire();
        check(g.tcpc.sourcing && g.client.attached_src == 1);
        g.vbus.setVoltage(5000);

        g.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        g.ccAlert(); // the sink is gone: Rd, VBUS being removed
        check(g.tcpc.pull == usbc::cc_pull::rd && g.vbus.discharging);
        g.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
        g.ccAlert(); // ... and the partner presents Rp meanwhile
        check(!g.timer.armed && !g.cc_timer.armed);
        g.vbus.setVoltage(0); // vSafe0V: AttachWait.SNK, tCCDebounce
        check(!g.vbus.discharging && g.cc_timer.armed && !g.timer.armed);
        g.vbus.setVoltage(5000);
        g.cc_timer.expire();
        check(g.tcpc.sinking && g.client.attached_snk == 1);
        check(g.client.orientation == usbc::plug_orientation::cc1);
    }

    // ... and its VBUS arriving only after the debounce is a fresh
    // attach as sink, not the source connection resumed
    {
        fixture g;
        drp tc2{g.tcpc, g.vbus, g.timer, g.cc_timer, usbc::rp_value::p_1a5, g.client};
        tc2.start();
        g.timer.expire(); // Rp phase
        g.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        g.ccAlert();
        g.cc_timer.expire();
        check(g.tcpc.sourcing && tc2.dataRole() == usbc::data_role::dfp);
        g.vbus.setVoltage(5000);

        g.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        g.ccAlert();
        g.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
        g.ccAlert();
        g.vbus.setVoltage(0); // vSafe0V: AttachWait.SNK
        g.cc_timer.expire();  // tCCDebounce, no VBUS yet
        check(!g.tcpc.sinking && g.client.attached_snk == 0);
        g.vbus.setVoltage(5000);
        check(g.tcpc.sinking && g.client.attached_snk == 1);
        check(g.client.orientation == usbc::plug_orientation::cc2);
        check(tc2.dataRole() == usbc::data_role::ufp);
    }

    // ErrorRecovery restarts a DRP without preference at Rd (4.5.2.2.2.2)
    check(tc.errorRecovery());
    check(f.tcpc.pull == usbc::cc_pull::open && f.timer.armed);
    f.timer.expire(); // tErrorRecovery -> Unattached.SNK
    check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);

    return failures;
}

int typeCDrpTrySrcTests()
{
    constexpr auto& timing = usbc::default_drp_timing;
    using drp = usbc::TypeCDrp<
        mock_tcpc,
        mock_vbus,
        manual_timer,
        timing,
        usbc::drp_preference::source,
        mock_drp_client>;

    // a source appeared and VBUS is present: where the sink flow would
    // attach, a source-preferring port enters Try.SRC
    auto const trySrc = [](fixture& f, drp& tc) {
        tc.start();
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire(); // tCCDebounce: would attach as sink -> Try.SRC
    };

    // a source-preferring port answers a sink attach with Try.SRC and
    // resolves to Attached.SRC when the partner presents Rd
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        check(!f.tcpc.sinking && !f.tcpc.sourcing);
        check(f.tcpc.pull == usbc::cc_pull::rp);
        check(f.client.attached_snk == 0);
        check(f.timer.armed && !f.cc_timer.armed); // tDRPTry runs, no report since the Rp flip

        // the partner flips to Rd (and stops sourcing VBUS)
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        check(f.cc_timer.armed); // tTryCCDebounce
        f.cc_timer.expire();
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        check(f.client.orientation == usbc::plug_orientation::cc1);
        check(!f.cc_timer.armed && !f.timer.armed);
    }

    // no Rd within tDRPTry, VBUS at vSafe0V: TryWait.SNK attaches as
    // sink on VBUS, but not before tCCDebounce since its entry
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        f.timer.expire(); // tDRPTry: no Rd -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && !f.cc_timer.armed); // tCCDebounce

        // the partner keeps its Rp (detected at once) and sources
        // again: Attached.SNK once the debounce is up
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        check(!f.tcpc.sinking && f.client.attached_snk == 0);
        f.timer.expire();
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.advertisement == usbc::rp_value::usb_default);
    }

    // ... or on the VBUS report arriving after the debounce
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        f.timer.expire(); // tDRPTry -> TryWait.SNK
        f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire(); // tCCDebounce, no VBUS yet
        check(!f.tcpc.sinking && !f.timer.armed);
        f.vbus.setVoltage(5000);
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.advertisement == usbc::rp_value::p_3a0);
    }

    // both pins open in TryWait.SNK: tPDDebounce, then back to toggling
    // - a flap within it does not restart the tCCDebounce
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        f.timer.expire(); // tDRPTry -> TryWait.SNK
        auto const debounce_started = f.timer.starts;

        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        check(f.cc_timer.armed); // tPDDebounce
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert(); // the Rp is back in time: detected at once
        check(!f.cc_timer.armed && f.timer.armed && f.timer.starts == debounce_started);

        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.cc_timer.expire(); // tPDDebounce: the partner is gone -> Unattached.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.starts == debounce_started + 1);
        f.timer.expire(); // the Rd slice ends: toggling
        check(f.tcpc.pull == usbc::cc_pull::rp);
        check(f.client.attached_snk == 0 && f.client.attached_src == 0);
    }

    // the sink leaves Attached.SRC: a source-preferring port goes to
    // TryWait.SNK once VBUS is down, not back to toggling
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        tc.start();
        f.timer.expire(); // Rp phase
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        f.cc_timer.expire(); // tCCDebounce -> Attached.SRC
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        f.vbus.setVoltage(5000);

        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // the sink is gone: Rd, VBUS being removed
        check(!f.tcpc.sourcing && f.vbus.discharging && f.tcpc.pull == usbc::cc_pull::rd);
        f.vbus.setVoltage(0); // vSafe0V -> TryWait.SNK
        check(!f.vbus.discharging && f.timer.armed);
        auto const debounce_started = f.timer.starts;

        // the partner turned source: Rp and VBUS attach after tCCDebounce
        f.tcpc.line_state = {usbc::cc_state::snk_power_1a5, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        check(!f.tcpc.sinking && f.timer.starts == debounce_started);
        f.timer.expire();
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.advertisement == usbc::rp_value::p_1a5);
    }

    // a flapping partner cannot extend tDRPTry: the wall is Try.SRC's
    // own timeout, armed once however the debounce bounces
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        auto const armed_once = f.timer.starts;
        auto const debounces_before = f.cc_timer.starts;

        // Rd appears and vanishes, twice: the debounce is started and
        // abandoned, the wall never moves
        for (int flap = 0; flap < 2; ++flap) {
            f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
            f.ccAlert();
            check(f.cc_timer.armed);
            f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
            f.ccAlert(); // open at once: the debounce is off
            check(!f.cc_timer.armed);
        }
        check(f.timer.starts == armed_once);
        check(f.cc_timer.starts == debounces_before + 2);

        f.timer.expire(); // tDRPTry -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && f.client.attached_src == 0);
    }

    // the wall running out mid-debounce does not abort it: Try.SRC
    // goes on, and a stable Rd still attaches
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tDRPTry: the debounce keeps running
        check(f.tcpc.pull == usbc::cc_pull::rp && f.cc_timer.armed);
        f.cc_timer.expire(); // Rd stable for tTryCCDebounce: attach
        check(f.tcpc.sourcing && f.client.attached_src == 1);
    }

    // ... while a failure after the wall, VBUS at vSafe0V, ends the
    // phase instead of resuming it
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tDRPTry is up mid-debounce
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // Rd lost: open detected at once -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && !f.cc_timer.armed); // TryWait.SNK's tCCDebounce running
        check(f.client.attached_src == 0);
    }

    // tDRPTry is up while the partner's VBUS is still above vSafe0V:
    // Rp stays presented until it is down, then TryWait.SNK
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up: Try.SRC goes on
        check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed);
        check(f.vbus.monitored == usbc::vbus_level::safe0v);
        f.vbus.setVoltage(0); // vSafe0V -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        check(f.client.attached_src == 0);
    }

    // ... or until tTryTimeout when VBUS never drops
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up
        check(f.tcpc.pull == usbc::cc_pull::rp);
        f.timer.expire(); // the rest of tTryTimeout -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        check(f.client.attached_src == 0);
    }

    // an Rd appearing after tDRPTry still attaches as source; VBUS
    // reaching vSafe0V meanwhile waits for the detection
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert(); // -> the Rd debounce
        check(f.cc_timer.armed);
        f.vbus.setVoltage(0);
        check(f.tcpc.pull == usbc::cc_pull::rp && f.cc_timer.armed);
        f.cc_timer.expire(); // Rd stable for tTryCCDebounce: attach
        check(f.tcpc.sourcing && f.client.attached_src == 1);
    }

    // ... while the debounce failing after that ends Try.SRC
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert(); // -> the Rd debounce
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // Rd lost with VBUS at vSafe0V -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && !f.cc_timer.armed);
        check(f.client.attached_src == 0);
    }

    // a debounce failing after tDRPTry with VBUS still up keeps trying
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tDRPTry is up mid-debounce, VBUS up
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // the debounce fails: Rp stays, vSafe0V watched
        check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed && !f.cc_timer.armed);
        check(f.vbus.monitored == usbc::vbus_level::safe0v);
        f.vbus.setVoltage(0); // -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_src == 0);
    }

    // tTryTimeout is up while an Rd is under debounce: its detection
    // attaches ...
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tTryTimeout mid-debounce
        check(f.tcpc.pull == usbc::cc_pull::rp && f.cc_timer.armed && !f.timer.armed);
        f.cc_timer.expire();
        check(f.tcpc.sourcing && f.client.attached_src == 1);
    }

    // ... and anything else leaves for TryWait.SNK
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySrc(f, tc);
        f.timer.expire(); // tDRPTry with VBUS up
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tTryTimeout mid-debounce
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // Rd lost -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        check(f.client.attached_src == 0);
    }

    // ErrorRecovery of a Try.SRC DRP restarts at Rp (4.5.2.2.2.2)
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        tc.start();
        check(tc.errorRecovery());
        check(f.tcpc.pull == usbc::cc_pull::open && f.timer.armed);
        f.timer.expire(); // tErrorRecovery -> Unattached.SRC
        check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed);
    }

    return failures;
}

int typeCDrpTrySnkTests()
{
    constexpr auto& timing = usbc::default_drp_timing;
    using drp = usbc::TypeCDrp<
        mock_tcpc,
        mock_vbus,
        manual_timer,
        timing,
        usbc::drp_preference::sink,
        mock_drp_client>;

    // a sink appeared during the Rp phase: where the source flow would
    // attach, a sink-preferring port enters Try.SNK
    auto const trySnk = [](fixture& f, drp& tc, usbc::cc_status sink_line) {
        tc.start();
        f.timer.expire(); // Rd phase -> Rp phase
        f.tcpc.line_state = sink_line;
        f.ccAlert();
        f.cc_timer.expire(); // tCCDebounce: would attach as source -> Try.SNK
    };
    constexpr usbc::cc_status rd_on_cc1{usbc::cc_state::src_rd, usbc::cc_state::src_open};
    constexpr usbc::cc_status rd_on_cc2{usbc::cc_state::src_open, usbc::cc_state::src_rd};

    // a sink-preferring port answers a sink's attach with Try.SNK and
    // resolves to Attached.SNK when the partner turns source
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc2);
        check(!f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_src == 0);
        check(f.timer.armed && !f.cc_timer.armed); // the tDRPTry wait, the pins not monitored

        // the partner flips to Rp and sources during the wait: nothing
        // is debounced before the wait is over
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        check(!f.cc_timer.armed);
        f.timer.expire(); // tDRPTry over: monitoring begins with the Rp present
        check(f.cc_timer.armed);
        f.cc_timer.expire(); // tTryCCDebounce
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.orientation == usbc::plug_orientation::cc2);
        check(f.client.advertisement == usbc::rp_value::p_3a0);
        check(tc.dataRole() == usbc::data_role::ufp);
    }

    // the partner's VBUS comes up only after its Rp was detected: the
    // attach follows the report
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc2);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
        f.ccAlert();
        f.timer.expire();    // tDRPTry over, the Rp is debounced
        f.cc_timer.expire(); // tTryCCDebounce without VBUS: wait for it
        check(f.client.attached_snk == 0 && !f.timer.armed && !f.cc_timer.armed);
        check(f.vbus.monitored == usbc::vbus_level::safe5v);

        f.vbus.setVoltage(5000);
        check(f.tcpc.sinking && f.client.attached_snk == 1);
    }

    // the partner insists on being a sink: no Rp for tTryCCDebounce
    // after the wait, and TryWait.SRC attaches as source
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc1);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();      // partner presents nothing while we are Rd
        f.timer.expire(); // tDRPTry -> monitoring: the open line debounces
        check(f.tcpc.pull == usbc::cc_pull::rd && f.cc_timer.armed);
        f.cc_timer.expire(); // tTryCCDebounce without an Rp -> TryWait.SRC
        check(f.tcpc.pull == usbc::cc_pull::rp);
        check(f.timer.armed && !f.cc_timer.armed); // tDRPTry, no report since the Rp flip

        f.tcpc.line_state = rd_on_cc1;
        f.ccAlert();
        f.cc_timer.expire(); // tTryCCDebounce
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        check(f.client.orientation == usbc::plug_orientation::cc1);
    }

    // an Rp appearing during the absence debounce is debounced in turn;
    // lost after its detection while VBUS is awaited, the absence
    // debounce starts over and ends the phase
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc1);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire(); // tDRPTry -> monitoring
        auto const monitoring = f.cc_timer.starts;
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert(); // -> the Rp debounce
        check(f.cc_timer.starts == monitoring + 1);
        f.cc_timer.expire(); // tTryCCDebounce, no VBUS: waiting for it
        check(!f.tcpc.sinking && !f.cc_timer.armed && f.tcpc.pull == usbc::cc_pull::rd);

        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert(); // the Rp is gone: no Rp for tTryCCDebounce?
        check(f.cc_timer.armed && f.tcpc.pull == usbc::cc_pull::rd);
        f.cc_timer.expire(); // -> TryWait.SRC
        check(f.tcpc.pull == usbc::cc_pull::rp && f.timer.armed);
        check(f.client.attached_snk == 0);
    }

    // TryWait.SRC with the Rd detected but VBUS still discharging: the
    // attach waits for vSafe0V, and losing the Rd meanwhile starts the
    // phase over with a fresh tDRPTry
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc1);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire(); // -> monitoring
        f.vbus.setVoltage(5000);
        f.cc_timer.expire(); // no Rp -> TryWait.SRC, VBUS not at vSafe0V
        auto const wall_armed = f.timer.starts;

        f.tcpc.line_state = rd_on_cc1;
        f.ccAlert();
        f.cc_timer.expire(); // Rd detected, waiting for vSafe0V
        check(!f.tcpc.sourcing && f.client.attached_src == 0);

        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // the Rd is gone: once more from the start
        check(f.timer.starts == wall_armed + 1);
        check(f.tcpc.pull == usbc::cc_pull::rp && f.client.attached_src == 0);

        f.tcpc.line_state = rd_on_cc1;
        f.ccAlert();
        f.cc_timer.expire();  // detected again
        f.vbus.setVoltage(0); // vSafe0V: attach
        check(f.tcpc.sourcing && f.client.attached_src == 1);
    }

    // nothing in TryWait.SRC: back to toggling at Rd after tDRPTry
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc1);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire();    // -> monitoring
        f.cc_timer.expire(); // -> TryWait.SRC
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert();
        f.timer.expire(); // tDRPTry -> Unattached.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_snk == 0 && f.client.attached_src == 0);
    }

    // tDRPTry expires in TryWait.SRC while an Rd is under debounce: the
    // debounce goes on, and its failure leaves for Unattached.SNK
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        trySnk(f, tc, rd_on_cc1);
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire();    // -> monitoring
        f.cc_timer.expire(); // -> TryWait.SRC

        f.tcpc.line_state = rd_on_cc1;
        f.ccAlert();      // -> the Rd debounce
        f.timer.expire(); // tDRPTry is up mid-debounce
        check(f.tcpc.pull == usbc::cc_pull::rp && f.cc_timer.armed && !f.timer.armed);
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert(); // Rd lost after the wall -> Unattached.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        check(f.client.attached_snk == 0 && f.client.attached_src == 0);
    }

    return failures;
}

int typeCDrpSwapTests()
{
    constexpr auto& timing = usbc::default_drp_timing;

    // not attached: nothing to swap; a sink attaches as UFP
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::none,
            mock_drp_client>;
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client};
        tc.start();
        check(!tc.beginSwapToSource() && !tc.beginSwapToSink());
        check(!tc.swapDataRole());
        f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire();
        check(f.tcpc.sinking && !f.tcpc.sourcing);
        check(tc.dataRole() == usbc::data_role::ufp); // a sink attaches as UFP
        check(!tc.beginSwapToSink());                 // not the departing role
    }

    // the agreed swaps, both ways (the arbitration is the PD layer's)
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::none,
            mock_drp_client,
            mock_data_role_listener>;
        fixture f;
        mock_data_role_listener listener;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client, listener};
        tc.start();

        // not attached: refused, and there is no standby to complete
        // or abort
        check(!tc.beginSwapToSource());
        check(!tc.completeSwap() && !tc.abortSwap());

        // attach as sink on CC2
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire();
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.orientation == usbc::plug_orientation::cc2);

        // data role swap: only the context's data role flips, forwarded
        // to the observers - no attach/detach seen
        check(tc.dataRole() == usbc::data_role::ufp);
        check(tc.swapDataRole());
        check(tc.dataRole() == usbc::data_role::dfp);
        check(listener.data_seen == usbc::data_role::dfp);
        check(f.tcpc.sinking && f.client.detached == 0);

        // allowed: standby with the power paths off, Rd presented, the
        // PS_RDY wait armed - and the old source dropping VBUS is NOT
        // a detach
        check(tc.beginSwapToSource());
        check(!f.tcpc.sinking && !f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && f.timer.duration == usbc::tc::drp::t_ps_source_off);
        f.vbus.setVoltage(0);              // the old source turns off
        check(f.client.detached == 1);     // the attached pair left standby once
        check(f.client.attached_snk == 1); // ... but no spurious re-attach cycle

        // PS_RDY: the pair stays attached, the orientation carries
        // over - and so does the swapped data role (a PR_Swap leaves
        // the data role alone)
        check(tc.completeSwap());
        check(f.tcpc.sourcing && !f.tcpc.sinking && f.tcpc.pull == usbc::cc_pull::rp);
        check(f.client.attached_src == 1);
        check(f.client.orientation == usbc::plug_orientation::cc2);
        check(tc.dataRole() == usbc::data_role::dfp);
        f.vbus.setVoltage(5000); // we source now

        // and back: standby waits for the new source's PS_RDY
        check(tc.beginSwapToSink());
        check(!f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && f.timer.duration == usbc::tc::drp::t_ps_source_on);
        f.vbus.setVoltage(0); // bus decays until the new source takes over
        f.vbus.setVoltage(5000);
        check(tc.completeSwap());
        check(f.tcpc.sinking && !f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_snk == 2);
        check(f.client.orientation == usbc::plug_orientation::cc2);
        check(tc.dataRole() == usbc::data_role::dfp); // survives both swaps
        // the current draw after a swap is the PD contract's business
        check(f.client.advertisement == usbc::rp_value::usb_default);

        // an aborted swap restores the departing role
        check(tc.beginSwapToSource());
        check(tc.abortSwap());
        check(f.tcpc.sinking && !f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(tc.dataRole() == usbc::data_role::dfp);

        // a standby left to its timeout falls back on its own; the
        // dead bus then detaches naturally
        check(tc.beginSwapToSource());
        f.vbus.setVoltage(0);
        f.timer.expire();       // tPSSourceOff: no PS_RDY came
        check(!f.tcpc.sinking); // back in Attached.SNK, VBUS gone: detach
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed); // toggling again
    }

    return failures;
}

int typeCDrpRoleLockTests()
{
    constexpr auto& timing = usbc::default_drp_timing;

    // locked: the toggle rests at Rd, re-asking every slice; unlocking
    // resumes the Rp slice on the next expiry, and a source appearing
    // while locked attaches as sink like any other
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::none,
            mock_drp_client,
            mock_role_lock>;
        fixture f;
        mock_role_lock lock{.sourcing = false};
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client, lock};
        tc.start();
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        auto const starts = f.timer.starts;
        f.timer.expire();
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed); // another Rd slice
        f.timer.expire();
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
        check(f.timer.starts == starts + 2);
        lock.sourcing = true;
        f.timer.expire();
        check(f.tcpc.pull == usbc::cc_pull::rp); // the Rp slice resumes
        lock.sourcing = false;
        f.timer.expire(); // the Rp slice ends as ever
        check(f.tcpc.pull == usbc::cc_pull::rd);

        f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire(); // tCCDebounce
        check(f.tcpc.sinking && f.client.attached_snk == 1);

        // the Rp vanishing from AttachWait.SNK would send the DRP on to
        // the Rp phase: locked, another Rd slice instead
        f.vbus.setVoltage(0);
        check(f.client.detached == 1 && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.cc_timer.armed); // the source is still there: debounced again
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.cc_timer.expire(); // tPDDebounce
        check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.armed);
    }

    // source preference, locked: where the port would try Rp it
    // attaches as sink - Rp never presented, the wall never armed;
    // unlocked, the same attach tries Rp
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::source,
            mock_drp_client,
            mock_role_lock>;
        fixture f;
        mock_role_lock lock{.sourcing = false};
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client, lock};
        tc.start();
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire(); // tCCDebounce: Try.SRC would follow
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(!f.timer.armed);

        f.vbus.setVoltage(0); // detach: the source still there is debounced again
        check(f.client.detached == 1 && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.cc_timer.armed);
        lock.sourcing = true;
        f.vbus.setVoltage(5000);
        f.cc_timer.expire();
        check(!f.tcpc.sinking && f.tcpc.pull == usbc::cc_pull::rp); // Try.SRC
        check(f.timer.armed && f.client.attached_snk == 1);
    }

    // ... and on the debounced path (Rp detected before VBUS arrives)
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::source,
            mock_drp_client,
            mock_role_lock>;
        fixture f;
        mock_role_lock lock{.sourcing = false};
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client, lock};
        tc.start();
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.cc_timer.expire(); // Rp detected, no VBUS yet
        check(!f.tcpc.sinking && f.tcpc.pull == usbc::cc_pull::rd);
        f.vbus.setVoltage(5000); // VBUS: Try.SRC would follow
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(!f.timer.armed);
    }

    // the lock's answer, as the PD layer reads it before requesting a
    // swap to source; an agreed swap executes regardless, and a lock
    // engaged while sourcing leaves the role standing
    {
        using drp = usbc::TypeCDrp<
            mock_tcpc,
            mock_vbus,
            manual_timer,
            timing,
            usbc::drp_preference::none,
            mock_drp_client,
            mock_role_lock>;
        fixture f;
        mock_role_lock lock;
        drp tc{f.tcpc, f.vbus, f.timer, f.cc_timer, f.client, lock};
        tc.start();
        f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.cc_timer.expire();
        check(f.tcpc.sinking);

        lock.sourcing = false;
        check(!tc.sourcingAllowed());
        lock.sourcing = true;
        check(tc.sourcingAllowed());
        check(tc.beginSwapToSource());
        check(tc.completeSwap());
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        f.vbus.setVoltage(5000);

        lock.sourcing = false; // engaged while sourcing: the role stands
        check(f.tcpc.sourcing);
        check(tc.beginSwapToSink());
    }

    return failures;
}
