/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mocks.hpp"

#include <usbc/TypeCDrp.hpp>

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
struct mock_drp_client {
    int attached_snk                   = 0;
    int attached_src                   = 0;
    int detached                       = 0;
    usbc::plug_orientation orientation = usbc::plug_orientation::cc1;
    usbc::rp_value advertisement       = usbc::rp_value::usb_default;

    void onAttachedSnk(usbc::plug_orientation o, usbc::rp_value rp)
    {
        ++attached_snk;
        orientation   = o;
        advertisement = rp;
    }
    void onAttachedSrc(usbc::plug_orientation o)
    {
        ++attached_src;
        orientation = o;
    }
    void onDetached() { ++detached; }
};
static_assert(usbc::concepts::tc_drp_client<mock_drp_client>);

// --- compile-time checks ----------------------------------------------------
namespace compile_time {

using timing = usbc::default_drp_timing;
static_assert(usbc::concepts::drp_timing<timing>);

// the toggle slices split tDRP by dcSRC.DRP
static_assert(usbc::tc::drp::t_src_slice<timing> ==
              timing::t_drp * timing::dc_src / 100);
static_assert(usbc::tc::drp::unattached_src<timing>::timeout +
                  usbc::tc::drp::unattached_snk<timing>::timeout ==
              timing::t_drp);

// a custom timing derives from the default and overrides members
struct sink_heavy_timing : usbc::default_drp_timing {
    static constexpr auto t_drp      = std::chrono::milliseconds{100};
    static constexpr unsigned dc_src = 30;
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
    manual_timer timer;

    void ccAlert()
    {
        tcpc.alerts |= usbc::alert_status::cc_status_changed;
        tcpc.callback(tcpc.context);
    }
};

} // namespace

int typeCDrpTests()
{
    using timing = usbc::default_drp_timing;
    using drp    = usbc::TypeCDrp<mock_tcpc, mock_vbus, manual_timer, mock_drp_client>;
    constexpr auto snk_slice = timing::t_drp - usbc::tc::drp::t_src_slice<timing>;
    constexpr auto src_slice = usbc::tc::drp::t_src_slice<timing>;

    fixture f;
    drp tc{f.tcpc, f.vbus, f.timer, f.client, usbc::rp_value::p_1a5};

    // construction rests in Disabled: nothing registered, nothing driven
    check(f.tcpc.callback == nullptr && !f.vbus.monitored);

    // start() goes live toggling at Rd with the sink slice armed
    tc.start();
    check(f.tcpc.pull == usbc::cc_pull::rd && !f.tcpc.sinking && !f.tcpc.sourcing);
    check(f.vbus.monitored == usbc::vbus_level::safe5v);
    check(f.timer.armed && f.timer.duration == snk_slice);

    // the toggle alternates Rd and Rp with the configured advertisement
    f.timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rp && f.tcpc.rp == usbc::rp_value::p_1a5);
    check(f.vbus.monitored == usbc::vbus_level::safe0v);
    check(f.timer.armed && f.timer.duration == src_slice);
    f.timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rd);
    check(f.timer.armed && f.timer.duration == snk_slice);

    // a source appears during the Rd phase: sink attach flow
    f.tcpc.line_state = {usbc::cc_state::snk_power_3a0, usbc::cc_state::snk_open};
    f.ccAlert();
    check(f.timer.armed && f.timer.duration == usbc::tc::t_cc_debounce);
    f.vbus.setVoltage(5000);
    f.timer.expire();
    check(f.tcpc.sinking && !f.tcpc.sourcing);
    check(f.client.attached_snk == 1 && f.client.attached_src == 0);
    check(f.client.orientation == usbc::plug_orientation::cc1);
    check(f.client.advertisement == usbc::rp_value::p_3a0);
    check(!f.timer.armed); // attached: no toggle running

    // detach resumes toggling at Rd
    f.vbus.setVoltage(0);
    check(!f.tcpc.sinking && f.client.detached == 1);
    check(f.tcpc.pull == usbc::cc_pull::rd);
    check(f.timer.armed && f.timer.duration == snk_slice);

    // a sink appears during the Rp phase: source attach flow
    f.timer.expire(); // now presenting Rp
    check(f.tcpc.pull == usbc::cc_pull::rp);
    f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
    f.ccAlert();
    check(f.timer.duration == usbc::tc::t_cc_debounce);
    f.timer.expire();
    check(f.tcpc.sourcing && !f.tcpc.sinking);
    check(f.client.attached_src == 1);
    check(f.client.orientation == usbc::plug_orientation::cc2);
    f.vbus.setVoltage(5000); // the supply raises VBUS

    // detach discharges before toggling resumes at Rp
    f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
    f.ccAlert();
    check(!f.tcpc.sourcing && f.vbus.discharging);
    check(f.client.detached == 2);
    f.vbus.setVoltage(0);
    check(!f.vbus.discharging);
    check(f.tcpc.pull == usbc::cc_pull::rp);
    check(f.timer.armed && f.timer.duration == src_slice);

    // noise during a toggle phase debounces back to toggling
    f.timer.expire(); // Rd phase
    f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
    f.ccAlert();
    check(f.timer.duration == usbc::tc::t_cc_debounce);
    f.timer.expire();
    check(f.tcpc.pull == usbc::cc_pull::rd && f.timer.duration == snk_slice);
    check(f.client.attached_snk == 1 && f.client.attached_src == 1);

    return failures;
}

int typeCDrpTrySrcTests()
{
    using timing = usbc::default_drp_timing;
    using drp    = usbc::TypeCDrp<mock_tcpc, mock_vbus, manual_timer, mock_drp_client, timing,
                                  usbc::drp_preference::source>;

    // a source-preferring port answers a sink attach with Try.SRC and
    // resolves to Attached.SRC when the partner presents Rd
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();

        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.timer.expire(); // tCCDebounce: would attach as sink -> Try.SRC
        check(!f.tcpc.sinking && !f.tcpc.sourcing);
        check(f.tcpc.pull == usbc::cc_pull::rp);
        check(f.client.attached_snk == 0);
        check(f.timer.armed && f.timer.duration == timing::t_drp_try);

        // the partner flips to Rd (and stops sourcing VBUS)
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        check(f.timer.duration == timing::t_try_cc_debounce);
        f.timer.expire();
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        check(f.client.orientation == usbc::plug_orientation::cc1);
    }

    // no Rd within tDRPTry: TryWait.SNK attaches as sink on VBUS
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();

        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.timer.expire(); // -> Try.SRC
        f.vbus.setVoltage(0);
        f.timer.expire(); // tDRPTry: no Rd -> TryWait.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.armed && f.timer.duration == timing::t_drp_try_wait);

        // the partner keeps its Rp and sources again: Attached.SNK
        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.advertisement == usbc::rp_value::usb_default);
    }

    // nothing at all in TryWait.SNK: back to toggling
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();

        f.tcpc.line_state = {usbc::cc_state::snk_default, usbc::cc_state::snk_open};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.timer.expire(); // -> Try.SRC
        f.vbus.setVoltage(0);
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.timer.expire(); // -> TryWait.SNK
        f.timer.expire(); // tDRPTryWait: nothing -> Unattached.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.timer.duration == timing::t_drp - usbc::tc::drp::t_src_slice<timing>);
        check(f.client.attached_snk == 0 && f.client.attached_src == 0);
    }

    return failures;
}

int typeCDrpTrySnkTests()
{
    using timing = usbc::default_drp_timing;
    using drp    = usbc::TypeCDrp<mock_tcpc, mock_vbus, manual_timer, mock_drp_client, timing,
                                  usbc::drp_preference::sink>;

    // a sink-preferring port answers a sink's attach with Try.SNK and
    // resolves to Attached.SNK when the partner turns source
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();
        f.timer.expire(); // Rd phase -> Rp phase

        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_rd};
        f.ccAlert();
        f.timer.expire(); // tCCDebounce: would attach as source -> Try.SNK
        check(!f.tcpc.sourcing && f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_src == 0);
        check(f.timer.armed && f.timer.duration == timing::t_drp_try);

        // the partner flips to Rp and sources during the wait
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_power_3a0};
        f.ccAlert();
        f.vbus.setVoltage(5000);
        f.timer.expire(); // tDRPTry over, Rp already in the context
        check(f.timer.duration == timing::t_pd_debounce);
        f.timer.expire();
        check(f.tcpc.sinking && f.client.attached_snk == 1);
        check(f.client.orientation == usbc::plug_orientation::cc2);
        check(f.client.advertisement == usbc::rp_value::p_3a0);
    }

    // the partner insists on being a sink: TryWait.SRC attaches as source
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();
        f.timer.expire(); // Rp phase

        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        f.timer.expire(); // -> Try.SNK
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();      // partner presents nothing while we are Rd
        f.timer.expire(); // tDRPTry -> monitoring
        check(f.timer.duration == timing::t_try_timeout - timing::t_drp_try);
        f.timer.expire(); // tTryTimeout -> TryWait.SRC
        check(f.tcpc.pull == usbc::cc_pull::rp);
        check(f.timer.duration == timing::t_drp_try_wait);

        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        check(f.timer.duration == timing::t_try_cc_debounce);
        f.timer.expire();
        check(f.tcpc.sourcing && f.client.attached_src == 1);
        check(f.client.orientation == usbc::plug_orientation::cc1);
    }

    // nothing in TryWait.SRC: back to toggling at Rd
    {
        fixture f;
        drp tc{f.tcpc, f.vbus, f.timer, f.client};
        tc.start();
        f.timer.expire(); // Rp phase

        f.tcpc.line_state = {usbc::cc_state::src_rd, usbc::cc_state::src_open};
        f.ccAlert();
        f.timer.expire(); // -> Try.SNK
        f.tcpc.line_state = {usbc::cc_state::snk_open, usbc::cc_state::snk_open};
        f.ccAlert();
        f.timer.expire(); // -> monitoring
        f.timer.expire(); // -> TryWait.SRC
        f.tcpc.line_state = {usbc::cc_state::src_open, usbc::cc_state::src_open};
        f.ccAlert();
        f.timer.expire(); // tDRPTryWait -> Unattached.SNK
        check(f.tcpc.pull == usbc::cc_pull::rd);
        check(f.client.attached_snk == 0 && f.client.attached_src == 0);
    }

    return failures;
}
