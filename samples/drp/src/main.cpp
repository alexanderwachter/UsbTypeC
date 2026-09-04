/*
 * USB Type-C dual-role port (DRP) sample: the port toggles between
 * presenting Rd and Rp per tDRP/dcSRC.DRP and resolves to whichever
 * role the partner complements - it charges from a charger and powers
 * a sink. The attach observer learns the resolved role from the info
 * type: tc::attach_info means attached as sink, plug_orientation
 * attached as source. Everything runs on the stack's own work queue
 * (priority set by CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY), which is
 * the serialization the stack requires.
 *
 * Toggle timing and role preference are compile-time configuration:
 * derive from usbc::default_drp_timing to change tDRP/dcSRC.DRP (spec
 * ranges enforced), and pick usbc::drp_preference::source or ::sink to
 * resolve attaches through Try.SRC or Try.SNK.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/TypeCDrp.hpp>
#include <usbc/zephyr/StateLogger.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drp_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

// Observer injected into the DRP's machine, watching both attached
// states' attachedInfo(); the info type encodes the resolved role
struct AttachLogger : fsm::observing<AttachLogger> {
    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::tc::attach_info info)
    {
        LOG_INF("attached as sink: CC%d",
                info.orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
    void notifyExit(usbc::tc::attach_info) { LOG_INF("source detached"); }
    void notifyEntry(usbc::plug_orientation orientation)
    {
        LOG_INF("attached as source: CC%d, VBUS on",
                orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
    void notifyExit(usbc::plug_orientation) { LOG_INF("sink detached, VBUS off"); }
};

// Swap-policy observer: the runtime say over PD-directed role swaps,
// consulted with the role the port would take. Without an observer
// providing allowSwap, swaps of that kind are refused
struct SwapArbiter {
    bool allowSwap(usbc::power_role role)
    {
        LOG_INF("power role swap to %s allowed",
                role == usbc::power_role::source ? "source" : "sink");
        return true;
    }
    bool allowSwap(usbc::data_role role)
    {
        LOG_INF("data role swap to %s allowed", role == usbc::data_role::dfp ? "DFP" : "UFP");
        return true;
    }
};

// Hears every data role change (a DR_Swap flips no terminations, so
// the port forwards the new role to observers providing onDataRole) -
// the place to retarget the USB stack between host and device
struct DataRoleLogger {
    void onDataRole(usbc::data_role role)
    {
        LOG_INF("data role now %s", role == usbc::data_role::dfp ? "DFP" : "UFP");
    }
};

// The StateLogger traces every transition (module usbc_fsm, debug
// level) - a toggling DRP logs several per tDRP
using Drp = usbc::TypeCDrp<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                           usbc::default_drp_timing, usbc::drp_preference::none, AttachLogger,
                           SwapArbiter, DataRoleLogger, usbc::zephyr::StateLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
AttachLogger logger;
SwapArbiter arbiter;
DataRoleLogger data_role_logger;
usbc::zephyr::StateLogger state_logger;
// Default Rp advertisement while presenting the source role
Drp drp{tcpc, vbus, timer, usbc::rp_value::usb_default, logger, arbiter, data_role_logger,
        state_logger};

// Demo stand-in for the PD layer: attempt a data role swap every few
// seconds, on the stack's work queue - the serialization swapDataRole()
// requires. Refused while not attached
void swapDemo(k_work* work)
{
    if (!drp.swapDataRole()) {
        LOG_INF("data role swap refused");
    }
    k_work_schedule_for_queue(&usbc::zephyr::workQueue(), k_work_delayable_from_work(work),
                              K_SECONDS(5));
}

K_WORK_DELAYABLE_DEFINE(swap_demo_work, swapDemo);

} // namespace

int main()
{
    drp.start(); // leave Disabled: toggle Rd/Rp, resolve with the partner
    k_work_schedule_for_queue(&usbc::zephyr::workQueue(), &swap_demo_work, K_SECONDS(5));

    LOG_INF("USB-C dual-role port running");
    return 0;
}
