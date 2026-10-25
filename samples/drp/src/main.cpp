/*
 * USB Type-C dual-role port (DRP) sample: the port toggles between
 * presenting Rd and Rp per tDRP/dcSRC.DRP and resolves to whichever
 * role the partner complements - it charges from a charger and powers
 * a sink. The attach observer learns the resolved role from the
 * attached states' attached_role annotation and the plug orientation
 * from their instance value. Everything runs on the stack's own work queue
 * (priority set by CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY), which is
 * the serialization the stack requires.
 *
 * Toggle timing and role preference are compile-time configuration:
 * pass a constexpr usbc::drp_timing instance (designated initializers
 * over the defaults, spec ranges enforced) to change tDRP/dcSRC.DRP,
 * and pick usbc::drp_preference::source or ::sink to resolve attaches
 * through Try.SRC or Try.SNK. Role swaps belong to USB PD - see the
 * pd_drp sample.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/zephyr/TraceLogger.hpp>
#include <usbc/TypeCDrp.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drp_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

// Observer injected into the DRP's machine: the attached states'
// attached_role annotation says which role attached and its exit is
// the detach; both roles deliver the plug orientation
struct AttachLogger : fsm::observing<AttachLogger> {
    void notifyEntry(usbc::tc::attached_role attached)
    {
        LOG_INF("attached as %s", attached.role == usbc::power_role::sink ? "sink" : "source");
    }
    void notifyExit(usbc::tc::attached_role) { LOG_INF("detached"); }
    void notifyEntry(usbc::plug_orientation orientation)
    {
        LOG_INF("plug orientation: CC%d", orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
};

// The TraceLogger traces every transition (module mtl_fsm, info
// level, behind CONFIG_MTL_FSM_TRACE) - a toggling DRP logs several
// per tDRP
using Drp = usbc::TypeCDrp<
    usbc::zephyr::Tcpc,
    usbc::zephyr::Vbus,
    usbc::zephyr::Timer,
    usbc::default_drp_timing,
    usbc::drp_preference::none,
    AttachLogger,
    mtl::zephyr::TraceLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
usbc::zephyr::Timer cc_timer; // the CC debounce
AttachLogger logger;
mtl::zephyr::TraceLogger state_logger;
// Default Rp advertisement while presenting the source role
Drp drp{tcpc, vbus, timer, cc_timer, usbc::rp_value::usb_default, logger, state_logger};

} // namespace

int main()
{
    drp.start(); // leave Disabled: toggle Rd/Rp, resolve with the partner

    LOG_INF("USB-C dual-role port running");
    return 0;
}
