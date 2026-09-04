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
 * resolve attaches through Try.SRC or Try.SNK. Role swaps belong to
 * USB PD - see the pd_drp sample.
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

// The StateLogger traces every transition (module usbc_fsm, debug
// level) - a toggling DRP logs several per tDRP
using Drp = usbc::TypeCDrp<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                           usbc::default_drp_timing, usbc::drp_preference::none, AttachLogger,
                           usbc::zephyr::StateLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
AttachLogger logger;
usbc::zephyr::StateLogger state_logger;
// Default Rp advertisement while presenting the source role
Drp drp{tcpc, vbus, timer, usbc::rp_value::usb_default, logger, state_logger};

} // namespace

int main()
{
    drp.start(); // leave Disabled: toggle Rd/Rp, resolve with the partner

    LOG_INF("USB-C dual-role port running");
    return 0;
}
