/*
 * USB Type-C source sample: attach/detach detection with the UsbTypeC
 * stack on Zephyr driver adapters. The port presents Rp, applies VBUS
 * when a sink's Rd has debounced and VBUS is at vSafe0V, and
 * discharges after the detach. Everything runs on the stack's own work
 * queue (priority set by CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY) -
 * the VBUS poller, the deferred TCPC alerts, and the debounce timer -
 * which is the serialization the stack requires.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/TypeCSource.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(source_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

struct AttachLogger {
    void onAttached(usbc::plug_orientation orientation)
    {
        LOG_INF("sink attached: CC%d, VBUS on",
                orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
    void onDetached() { LOG_INF("sink detached, VBUS off"); }
};

using Source =
    usbc::TypeCSource<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer, AttachLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
AttachLogger logger;

} // namespace

int main()
{
    // Construction is the go-live moment: the port presents Rp and
    // reacts to sinks from here on. Default Rp advertisement: the
    // board sources default USB current only
    static Source source{tcpc, vbus, timer, logger, usbc::rp_value::usb_default};
    static_cast<void>(source);

    LOG_INF("USB-C source port running");
    return 0;
}
