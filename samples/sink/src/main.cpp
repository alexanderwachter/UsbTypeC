/*
 * USB Type-C sink sample: attach/detach detection with the UsbTypeC
 * stack on Zephyr driver adapters. Everything runs on the stack's own
 * work queue (priority set by CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY)
 * - the VBUS poller, the deferred TCPC alerts, and the debounce timer
 * - which is the serialization the stack requires. The sink registers
 * itself with the drivers on construction; there is no glue to write.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/TypeC.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sink_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

unsigned advertisedMilliamps(usbc::rp_value advertisement)
{
    switch (advertisement) {
    case usbc::rp_value::p_1a5: return 1500;
    case usbc::rp_value::p_3a0: return 3000;
    default: return 500;
    }
}

struct AttachLogger {
    void onAttached(usbc::plug_orientation orientation, usbc::rp_value advertisement)
    {
        LOG_INF("attached: CC%d, source advertises %u mA",
                orientation == usbc::plug_orientation::cc1 ? 1 : 2,
                advertisedMilliamps(advertisement));
    }
    void onDetached() { LOG_INF("detached"); }
};

using Sink =
    usbc::TypeCSink<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer, AttachLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
AttachLogger logger;

} // namespace

int main()
{
    static Sink sink{tcpc, vbus, timer, logger};
    static_cast<void>(sink);

    LOG_INF("USB-C sink port running");
    return 0;
}
