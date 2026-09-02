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

#include <usbc/TypeCSink.hpp>
#include <usbc/zephyr/StateLogger.hpp>
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

// Observer injected into the sink's machine, watching the attached
// state's attachedInfo()
struct AttachLogger : fsm::observing<AttachLogger> {
    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::tc::attach_info info)
    {
        LOG_INF("attached: CC%d, source advertises %u mA",
                info.orientation == usbc::plug_orientation::cc1 ? 1 : 2,
                advertisedMilliamps(info.advertisement));
    }
    void notifyExit(usbc::tc::attach_info) { LOG_INF("detached"); }
};

// The StateLogger traces every transition (module usbc_fsm, debug level)
using Sink = usbc::TypeCSink<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                             AttachLogger, usbc::zephyr::StateLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer timer;
AttachLogger logger;
usbc::zephyr::StateLogger state_logger;
Sink sink{tcpc, vbus, timer, logger, state_logger};

} // namespace

int main()
{
    sink.start(); // leave Disabled: present Rd, react to sources

    LOG_INF("USB-C sink port running");
    return 0;
}
