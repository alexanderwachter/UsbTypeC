/*
 * USB PD sink sample: the complete port behind usbc::PdSink. The
 * facade wires the Type-C layer to the policy engine - attach,
 * detach, the PD alerts, the hard-reset window (the source's VBUS
 * cycle is not a detach), and the Error Recovery escalation. The
 * user code below provides the domain pieces only: capabilities, the
 * selection policy, and the power effects. Everything runs on the
 * stack's own work queue (CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY).
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/PdSink.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <array>

LOG_MODULE_REGISTER(pd_sink_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

// What this sink can take; also the Sink_Capabilities answer
constexpr std::array sink_capabilities{usbc::sink_capability{5000, 3000},
                                       usbc::sink_capability{9000, 3000},
                                       usbc::sink_capability{15000, 3000}};

// The power side of the engine, injected as an observer. The board has
// no real input regulator to program: log what one would do
struct Power : usbc::SinkPower<Power> {
    bool setLimit(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("load limit: %d mV, %d mA", voltage, current);
        return true;
    }
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("contract lost, back to vSafe5V"); }
};

// Extra observer in the connection machine: log the attach results
struct AttachLogger : fsm::observing<AttachLogger> {
    void notifyEntry(usbc::tc::attach_info info)
    {
        LOG_INF("attached: CC%d", info.orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
    void notifyExit(usbc::tc::attach_info) { LOG_INF("detached"); }
};

using Port = usbc::PdSink<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                          usbc::PowerPolicy, Power, AttachLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::pd_sink_timers<usbc::zephyr::Timer> timers;

usbc::PowerPolicy policy{5000, 27000}; // at least 5 W, aim for 27 W
Power power;
AttachLogger attach_logger;
Port port{tcpc, vbus, timers, sink_capabilities, policy, power, attach_logger};

} // namespace

int main()
{
    // start() is the go-live moment: the port leaves Disabled, applies
    // its terminations, and attach events flow from this line on
    port.start();

    LOG_INF("USB PD sink port running");
    return 0;
}
