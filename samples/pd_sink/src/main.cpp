/*
 * USB PD sink sample: attach detection plus power negotiation. The
 * Type-C layer handles attach/detach and forwards the PD alert bits to
 * the policy engine, which negotiates a contract selected by
 * PowerPolicy from the sink capabilities below. Everything runs on the
 * stack's own work queue (CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY).
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/PolicyEngine.hpp>
#include <usbc/TypeC.hpp>
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

using Engine = usbc::SinkPolicyEngine<usbc::zephyr::Tcpc, usbc::zephyr::Timer, usbc::PowerPolicy,
                                      Power>;

// Reacts to the connection layer and feeds the engine
struct PortClient {
    Engine& engine;

    void onAttached(usbc::plug_orientation orientation, usbc::rp_value advertisement)
    {
        LOG_INF("attached: CC%d", orientation == usbc::plug_orientation::cc1 ? 1 : 2);
        static_cast<void>(advertisement);
        engine.vbusPresent(); // a sink's attach implies VBUS
    }
    void onDetached()
    {
        engine.vbusRemoved();
        LOG_INF("detached");
    }
    void onPdAlert(usbc::alert_status alerts) { engine.onAlert(alerts); }
};

using Sink = usbc::TypeCSink<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                             PortClient>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer tc_timer;
usbc::zephyr::Timer prl_timer;
usbc::zephyr::Timer pe_timer;

usbc::PowerPolicy policy{5000, 27000}; // at least 5 W, aim for 27 W
Power power;
Engine engine{tcpc, prl_timer, pe_timer, sink_capabilities, policy, power};
PortClient port_client{engine};

} // namespace

int main()
{
    static Sink sink{tcpc, vbus, tc_timer, port_client};
    static_cast<void>(sink);

    LOG_INF("USB PD sink port running");
    return 0;
}
