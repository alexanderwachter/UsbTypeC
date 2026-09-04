/*
 * USB PD dual-role port sample: DRP toggling plus power negotiation in
 * whichever role the attach resolves to, behind usbc::PdDrp - the user
 * side is only the domain pieces: drivers, timers, capabilities,
 * policies, and the power effects. Engine routing, alert dispatch, and
 * message-header bookkeeping live in the library.
 *
 * The joystick stands in for the PD swap messaging: SEL swaps the
 * power role, LEFT the data role. Both swaps are local-only until the
 * policy engines speak PR_Swap/DR_Swap - a partner will not follow,
 * this demonstrates the port control.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/PdDrp.hpp>
#include <usbc/zephyr/StateLogger.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <array>

LOG_MODULE_REGISTER(pd_drp_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

// What this port takes as a sink, and offers as a source
constexpr std::array sink_capabilities{usbc::sink_capability{5000, 3000},
                                       usbc::sink_capability{9000, 3000}};
constexpr std::array source_caps{usbc::pdo::makeFixedSource(5000, 1500)};

// The sink engine's power side: no real input regulator, log its work
struct Power : usbc::SinkPower<Power> {
    bool setLimit(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("load limit: %d mV, %d mA", voltage, current);
        return true;
    }
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("sink contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("sink contract lost, back to vSafe5V"); }
};

// Log-only supply for the source engine: reports the target settled
// from the stack's work queue (never synchronously - the engine is
// mid-transition when setOutput() runs)
struct Supply {
    k_work work{};
    usbc::supply_callback callback = nullptr;
    void* context                  = nullptr;

    Supply()
    {
        k_work_init(&work, [](k_work* item) {
            auto* self = CONTAINER_OF(item, Supply, work);
            self->callback(self->context, true);
        });
    }

    void setCallback(usbc::supply_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt voltage, usbc::milliamp current_limit)
    {
        LOG_INF("supply: %d mV, %d mA", voltage, current_limit);
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &work);
        return true;
    }
};
static_assert(usbc::concepts::source_supply<Supply>);

struct Contract : usbc::SourcePower<Contract> {
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("source contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("source contract lost, back to vSafe5V"); }
};

// The StateLogger rides along in the connection machine (module
// usbc_fsm, debug level)
using Port = usbc::PdDrp<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                         usbc::PowerPolicy, Power, usbc::RequestPolicy, Supply, Contract,
                         usbc::default_drp_timing, usbc::drp_preference::none,
                         usbc::zephyr::StateLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::pd_drp_timers<usbc::zephyr::Timer> timers;

usbc::PowerPolicy sink_policy{5000, 27000}; // at least 5 W, aim for 27 W
Power power;
usbc::RequestPolicy source_policy;
Supply supply;
Contract contract;
usbc::zephyr::StateLogger state_logger;

// The Rp matches the 5 V capability the port advertises through PD
Port port{tcpc,   vbus,     timers,        sink_capabilities,     sink_policy,  power,
          source_caps, source_policy, supply, contract, usbc::rp_value::p_1a5, state_logger};

// The joystick stands in for the PD swap messaging, submitted to the
// stack's queue - the serialization the swap calls require. Without a
// partner PS_RDY exchange the standby is completed right away
void powerSwap(k_work*)
{
    auto const role  = port.powerRole();
    bool const begun = role == usbc::power_role::sink     ? port.beginSwapToSource()
                       : role == usbc::power_role::source ? port.beginSwapToSink()
                                                          : false;
    if (!begun) {
        LOG_INF("power role swap refused");
        return;
    }
    port.completeSwap();
    LOG_INF("power role now %s",
            port.powerRole() == usbc::power_role::source ? "source" : "sink");
}

void dataSwap(k_work*)
{
    if (!port.swapDataRole()) {
        LOG_INF("data role swap refused");
        return;
    }
    LOG_INF("data role now %s", port.dataRole() == usbc::data_role::dfp ? "DFP" : "UFP");
}

K_WORK_DEFINE(power_swap_work, powerSwap);
K_WORK_DEFINE(data_swap_work, dataSwap);

gpio_dt_spec const power_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
gpio_dt_spec const data_button  = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
gpio_callback power_button_cb;
gpio_callback data_button_cb;

void setupButton(gpio_dt_spec const& button, gpio_callback& callback,
                 gpio_callback_handler_t handler)
{
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&callback, handler, BIT(button.pin));
    gpio_add_callback(button.port, &callback);
}

} // namespace

int main()
{
    port.start(); // leave Disabled: toggle Rd/Rp, resolve with the partner

    setupButton(power_button, power_button_cb, [](const device*, gpio_callback*, uint32_t) {
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &power_swap_work);
    });
    setupButton(data_button, data_button_cb, [](const device*, gpio_callback*, uint32_t) {
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &data_swap_work);
    });

    LOG_INF("USB PD dual-role port running; SEL: power role swap, LEFT: data role swap");
    return 0;
}
