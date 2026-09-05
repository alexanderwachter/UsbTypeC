/*
 * USB PD dual-role port sample: DRP toggling plus power negotiation in
 * whichever role the attach resolves to, behind usbc::PdDrp - the user
 * side is only the domain pieces: drivers, timers, capabilities,
 * policies, and the power effects. Engine routing, alert dispatch, and
 * message-header bookkeeping live in the library.
 *
 * The joystick triggers the PD swap messaging: SEL requests a
 * PR_Swap, LEFT a DR_Swap. The engines run the full exchange - the
 * request, the partner's Accept, and for the power swap the PS_RDY
 * hand-off with the termination flip in between.
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
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <array>
#include <cstdint>

LOG_MODULE_REGISTER(pd_drp_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)
#define PWRCTRL_NODE DT_ALIAS(usbc_port0_pwrctrl)

namespace {

// What this port takes as a sink, and offers as a source
constexpr std::array sink_capabilities{usbc::sink_capability{5000, 3000},
                                       usbc::sink_capability{9000, 3000}};
// The source offers come straight from the connector node's
// source-pdos: the DT PDO_FIXED words are the PD wire format that
// usbc::pdo::makeFixedSource() builds
constexpr std::array<std::uint32_t, DT_PROP_LEN(USBC_PORT0_NODE, source_pdos)> source_caps{
    DT_FOREACH_PROP_ELEM_SEP(USBC_PORT0_NODE, source_pdos, DT_PROP_BY_IDX, (,))};

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

// The eval board's source power path: a PWM duty cycle selects the
// DCDC output voltage, GPIOs gate the DCDC and the VBUS source
// switch. The settled report comes from the stack's work queue, never
// synchronously - the engine is mid-transition when setOutput() runs
// (no output-voltage feedback on this board, as in Zephyr's sample)
struct Supply {
    // duty cycles measured for the eval board's DCDC (50 us period)
    static constexpr uint32_t pulseFor(usbc::millivolt voltage)
    {
        switch (voltage) {
        case 5000:  return 21500;
        case 9000:  return 30000;
        case 15000: return 45000;
        default:    return 0;
        }
    }

    pwm_dt_spec const voltage_select = PWM_DT_SPEC_GET(PWRCTRL_NODE);
    gpio_dt_spec const source_en     = GPIO_DT_SPEC_GET(PWRCTRL_NODE, source_en_gpios);
    gpio_dt_spec const dcdc_en       = GPIO_DT_SPEC_GET(PWRCTRL_NODE, dcdc_en_gpios);
    k_work work{};
    usbc::supply_ready_callback callback = nullptr;
    void* context                        = nullptr;

    Supply()
    {
        k_work_init(&work, [](k_work* item) {
            auto* self = CONTAINER_OF(item, Supply, work);
            self->callback(self->context, true);
        });
    }

    bool init()
    {
        return gpio_pin_configure_dt(&source_en, GPIO_OUTPUT_ACTIVE) == 0 &&
               gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT_ACTIVE) == 0 &&
               pwm_set_pulse_dt(&voltage_select, pulseFor(0)) == 0;
    }

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
    {
        callback = cb;
        context  = ctx;
    }
    bool setOutput(usbc::millivolt voltage, usbc::milliamp current_limit)
    {
        LOG_INF("supply: %d mV, %d mA", voltage, current_limit);
        if (pwm_set_pulse_dt(&voltage_select, pulseFor(voltage)) != 0) {
            return false;
        }
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &work);
        return true;
    }
};
static_assert(usbc::concepts::source_supply<Supply>);

// Signals that the power supply is live at the contract's operating
// point: the engine has driven Supply::setOutput(), the output
// settled, and PS_RDY is on the wire when onContract fires.
// onContractLost reports the end (detach, Hard Reset), output back at
// vSafe5V. Report-only - the engine programs the supply itself
struct ContractMonitor : usbc::SourcePower<ContractMonitor> {
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("source contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("source contract lost, back to vSafe5V"); }
};

// The injected vconn_port: enables the feature by tag, connects the
// VCONN switch - behind the TCPC driver on this board - and allows
// taking the role. It brings the VCONN machine and the engines'
// VCONN_Swap states in; a port without an enabler compiles the
// feature out
struct VconnPolicy : fsm::observing<VconnPolicy> {
    using enables = usbc::pe::vconn_feature;

    usbc::zephyr::Tcpc* tcpc = nullptr; // wired in main()

    bool setVconn(bool on) { return tcpc->setVconn(on); }
    bool allowSwap(usbc::vconn_source_role) { return true; }
};

// Enables PR_Swap and DR_Swap by tag and arbitrates them - this
// board always agrees. Dropping this observer compiles both swap
// features out and the partner's requests are answered Not_Supported
struct SwapPolicy : fsm::observing<SwapPolicy> {
    using enables = mtl::typelist<usbc::pe::pr_swap_feature, usbc::pe::dr_swap_feature>;

    bool allowSwap(usbc::power_role) { return true; }
    bool allowSwap(usbc::data_role) { return true; }
};

// The StateLogger rides along in the connection machine (module
// usbc_fsm, debug level)
using Port = usbc::PdDrp<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                         usbc::PowerPolicy, Power, usbc::RequestPolicy, Supply, ContractMonitor,
                         usbc::default_drp_timing, usbc::drp_preference::none,
                         usbc::zephyr::StateLogger, VconnPolicy, SwapPolicy>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::pd_drp_timers<usbc::zephyr::Timer> timers;

usbc::PowerPolicy sink_policy{5000, 27000}; // at least 5 W, aim for 27 W
Power power;
usbc::RequestPolicy source_policy;
Supply supply;
ContractMonitor contract_monitor;
usbc::zephyr::StateLogger state_logger;
VconnPolicy vconn_policy;
SwapPolicy swap_policy;

// The Rp matches the 5 V capability the port advertises through PD
Port port{tcpc,        vbus,          timers, sink_capabilities, sink_policy,           power,
          source_caps, source_policy, supply, contract_monitor,  usbc::rp_value::p_1a5,
          state_logger, vconn_policy, swap_policy};

// The joystick triggers the PD swap messaging, submitted to the
// stack's queue - the serialization the swap calls require. The
// request goes on the wire; the roles change once the partner accepts
// and the PS_RDY hand-off completes
void powerSwap(k_work*)
{
    if (!port.swapPowerRole()) {
        LOG_INF("power role swap refused (no contract, busy, or vetoed)");
        return;
    }
    LOG_INF("PR_Swap sent");
}

void dataSwap(k_work*)
{
    if (!port.swapDataRole()) {
        LOG_INF("data role swap refused (no contract, busy, or vetoed)");
        return;
    }
    LOG_INF("DR_Swap sent");
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
    if (!supply.init()) {
        LOG_ERR("supply hardware init failed");
        return -1;
    }
    vconn_policy.tcpc = &tcpc; // this board's switch sits behind the TCPC
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
