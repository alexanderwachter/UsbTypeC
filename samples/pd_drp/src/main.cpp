/*
 * USB PD dual-role port sample: DRP toggling plus power negotiation in
 * whichever role the attach resolves to. One policy engine per role is
 * instantiated up front; the PortRouter observer activates the one the
 * resolved role needs, routes the PD alert bits to it, arbitrates role
 * swaps, and keeps the TCPC's message header aligned with the current
 * roles. A role swap tears the old role's engine down and brings the
 * other one up through the same attach observation as a plug-in - the
 * swap needs no wiring of its own.
 *
 * The joystick stands in for the PD swap messaging: SEL swaps the
 * power role, LEFT the data role. Both swaps are local-only until the
 * policy engines speak PR_Swap/DR_Swap - a partner will not follow,
 * this demonstrates the port plumbing.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/SinkPolicyEngine.hpp>
#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCDrp.hpp>
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

using SinkEngine = usbc::SinkPolicyEngine<usbc::zephyr::Tcpc, usbc::zephyr::Timer,
                                          usbc::PowerPolicy, Power>;
using SourceEngine = usbc::SourcePolicyEngine<usbc::zephyr::Tcpc, usbc::zephyr::Timer,
                                              usbc::RequestPolicy, Supply, Contract>;

// Observer injected into the DRP's machine: activates the engine the
// resolved role needs, routes the PD alerts to it, arbitrates swaps,
// and keeps the message header's roles current
struct PortRouter : fsm::observing<PortRouter> {
    usbc::zephyr::Tcpc& tcpc;
    SinkEngine& snk;
    SourceEngine& src;

    enum class active_role { none, sink, source };
    active_role active   = active_role::none;
    usbc::data_role data = usbc::data_role::ufp;
    bool swapping        = false; // a power swap preserves the data role

    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::tc::attach_info info)
    {
        LOG_INF("attached as sink: CC%d",
                info.orientation == usbc::plug_orientation::cc1 ? 1 : 2);
        if (!swapping) {
            data = usbc::data_role::ufp; // a fresh sink attach is UFP
        }
        swapping = false;
        header(usbc::power_role::sink);
        active = active_role::sink;
        snk.vbusPresent();
    }
    void notifyExit(usbc::tc::attach_info)
    {
        snk.vbusRemoved();
        active = active_role::none;
        LOG_INF("sink role ended");
    }
    void notifyEntry(usbc::plug_orientation orientation)
    {
        LOG_INF("attached as source: CC%d",
                orientation == usbc::plug_orientation::cc1 ? 1 : 2);
        if (!swapping) {
            data = usbc::data_role::dfp; // a fresh source attach is DFP
        }
        swapping = false;
        header(usbc::power_role::source);
        active = active_role::source;
        src.attached();
    }
    void notifyExit(usbc::plug_orientation)
    {
        src.detached();
        active = active_role::none;
        LOG_INF("source role ended");
    }

    void onPdAlert(usbc::alert_status alerts)
    {
        switch (active) {
        case active_role::sink: snk.onAlert(alerts); break;
        case active_role::source: src.onAlert(alerts); break;
        case active_role::none: break; // nobody negotiating
        }
    }

    bool allowSwap(usbc::power_role role)
    {
        LOG_INF("power role swap to %s allowed",
                role == usbc::power_role::source ? "source" : "sink");
        swapping = true;
        return true;
    }
    bool allowSwap(usbc::data_role role)
    {
        LOG_INF("data role swap to %s allowed", role == usbc::data_role::dfp ? "DFP" : "UFP");
        return true;
    }
    void onDataRole(usbc::data_role role)
    {
        data = role;
        header(active == active_role::source ? usbc::power_role::source
                                             : usbc::power_role::sink);
        LOG_INF("data role now %s", role == usbc::data_role::dfp ? "DFP" : "UFP");
    }

    void header(usbc::power_role power)
    {
        tcpc.setMessageHeaderInfo({power, data, usbc::pd_revision::rev_3_x});
    }
};

using Drp = usbc::TypeCDrp<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                           usbc::default_drp_timing, usbc::drp_preference::none, PortRouter>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer tc_timer;
usbc::zephyr::Timer snk_prl_timer;
usbc::zephyr::Timer snk_pe_timer;
usbc::zephyr::Timer src_prl_timer;
usbc::zephyr::Timer src_pe_timer;

usbc::PowerPolicy sink_policy{5000, 27000}; // at least 5 W, aim for 27 W
Power power;
SinkEngine sink_engine{tcpc, snk_prl_timer, snk_pe_timer, sink_capabilities, sink_policy, power};

usbc::RequestPolicy source_policy;
Supply supply;
Contract contract;
SourceEngine source_engine{tcpc, src_prl_timer, src_pe_timer, source_caps, source_policy,
                           supply, contract};

PortRouter router{.tcpc = tcpc, .snk = sink_engine, .src = source_engine};
// The Rp matches the 5 V capability the port advertises through PD
Drp drp{tcpc, vbus, tc_timer, usbc::rp_value::p_1a5, router};

// The joystick stands in for the PD swap messaging, submitted to the
// stack's queue - the serialization the swap calls require. Without a
// partner PS_RDY exchange the standby is completed right away
void powerSwap(k_work*)
{
    auto const role  = drp.powerRole();
    bool const begun = role == usbc::power_role::sink     ? drp.beginSwapToSource()
                       : role == usbc::power_role::source ? drp.beginSwapToSink()
                                                          : false;
    if (!begun) {
        LOG_INF("power role swap refused");
        return;
    }
    drp.completeSwap();
}

void dataSwap(k_work*)
{
    if (!drp.swapDataRole()) {
        LOG_INF("data role swap refused");
    }
}

K_WORK_DEFINE(power_swap_work, powerSwap);
K_WORK_DEFINE(data_swap_work, dataSwap);

gpio_dt_spec const power_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
gpio_dt_spec const data_button  = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
gpio_callback power_button_cb;
gpio_callback data_button_cb;

void setupButton(gpio_dt_spec const& button, gpio_callback& callback, gpio_callback_handler_t handler)
{
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&callback, handler, BIT(button.pin));
    gpio_add_callback(button.port, &callback);
}

} // namespace

int main()
{
    drp.start(); // leave Disabled: toggle Rd/Rp, resolve with the partner

    setupButton(power_button, power_button_cb, [](const device*, gpio_callback*, uint32_t) {
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &power_swap_work);
    });
    setupButton(data_button, data_button_cb, [](const device*, gpio_callback*, uint32_t) {
        k_work_submit_to_queue(&usbc::zephyr::workQueue(), &data_swap_work);
    });

    LOG_INF("USB PD dual-role port running; SEL: power role swap, LEFT: data role swap");
    return 0;
}
