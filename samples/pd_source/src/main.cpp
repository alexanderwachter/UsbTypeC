/*
 * USB PD source sample: attach detection plus power advertisement and
 * negotiation. The Type-C source layer applies VBUS and forwards the
 * PD alert bits to the policy engine, which advertises the fixed
 * source PDOs below and evaluates Requests through RequestPolicy.
 * Everything runs on the stack's own work queue
 * (CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY).
 *
 * The board has no programmable supply: the Supply below logs the
 * operating point and reports it settled from the stack's work queue -
 * a real implementation programs the regulator and reports through the
 * callback when the output is in tolerance.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/SourcePolicyEngine.hpp>
#include <usbc/TypeCSource.hpp>
#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <array>

LOG_MODULE_REGISTER(pd_source_sample, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

namespace {

// What this source offers; also the Source_Capabilities content.
// Matches the source-pdos of the board overlay
constexpr std::array source_caps{usbc::pdo::makeFixedSource(5000, 1500),
                                 usbc::pdo::makeFixedSource(9000, 1000)};

// Log-only supply: reports the target settled from the stack's work
// queue (never synchronously - the engine is mid-transition when
// setOutput() runs)
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

// The contract-notification side of the engine, injected as an observer
struct Contract : usbc::SourcePower<Contract> {
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("contract lost, back to vSafe5V"); }
};

using Engine = usbc::SourcePolicyEngine<usbc::zephyr::Tcpc, usbc::zephyr::Timer,
                                        usbc::RequestPolicy, Supply, Contract>;

// Reacts to the connection layer and feeds the engine
struct PortClient {
    Engine& engine;

    void onAttached(usbc::plug_orientation orientation)
    {
        LOG_INF("sink attached: CC%d", orientation == usbc::plug_orientation::cc1 ? 1 : 2);
        engine.attached();
    }
    void onDetached()
    {
        engine.detached();
        LOG_INF("sink detached");
    }
    void onPdAlert(usbc::alert_status alerts) { engine.onAlert(alerts); }
};

using Source = usbc::TypeCSource<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                                 PortClient>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::zephyr::Timer tc_timer;
usbc::zephyr::Timer prl_timer;
usbc::zephyr::Timer pe_timer;

usbc::RequestPolicy policy;
Supply supply;
Contract contract;
Engine engine{tcpc, prl_timer, pe_timer, source_caps, policy, supply, contract};
PortClient port_client{engine};
// The Rp matches the 5 V capability the port advertises through PD
Source source{tcpc, vbus, tc_timer, port_client, usbc::rp_value::p_1a5};

} // namespace

int main()
{

    // start() is the go-live moment: the port leaves Disabled,
    // presents Rp, and reacts to sinks from this line on
    source.start();

    LOG_INF("USB PD source port running");
    return 0;
}
