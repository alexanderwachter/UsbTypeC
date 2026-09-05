/*
 * USB PD source sample: the complete port behind usbc::PdSource. The
 * facade wires the Type-C layer to the policy engine - attach,
 * detach, the PD alerts, and the Error Recovery escalation; a hard
 * reset's VBUS cycle is the engine's own supply choreography. The
 * user code below provides the domain pieces only: capabilities, the
 * request policy, the supply, and the contract monitor. Everything
 * runs on the stack's own work queue
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

#include <usbc/PdSource.hpp>
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
    usbc::supply_ready_callback callback = nullptr;
    void* context                        = nullptr;

    Supply()
    {
        k_work_init(&work, [](k_work* item) {
            auto* self = CONTAINER_OF(item, Supply, work);
            self->callback(self->context, true);
        });
    }

    void setReadyCallback(usbc::supply_ready_callback cb, void* ctx)
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

// Signals that the power supply is live at the contract's operating
// point: the engine has driven Supply::setOutput(), the output
// settled, and PS_RDY is on the wire when onContract fires.
// onContractLost reports the end (detach, Hard Reset), output back at
// vSafe5V. Report-only - the engine programs the supply itself
struct ContractMonitor : usbc::SourcePower<ContractMonitor> {
    void onContract(usbc::millivolt voltage, usbc::milliamp current)
    {
        LOG_INF("contract: %d mV at %d mA", voltage, current);
    }
    void onContractLost() { LOG_WRN("contract lost, back to vSafe5V"); }
};

// Extra observer in the connection machine: log the attach results
struct AttachLogger : fsm::observing<AttachLogger> {
    static constexpr auto observe_nonstatic(auto const& state)
        -> decltype((state.attachedInfo()))
    {
        return state.attachedInfo();
    }
    void notifyEntry(usbc::plug_orientation orientation)
    {
        LOG_INF("sink attached: CC%d", orientation == usbc::plug_orientation::cc1 ? 1 : 2);
    }
    void notifyExit(usbc::plug_orientation) { LOG_INF("sink detached"); }
};

using Port = usbc::PdSource<usbc::zephyr::Tcpc, usbc::zephyr::Vbus, usbc::zephyr::Timer,
                            usbc::RequestPolicy, Supply, ContractMonitor, AttachLogger>;

usbc::zephyr::Tcpc tcpc{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, tcpc))};
usbc::zephyr::Vbus vbus{DEVICE_DT_GET(DT_PROP(USBC_PORT0_NODE, vbus))};
usbc::pd_source_timers<usbc::zephyr::Timer> timers;

usbc::RequestPolicy policy;
Supply supply;
ContractMonitor contract_monitor;
AttachLogger attach_logger;
// The Rp matches the 5 V capability the port advertises through PD
Port port{tcpc, vbus,   timers,           source_caps,  policy,
          supply, contract_monitor, usbc::rp_value::p_1a5, attach_logger};

} // namespace

int main()
{
    // start() is the go-live moment: the port leaves Disabled,
    // presents Rp, and reacts to sinks from this line on
    port.start();

    LOG_INF("USB PD source port running");
    return 0;
}
