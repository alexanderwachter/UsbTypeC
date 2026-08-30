/*
 * Adapter from a Zephyr TCPC device driver
 * (zephyr/drivers/usb_c/usbc_tcpc.h) to the stack's tcpc,
 * vconn_switch, and pd_transport concepts.
 *
 * Alerts arrive per-event from the driver's callback context, are
 * accumulated atomically, and the registered alert callback is
 * delivered from a workqueue (the system workqueue by default) - the
 * stack's serialized context. readAlert() drains the accumulated
 * flags.
 * The PD revision from setMessageHeaderInfo() cannot be forwarded -
 * Zephyr's set_roles carries no revision. Ports whose VBUS switches
 * live outside the TCPC (dedicated PPC, GPIO) wrap or replace
 * sourceVbus()/sinkVbus(). The stack expects single transmission
 * attempts: configure the driver without hardware retries.
 *
 * The instance is pinned: the driver holds its address for the alert
 * callback.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Tcpc.hpp>

#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <optional>

namespace usbc::zephyr {

class Tcpc {
public:
    // nullptr delivers the alert callback on the system workqueue
    explicit Tcpc(device const* dev, k_work_q* queue = nullptr);
    Tcpc(Tcpc const&)            = delete;
    Tcpc& operator=(Tcpc const&) = delete;

    void setAlertHandler(alert_callback callback, void* context);
    std::optional<alert_status> readAlert();

    bool setCc(cc_pull pull, rp_value rp);
    std::optional<cc_status> readCcStatus();
    bool setPlugOrientation(plug_orientation orientation);

    bool sourceVbus(bool enable);
    bool sinkVbus(bool enable);
    bool setVconn(bool enable);

    bool setMessageHeaderInfo(message_header_info info);
    bool setReceiveDetect(receive_detect detect);
    bool transmit(pd_message const& message);
    bool transmit(transmit_signal signal);
    bool receive(pd_message& out);

private:
    static void alert(device const* dev, void* data, tcpc_alert alert);
    static void notifyWork(k_work* work);

    device const* dev_;
    k_work_q* queue_;
    k_work alert_work_{};
    alert_callback callback_ = nullptr;
    void* context_           = nullptr;
    atomic_t pending_        = ATOMIC_INIT(0);
    cc_pull pull_            = cc_pull::open;
};

static_assert(concepts::tcpc<Tcpc>);
static_assert(concepts::vconn_switch<Tcpc>);
static_assert(concepts::pd_transport<Tcpc>);

} // namespace usbc::zephyr
