/*
 * Adapter from a Zephyr TCPC device driver
 * (zephyr/drivers/usb_c/usbc_tcpc.h) to the stack's cc and pd_phy
 * concepts: one device with one alert handler, so one object serves as
 * both blocks and a port passes it twice. The VBUS paths are the vbus
 * block's (Vbus.hpp), not this adapter's.
 *
 * Alerts arrive per-event from the driver's callback context, are
 * accumulated atomically, and leave as the blocks' typed callbacks from
 * the stack's work queue (WorkQueue.hpp) - the stack's serialized
 * context: a CC change as the lines' reading, a hard reset, a transmit
 * outcome, every received message fetched from the driver. The PD
 * revision from setMessageHeaderInfo() cannot be forwarded - Zephyr's
 * set_roles carries no revision. Zephyr's TCPC API is the TCPCI
 * contract: the driver runs CRCReceiveTimer and the nRetryCount
 * retransmissions and reports one outcome per message (success on
 * GoodCRC, failed once its retries are spent), so this adapter declares
 * `retransmits` and the protocol layer leaves the retransmission to it.
 *
 * The instance is pinned: the driver holds its address for the alert
 * callback and the machines hold it as an observer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Cc.hpp>
#include <usbc/PdPhy.hpp>

#include <mtl/StateMachine.hpp>

#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <optional>

namespace usbc::zephyr {

class Tcpc : public fsm::observing<Tcpc> {
public:
    explicit Tcpc(device const* dev);
    Tcpc(Tcpc const&) = delete;
    Tcpc& operator=(Tcpc const&) = delete;

    // the driver runs CRCReceiveTimer and the retransmissions
    static constexpr bool retransmits = true;

    // --- the CC block ----------------------------------------------------------
    void notifyEntry(cc_termination termination);
    bool setRp(rp_value rp);
    void setCallback(cc_callback callback, void* context);
    std::optional<cc_status> readCcStatus();

    // Not part of the concepts: on boards where the TCPC driver owns the
    // VCONN switch, the injected vconn_port forwards here
    bool setVconn(bool enable);

    // --- the PD PHY ------------------------------------------------------------
    void setCallbacks(pd_phy_callbacks const& callbacks, void* context);
    bool setMessageHeaderInfo(message_header_info info);
    bool setReceiveDetect(receive_detect detect);
    bool selectCcLine(plug_orientation line);
    bool transmit(pd_message const& message);
    bool transmit(transmit_signal signal);

private:
    static void alert(device const* dev, void* data, tcpc_alert alert);
    static void deliver(k_work* work);
    bool applyTermination();
    void reportCc();
    void reportReceived();

    device const* dev_;
    k_work alert_work_{};
    cc_callback cc_callback_ = nullptr;
    void* cc_context_ = nullptr;
    pd_phy_callbacks phy_callbacks_{};
    void* phy_context_ = nullptr;
    atomic_t pending_ = ATOMIC_INIT(0);
    cc_pull pull_ = cc_pull::open;
    rp_value rp_ = rp_value::usb_default;
};

static_assert(concepts::cc<Tcpc>);
static_assert(concepts::pd_phy<Tcpc>);

} // namespace usbc::zephyr
