/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/Tcpc.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/logging/log.h>

#include <algorithm>
#include <cstring>

LOG_MODULE_REGISTER(usb_typec, CONFIG_USB_TYPEC_STACK_LOG_LEVEL);

namespace usbc::zephyr {

namespace {

// The driver's alerts, accumulated until the work queue delivers them
enum class pending : atomic_val_t {
    none = 0,
    cc_changed = 1u << 0,
    message_received = 1u << 1,
    transmit_success = 1u << 2,
    transmit_discarded = 1u << 3,
    transmit_failed = 1u << 4,
    hard_reset_received = 1u << 5,
};

constexpr pending toPending(tcpc_alert alert)
{
    switch (alert) {
    case TCPC_ALERT_CC_STATUS: return pending::cc_changed;
    case TCPC_ALERT_MSG_STATUS: return pending::message_received;
    case TCPC_ALERT_TRANSMIT_MSG_SUCCESS: return pending::transmit_success;
    case TCPC_ALERT_TRANSMIT_MSG_DISCARDED: return pending::transmit_discarded;
    case TCPC_ALERT_TRANSMIT_MSG_FAILED: return pending::transmit_failed;
    case TCPC_ALERT_HARD_RESET_RECEIVED: return pending::hard_reset_received;
    default: return pending::none;
    }
}

constexpr bool has(atomic_val_t bits, pending bit)
{
    return (bits & static_cast<atomic_val_t>(bit)) != 0;
}

constexpr tc_cc_pull toZephyr(cc_pull pull)
{
    switch (pull) {
    case cc_pull::ra: return TC_CC_RA;
    case cc_pull::rp: return TC_CC_RP;
    case cc_pull::rd: return TC_CC_RD;
    default: return TC_CC_OPEN;
    }
}

constexpr tc_rp_value toZephyr(rp_value rp)
{
    switch (rp) {
    case rp_value::p_1a5: return TC_RP_1A5;
    case rp_value::p_3a0: return TC_RP_3A0;
    default: return TC_RP_USB;
    }
}

constexpr pd_packet_type toZephyr(sop_type sop)
{
    switch (sop) {
    case sop_type::sop_prime: return PD_PACKET_SOP_PRIME;
    case sop_type::sop_double_prime: return PD_PACKET_PRIME_PRIME;
    case sop_type::sop_prime_debug: return PD_PACKET_DEBUG_PRIME;
    case sop_type::sop_double_prime_debug: return PD_PACKET_DEBUG_PRIME_PRIME;
    default: return PD_PACKET_SOP;
    }
}

constexpr sop_type fromZephyr(pd_packet_type type)
{
    switch (type) {
    case PD_PACKET_SOP_PRIME: return sop_type::sop_prime;
    case PD_PACKET_PRIME_PRIME: return sop_type::sop_double_prime;
    case PD_PACKET_DEBUG_PRIME: return sop_type::sop_prime_debug;
    case PD_PACKET_DEBUG_PRIME_PRIME: return sop_type::sop_double_prime_debug;
    default: return sop_type::sop;
    }
}

// Zephyr reports voltage states; our cc_state also encodes the role
// side, and an open reading depends on the presented pull
constexpr cc_state fromVoltage(tc_cc_voltage_state state, cc_pull presented)
{
    switch (state) {
    case TC_CC_VOLT_RA: return cc_state::src_ra;
    case TC_CC_VOLT_RD: return cc_state::src_rd;
    case TC_CC_VOLT_RP_DEF: return cc_state::snk_default;
    case TC_CC_VOLT_RP_1A5: return cc_state::snk_power_1a5;
    case TC_CC_VOLT_RP_3A0: return cc_state::snk_power_3a0;
    default: return presented == cc_pull::rd ? cc_state::snk_open : cc_state::src_open;
    }
}

} // namespace

Tcpc::Tcpc(device const* dev) : dev_(dev)
{
    k_work_init(&alert_work_, &Tcpc::deliver);
    int const ret = tcpc_set_alert_handler_cb(dev_, &Tcpc::alert, this);
    if (ret != 0) {
        LOG_ERR("registering the alert handler failed (%d)", ret);
    }
}

// --- the CC block --------------------------------------------------------------

void Tcpc::notifyEntry(cc_termination termination)
{
    pull_ = termination.pull;
    applyTermination();
}

// The Rp presented while pulling up: a change while Rp is presented
// (the source's SinkTxOk/SinkTxNG) takes effect at once
bool Tcpc::setRp(rp_value rp)
{
    rp_ = rp;
    int const ret = tcpc_select_rp_value(dev_, toZephyr(rp));
    if (ret != 0 && ret != -ENOSYS) {
        LOG_ERR("selecting the Rp value failed (%d)", ret);
        return false;
    }
    return pull_ != cc_pull::rp || applyTermination();
}

bool Tcpc::applyTermination()
{
    int const ret = tcpc_set_cc(dev_, toZephyr(pull_));
    if (ret != 0) {
        LOG_ERR("setting the CC pull failed (%d)", ret);
    }
    return ret == 0;
}

void Tcpc::setCallback(cc_callback callback, void* context)
{
    cc_callback_ = callback;
    cc_context_ = context;
}

std::optional<cc_status> Tcpc::readCcStatus()
{
    tc_cc_voltage_state cc1;
    tc_cc_voltage_state cc2;
    int const ret = tcpc_get_cc(dev_, &cc1, &cc2);
    if (ret != 0) {
        LOG_ERR("reading the CC status failed (%d)", ret);
        return std::nullopt;
    }
    LOG_DBG("cc1 %d, cc2 %d (pull %d)", cc1, cc2, static_cast<int>(pull_));
    return cc_status{fromVoltage(cc1, pull_), fromVoltage(cc2, pull_)};
}

bool Tcpc::setVconn(bool enable)
{
    int const ret = tcpc_set_vconn(dev_, enable);
    if (ret != 0) {
        LOG_ERR("switching VCONN %s failed (%d)", enable ? "on" : "off", ret);
    }
    LOG_DBG("VCONN %s", enable ? "on" : "off");
    return ret == 0;
}

// --- the PD PHY ----------------------------------------------------------------

void Tcpc::setCallbacks(pd_phy_callbacks const& callbacks, void* context)
{
    phy_callbacks_ = callbacks;
    phy_context_ = context;
}

bool Tcpc::setMessageHeaderInfo(message_header_info info)
{
    int const ret = tcpc_set_roles(
        dev_,
        info.power == power_role::source ? TC_ROLE_SOURCE : TC_ROLE_SINK,
        info.data == data_role::dfp ? TC_ROLE_DFP : TC_ROLE_UFP
    );
    if (ret != 0) {
        LOG_ERR("setting the message header roles failed (%d)", ret);
    }
    return ret == 0;
}

bool Tcpc::setReceiveDetect(receive_detect detect)
{
    constexpr auto prime_group = receive_detect::sop_prime | receive_detect::sop_double_prime |
                                 receive_detect::sop_prime_debug |
                                 receive_detect::sop_double_prime_debug;
    int const prime = tcpc_sop_prime_enable(dev_, any(detect & prime_group));
    if (prime != 0 && prime != -ENOSYS) {
        LOG_ERR("enabling SOP' reception failed (%d)", prime);
        return false;
    }
    int const ret = tcpc_set_rx_enable(dev_, detect != receive_detect::none);
    if (ret != 0) {
        LOG_ERR("enabling message reception failed (%d)", ret);
    }
    return ret == 0;
}

bool Tcpc::selectCcLine(plug_orientation line)
{
    int const ret = tcpc_set_cc_polarity(
        dev_,
        line == plug_orientation::cc1 ? TC_POLARITY_CC1 : TC_POLARITY_CC2
    );
    if (ret != 0) {
        LOG_ERR("selecting the CC line failed (%d)", ret);
    }
    LOG_DBG("PD on cc%d", line == plug_orientation::cc1 ? 1 : 2);
    return ret == 0;
}

bool Tcpc::transmit(pd_message const& message)
{
    pd_msg msg{};
    msg.type = toZephyr(message.sop);
    msg.header.raw_value = message.header;
    msg.len = message.payload_size;
    std::memcpy(msg.data, message.payload.data(), message.payload_size);
    int const ret = tcpc_transmit_data(dev_, &msg);
    if (ret != 0) {
        LOG_ERR("message transmission failed (%d)", ret);
    }
    return ret == 0;
}

bool Tcpc::transmit(transmit_signal signal)
{
    pd_msg msg{};
    switch (signal) {
    case transmit_signal::hard_reset: msg.type = PD_PACKET_TX_HARD_RESET; break;
    case transmit_signal::cable_reset: msg.type = PD_PACKET_CABLE_RESET; break;
    case transmit_signal::bist_carrier_mode_2: msg.type = PD_PACKET_TX_BIST_MODE_2; break;
    }
    int const ret = tcpc_transmit_data(dev_, &msg);
    if (ret != 0) {
        LOG_ERR("signal transmission failed (%d)", ret);
    }
    return ret == 0;
}

// --- the alerts ----------------------------------------------------------------

void Tcpc::alert(device const*, void* data, tcpc_alert alert)
{
    auto* self = static_cast<Tcpc*>(data);
    auto const bit = toPending(alert);
    if (bit == pending::none) {
        LOG_DBG("unhandled TCPC alert %d", static_cast<int>(alert));
        return;
    }
    atomic_or(&self->pending_, static_cast<atomic_val_t>(bit));
    k_work_submit_to_queue(&workQueue(), &self->alert_work_);
}

// In the stack's context: the CC block's report, then the PHY's in the
// order the protocol layer wants them - a hard reset ahead of a
// transmit outcome, the messages last
void Tcpc::deliver(k_work* work)
{
    auto* self = CONTAINER_OF(work, Tcpc, alert_work_);
    auto const bits = atomic_clear(&self->pending_);
    if (has(bits, pending::cc_changed)) {
        self->reportCc();
    }
    auto const& phy = self->phy_callbacks_;
    if (has(bits, pending::hard_reset_received) && phy.hard_reset_received != nullptr) {
        phy.hard_reset_received(self->phy_context_);
    }
    if (phy.transmitted != nullptr) {
        if (has(bits, pending::transmit_success)) {
            phy.transmitted(self->phy_context_, transmit_outcome::success);
        }
        if (has(bits, pending::transmit_discarded)) {
            phy.transmitted(self->phy_context_, transmit_outcome::discarded);
        }
        if (has(bits, pending::transmit_failed)) {
            phy.transmitted(self->phy_context_, transmit_outcome::failed);
        }
    }
    if (has(bits, pending::message_received)) {
        self->reportReceived();
    }
}

void Tcpc::reportCc()
{
    if (cc_callback_ == nullptr) {
        return;
    }
    if (auto const cc = readCcStatus()) {
        cc_callback_(cc_context_, *cc);
    }
}

// Every pending message, in order; without a holder of the PHY the
// messages are drained unheard
void Tcpc::reportReceived()
{
    pd_msg msg{};
    while (tcpc_get_rx_pending_msg(dev_, &msg) >= 0) {
        if (phy_callbacks_.received == nullptr) {
            continue;
        }
        pd_message message{};
        message.sop = fromZephyr(msg.type);
        message.header = msg.header.raw_value;
        message.payload_size = static_cast<std::uint8_t>(
            std::min<std::uint32_t>(msg.len, pd_message::max_payload_size)
        );
        std::memcpy(message.payload.data(), msg.data, message.payload_size);
        phy_callbacks_.received(phy_context_, message);
    }
}

} // namespace usbc::zephyr
