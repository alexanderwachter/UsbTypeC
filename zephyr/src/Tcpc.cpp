/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/Tcpc.hpp>

#include <zephyr/logging/log.h>

#include <algorithm>
#include <cstring>

LOG_MODULE_REGISTER(usb_typec, CONFIG_USB_TYPEC_STACK_LOG_LEVEL);

namespace usbc::zephyr {

namespace {

constexpr alert_status mapAlert(tcpc_alert alert)
{
    switch (alert) {
    case TCPC_ALERT_CC_STATUS: return alert_status::cc_status_changed;
    case TCPC_ALERT_MSG_STATUS: return alert_status::message_received;
    case TCPC_ALERT_TRANSMIT_MSG_SUCCESS: return alert_status::transmit_success;
    case TCPC_ALERT_TRANSMIT_MSG_DISCARDED: return alert_status::transmit_discarded;
    case TCPC_ALERT_TRANSMIT_MSG_FAILED: return alert_status::transmit_failed;
    case TCPC_ALERT_HARD_RESET_RECEIVED: return alert_status::hard_reset_received;
    case TCPC_ALERT_FAULT_STATUS: return alert_status::fault;
    default: return alert_status::none;
    }
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

Tcpc::Tcpc(device const* dev, k_work_q* queue) : dev_(dev), queue_(queue)
{
    k_work_init(&alert_work_, &Tcpc::notifyWork);
}

void Tcpc::setAlertHandler(alert_callback callback, void* context)
{
    callback_     = callback;
    context_      = context;
    int const ret = tcpc_set_alert_handler_cb(dev_, &Tcpc::alert, this);
    if (ret != 0) {
        LOG_ERR("registering the alert handler failed (%d)", ret);
    }
}

std::optional<alert_status> Tcpc::readAlert()
{
    return static_cast<alert_status>(atomic_clear(&pending_));
}

bool Tcpc::setCc(cc_pull pull, rp_value rp)
{
    if (pull == cc_pull::rp) {
        int const ret = tcpc_select_rp_value(dev_, toZephyr(rp));
        if (ret != 0 && ret != -ENOSYS) {
            LOG_ERR("selecting the Rp value failed (%d)", ret);
            return false;
        }
    }
    pull_         = pull;
    int const ret = tcpc_set_cc(dev_, toZephyr(pull));
    if (ret != 0) {
        LOG_ERR("setting the CC pull failed (%d)", ret);
    }
    return ret == 0;
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
    return cc_status{fromVoltage(cc1, pull_), fromVoltage(cc2, pull_)};
}

bool Tcpc::setPlugOrientation(plug_orientation orientation)
{
    int const ret = tcpc_set_cc_polarity(
        dev_, orientation == plug_orientation::cc1 ? TC_POLARITY_CC1 : TC_POLARITY_CC2);
    if (ret != 0) {
        LOG_ERR("setting the CC polarity failed (%d)", ret);
    }
    return ret == 0;
}

bool Tcpc::sourceVbus(bool enable)
{
    int const ret = tcpc_set_src_ctrl(dev_, enable);
    if (ret != 0) {
        LOG_ERR("switching source VBUS %s failed (%d)", enable ? "on" : "off", ret);
    }
    return ret == 0;
}

bool Tcpc::sinkVbus(bool enable)
{
    int const ret = tcpc_set_snk_ctrl(dev_, enable);
    if (ret != 0) {
        LOG_ERR("switching sink VBUS %s failed (%d)", enable ? "on" : "off", ret);
    }
    return ret == 0;
}

bool Tcpc::setVconn(bool enable)
{
    int const ret = tcpc_set_vconn(dev_, enable);
    if (ret != 0) {
        LOG_ERR("switching VCONN %s failed (%d)", enable ? "on" : "off", ret);
    }
    return ret == 0;
}

bool Tcpc::setMessageHeaderInfo(message_header_info info)
{
    int const ret =
        tcpc_set_roles(dev_, info.power == power_role::source ? TC_ROLE_SOURCE : TC_ROLE_SINK,
                       info.data == data_role::dfp ? TC_ROLE_DFP : TC_ROLE_UFP);
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

bool Tcpc::transmit(pd_message const& message)
{
    pd_msg msg{};
    msg.type             = toZephyr(message.sop);
    msg.header.raw_value = message.header;
    msg.len              = message.payload_size;
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

bool Tcpc::receive(pd_message& out)
{
    pd_msg msg{};
    if (tcpc_get_rx_pending_msg(dev_, &msg) < 0) {
        return false; // nothing pending is a normal outcome
    }
    out.sop          = fromZephyr(msg.type);
    out.header       = msg.header.raw_value;
    out.payload_size = static_cast<std::uint8_t>(
        std::min<std::uint32_t>(msg.len, pd_message::max_payload_size));
    std::memcpy(out.payload.data(), msg.data, out.payload_size);
    return true;
}

void Tcpc::alert(device const*, void* data, tcpc_alert alert)
{
    auto* self      = static_cast<Tcpc*>(data);
    auto const bits = mapAlert(alert);
    if (bits == alert_status::none) {
        LOG_DBG("unhandled TCPC alert %d", static_cast<int>(alert));
        return;
    }
    atomic_or(&self->pending_, static_cast<atomic_val_t>(bits));
    if (self->queue_ != nullptr) {
        k_work_submit_to_queue(self->queue_, &self->alert_work_);
    } else {
        k_work_submit(&self->alert_work_);
    }
}

void Tcpc::notifyWork(k_work* work)
{
    auto* self = CONTAINER_OF(work, Tcpc, alert_work_);
    if (self->callback_ != nullptr) {
        self->callback_(self->context_);
    }
}

} // namespace usbc::zephyr
