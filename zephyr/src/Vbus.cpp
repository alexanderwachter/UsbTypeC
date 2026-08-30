/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/Vbus.hpp>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(usb_typec, CONFIG_USB_TYPEC_STACK_LOG_LEVEL);

namespace usbc::zephyr {

namespace {

constexpr tc_vbus_level toZephyr(vbus_level level)
{
    switch (level) {
    case vbus_level::safe0v: return TC_VBUS_SAFE0V;
    case vbus_level::safe5v: return TC_VBUS_PRESENT;
    default: return TC_VBUS_REMOVED; // sink_disconnect and the PD approximation
    }
}

} // namespace

Vbus::Vbus(device const* dev, k_work_q* queue, std::chrono::milliseconds poll_interval)
    : dev_(dev), queue_(queue), poll_interval_(poll_interval)
{
    k_work_init_delayable(&work_, &Vbus::poll);
}

bool Vbus::enable(bool on)
{
    int const ret = usbc_vbus_enable(dev_, on);
    if (ret != 0 && ret != -ENOSYS) {
        LOG_ERR("enabling VBUS detection failed (%d)", ret);
        return false;
    }
    return true;
}

void Vbus::setCallback(vbus_callback callback, void* context)
{
    callback_ = callback;
    context_  = context;
}

bool Vbus::monitor(vbus_level level)
{
    level_          = toZephyr(level);
    report_pending_ = true; // contract: report the current state once known
    schedule(K_NO_WAIT);
    return true;
}

bool Vbus::discharge(bool on)
{
    int const ret = usbc_vbus_discharge(dev_, on);
    if (ret != 0) {
        LOG_ERR("switching VBUS discharge %s failed (%d)", on ? "on" : "off", ret);
    }
    return ret == 0;
}

void Vbus::poll(k_work* work)
{
    auto* delayable = k_work_delayable_from_work(work);
    auto* self      = CONTAINER_OF(delayable, Vbus, work_);

    bool const met = usbc_vbus_check_level(self->dev_, self->level_);
    if ((self->report_pending_ || met != self->met_) && self->callback_ != nullptr) {
        self->report_pending_ = false;
        self->met_            = met;
        self->callback_(self->context_, met);
    }
    self->schedule(K_MSEC(self->poll_interval_.count()));
}

void Vbus::schedule(k_timeout_t delay)
{
    if (queue_ != nullptr) {
        k_work_reschedule_for_queue(queue_, &work_, delay);
    } else {
        k_work_reschedule(&work_, delay);
    }
}

} // namespace usbc::zephyr
