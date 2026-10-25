/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/Vbus.hpp>
#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/drivers/usb_c/usbc_ppc.h>
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

Vbus::Vbus(device const* dev, std::chrono::milliseconds poll_interval)
    : dev_(dev), poll_interval_(poll_interval)
{
    k_work_init_delayable(&work_, &Vbus::poll);
}

Vbus::Vbus(device const* dev, device const* ppc, std::chrono::milliseconds poll_interval)
    : dev_(dev), ppc_(ppc), poll_interval_(poll_interval)
{
    k_work_init_delayable(&work_, &Vbus::poll);
}

// The switch position: what the new position does not want goes off
// first, then the wanted path closes - never two paths at once
void Vbus::notifyEntry(vbus_power power)
{
    auto const path = power.path;
    if (path != vbus_path::safe0v) {
        discharge(false);
    }
    if (path != vbus_path::safe5v) {
        sourcePath(false);
    }
    if (path != vbus_path::sink) {
        sinkPath(false);
    }
    switch (path) {
    case vbus_path::sink: sinkPath(true); break;
    case vbus_path::safe5v: sourcePath(true); break;
    case vbus_path::safe0v: discharge(true); break;
    case vbus_path::open: break;
    }
}

void Vbus::notifyEntry(vbus_level level)
{
    monitored_ = level;
    if (level == vbus_level::unwatched) {
        // monitoring off: stop the sampling entirely
        k_work_cancel_delayable(&work_);
        return;
    }
    report_pending_ = true; // contract: report the current state once known
    schedule(K_NO_WAIT);
}

bool Vbus::check(vbus_at_safe0v)
{
    return meets(vbus_level::safe0v);
}

bool Vbus::check(vbus_at_safe5v)
{
    return meets(vbus_level::safe5v);
}

void Vbus::setCallback(vbus_callback callback, void* context)
{
    callback_ = callback;
    context_ = context;
}

bool Vbus::meets(vbus_level level)
{
    return usbc_vbus_check_level(dev_, toZephyr(level));
}

// A board without a power path controller has no switch to drive: the
// path is its own, as with a port protector's gate
bool Vbus::sourcePath(bool on)
{
    if (ppc_ == nullptr) {
        return true;
    }
    int const ret = ppc_set_src_ctrl(ppc_, on);
    if (ret != 0 && ret != -ENOSYS) {
        LOG_ERR("switching the source path %s failed (%d)", on ? "on" : "off", ret);
        return false;
    }
    LOG_DBG("source path %s", on ? "on" : "off");
    return true;
}

bool Vbus::sinkPath(bool on)
{
    if (ppc_ == nullptr) {
        return true;
    }
    int const ret = ppc_set_snk_ctrl(ppc_, on);
    if (ret != 0 && ret != -ENOSYS) {
        LOG_ERR("switching the sink path %s failed (%d)", on ? "on" : "off", ret);
        return false;
    }
    return true;
}

bool Vbus::discharge(bool on)
{
    int const ret =
        ppc_ != nullptr ? ppc_set_vbus_discharge(ppc_, on) : usbc_vbus_discharge(dev_, on);
    if (ret != 0 && ret != -ENOSYS) {
        LOG_ERR("switching VBUS discharge %s failed (%d)", on ? "on" : "off", ret);
        return false;
    }
    return true;
}

void Vbus::poll(k_work* work)
{
    auto* delayable = k_work_delayable_from_work(work);
    auto* self = CONTAINER_OF(delayable, Vbus, work_);

    bool const met = self->meets(self->monitored_);
    if ((self->report_pending_ || met != self->met_) && self->callback_ != nullptr) {
        self->report_pending_ = false;
        self->met_ = met;
        self->callback_(self->context_, self->monitored_, met);
    }
    self->schedule(K_MSEC(self->poll_interval_.count()));
}

void Vbus::schedule(k_timeout_t delay)
{
    k_work_reschedule_for_queue(&workQueue(), &work_, delay);
}

} // namespace usbc::zephyr
