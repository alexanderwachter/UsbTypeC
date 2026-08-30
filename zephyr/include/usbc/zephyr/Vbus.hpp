/*
 * Adapter from a Zephyr VBUS device driver
 * (zephyr/drivers/usb_c/usbc_vbus.h) to the stack's event-driven vbus
 * concept. Zephyr's driver is poll-based, so the adapter samples
 * check_level() from a delayable work item on the stack's work queue
 * (WorkQueue.hpp) and reports through the callback on every change -
 * plus once after monitor(), as the concept requires.
 * sink_disconnect_pd has no Zephyr level and is approximated with
 * TC_VBUS_REMOVED.
 *
 * The instance is pinned: the work item holds its address.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/Vbus.hpp>

#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/kernel.h>

#include <chrono>

namespace usbc::zephyr {

class Vbus {
public:
    explicit Vbus(device const* dev,
                  std::chrono::milliseconds poll_interval = std::chrono::milliseconds{5});
    Vbus(Vbus const&)            = delete;
    Vbus& operator=(Vbus const&) = delete;

    bool enable(bool on);
    void setCallback(vbus_callback callback, void* context);
    bool monitor(vbus_level level);
    bool discharge(bool on);

private:
    static void poll(k_work* work);
    void schedule(k_timeout_t delay);

    device const* dev_;
    k_work_delayable work_{};
    std::chrono::milliseconds poll_interval_;
    vbus_callback callback_ = nullptr;
    void* context_          = nullptr;
    tc_vbus_level level_    = TC_VBUS_PRESENT;
    bool met_               = false;
    bool report_pending_    = false;
};

static_assert(concepts::vbus<Vbus>);

} // namespace usbc::zephyr
