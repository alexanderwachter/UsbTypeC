/*
 * Adapter from a Zephyr VBUS device driver
 * (zephyr/drivers/usb_c/usbc_vbus.h), and optionally a power path
 * controller (usbc_ppc.h), to the stack's vbus concept. Zephyr's VBUS
 * driver is poll-based, so the adapter samples check_level() from a
 * delayable work item on the stack's work queue (WorkQueue.hpp) and
 * reports through the callback on every change - plus once after a
 * state arms a level, as the concept requires. sink_disconnect_pd has
 * no Zephyr level and is approximated with TC_VBUS_REMOVED. A port
 * whose paths sit in a power path controller names its device in the
 * constructor: the states' switch position drives the PPC's source,
 * sink and discharge paths then. Without one, only the VBUS driver's
 * discharge is driven - the paths are the board's own (ucpd_stm32 has
 * no VBUS switches).
 *
 * The instance is pinned: the work item holds its address and the
 * machines hold it as an observer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <usbc/interface/Vbus.hpp>

#include <mtl/StateMachine.hpp>

#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/kernel.h>

#include <chrono>

namespace usbc::zephyr {

class Vbus : public fsm::observing<Vbus> {
public:
    explicit Vbus(
        device const* dev,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds{5}
    );
    Vbus(
        device const* dev,
        device const* ppc,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds{5}
    );
    Vbus(Vbus const&) = delete;
    Vbus& operator=(Vbus const&) = delete;

    void notifyEntry(vbus_power power);
    void notifyEntry(vbus_level level);
    bool check(vbus_at_safe0v);
    bool check(vbus_at_safe5v);
    void setCallback(vbus_callback callback, void* context);

private:
    static void poll(k_work* work);
    void schedule(k_timeout_t delay);
    bool meets(vbus_level level);
    bool sourcePath(bool on);
    bool sinkPath(bool on);
    bool discharge(bool on);

    device const* dev_;
    device const* ppc_ = nullptr;
    k_work_delayable work_{};
    std::chrono::milliseconds poll_interval_;
    vbus_callback callback_ = nullptr;
    void* context_ = nullptr;
    vbus_level monitored_ = vbus_level::unwatched;
    bool met_ = false;
    bool report_pending_ = false;
};

static_assert(concepts::vbus<Vbus>);

} // namespace usbc::zephyr
