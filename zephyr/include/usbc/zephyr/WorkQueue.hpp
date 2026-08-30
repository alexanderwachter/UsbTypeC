/*
 * The stack's work queue: one dedicated thread on which the Zephyr
 * adaptation layer serializes everything - TCPC alert delivery, VBUS
 * monitoring, and the state machine timers. Its priority and stack
 * size come from Kconfig (USB_TYPEC_STACK_THREAD_PRIORITY,
 * USB_TYPEC_STACK_STACK_SIZE); the queue is started by SYS_INIT before
 * the application runs.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/zephyr/Timer.hpp>

#include <zephyr/kernel.h>

namespace usbc::zephyr {

k_work_q& workQueue();

// Timer policy for the stack's state machines, pinned to the stack's
// work queue
class Timer : public mtl::zephyr::WorkqueueTimer {
public:
    Timer();
};

} // namespace usbc::zephyr
