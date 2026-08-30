/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/WorkQueue.hpp>

#include <zephyr/init.h>

namespace usbc::zephyr {

namespace {

K_KERNEL_STACK_DEFINE(work_queue_stack, CONFIG_USB_TYPEC_STACK_STACK_SIZE);

k_work_q work_queue;

int startWorkQueue()
{
    static constexpr k_work_queue_config config{.name = "usb_typec"};
    k_work_queue_start(&work_queue, work_queue_stack, K_KERNEL_STACK_SIZEOF(work_queue_stack),
                       CONFIG_USB_TYPEC_STACK_THREAD_PRIORITY, &config);
    return 0;
}

SYS_INIT(startWorkQueue, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

} // namespace

k_work_q& workQueue()
{
    return work_queue;
}

Timer::Timer() : mtl::zephyr::WorkqueueTimer(&workQueue()) {}

} // namespace usbc::zephyr
