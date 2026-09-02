/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <usbc/zephyr/StateLogger.hpp>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usbc_fsm, CONFIG_USB_TYPEC_STACK_LOG_LEVEL);

namespace usbc::zephyr {

void logInitialState(char const* state)
{
    LOG_DBG("-> %s", state);
}

void logStateChange(char const* from, char const* to)
{
    LOG_DBG("%s -> %s", from, to);
}

} // namespace usbc::zephyr
