/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The TCPP03-M20 beyond the power path controller API
 * (zephyr/drivers/usb_c/usbc_ppc.h): its VCONN switch and VCONN
 * discharge, which the PPC API has no operations for. A UCPD-based
 * port registers them as the TCPC driver's VCONN callbacks
 * (tcpc_set_vconn_cb, tcpc_set_vconn_discharge_cb).
 */

#ifndef USBC_ZEPHYR_DRIVERS_USB_C_TCPP03_M20_H_
#define USBC_ZEPHYR_DRIVERS_USB_C_TCPP03_M20_H_

#include <zephyr/device.h>
#include <zephyr/drivers/usb_c/usbc_tc.h>

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Switch VCONN onto a CC pin, or off both
 *
 * @param dev      TCPP03-M20 device
 * @param polarity The CC pin to carry VCONN
 * @param enable   Close the switch when true, open both when false
 *
 * @retval 0 on success
 * @retval -errno of the I2C transfer on failure
 */
int tcpp03_m20_set_vconn(const struct device *dev, enum tc_cc_polarity polarity, bool enable);

/**
 * @brief Switch the VCONN discharge path
 *
 * @param dev    TCPP03-M20 device
 * @param enable Discharge when true
 *
 * @retval 0 on success
 * @retval -errno of the I2C transfer on failure
 */
int tcpp03_m20_set_vconn_discharge(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* USBC_ZEPHYR_DRIVERS_USB_C_TCPP03_M20_H_ */
