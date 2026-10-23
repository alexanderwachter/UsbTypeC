/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_tcpp03_m20

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/usb_c/tcpp03_m20.h>
#include <zephyr/drivers/usb_c/usbc_ppc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tcpp03_m20, CONFIG_USBC_PPC_LOG_LEVEL);

/*
 * Register addresses (DS13618 figure 9): the acknowledge register has the
 * control register's layout and shows what the device applies - the two
 * diverge when a protection trips and the device opens the switches
 */
#define TCPP03_REG_CONTROL     0x00 /* the state as written */
#define TCPP03_REG_ACKNOWLEDGE 0x01 /* the state as applied */
#define TCPP03_REG_FLAGS       0x02 /* faults and VBUS presence */

/* Control and acknowledge register fields */
#define TCPP03_CONTROL_VCONN_CC1       BIT(0)
#define TCPP03_CONTROL_VCONN_CC2       BIT(1)
#define TCPP03_CONTROL_VCONN_MASK      (TCPP03_CONTROL_VCONN_CC1 | TCPP03_CONTROL_VCONN_CC2)
/* Opposite polarities (DS13618 figure 11): GDP=1 closes, GDC=1 opens */
#define TCPP03_CONTROL_PROVIDER_CLOSED BIT(2)
#define TCPP03_CONTROL_CONSUMER_OPEN   BIT(3)
#define TCPP03_CONTROL_POWER_NORMAL    BIT(4)
#define TCPP03_CONTROL_POWER_LOW       BIT(5)
#define TCPP03_CONTROL_POWER_MASK      (TCPP03_CONTROL_POWER_NORMAL | TCPP03_CONTROL_POWER_LOW)
#define TCPP03_CONTROL_POWER_HIBERNATE 0 /* the reset value: dead-battery Rd on CC */
#define TCPP03_CONTROL_VBUS_DISCHARGE  BIT(6)
#define TCPP03_CONTROL_VCONN_DISCHARGE BIT(7)

/* Flag register bits */
#define TCPP03_FLAG_OCP_VCONN BIT(0)
#define TCPP03_FLAG_OCP_VBUS  BIT(1)
#define TCPP03_FLAG_OVP_VBUS  BIT(2)
#define TCPP03_FLAG_OTP       BIT(3)
#define TCPP03_FLAG_OVP_CC    BIT(4)
#define TCPP03_FLAG_VBUS_OK   BIT(5)
#define TCPP03_FLAG_TCPP02    BIT(7) /* set on a TCPP02-M18, clear on a TCPP03-M20 */

struct tcpp03_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec enable_gpio; /* ENABLE, optional: tied high otherwise */
	struct gpio_dt_spec flag_gpio;   /* FLGn, optional */
};

struct tcpp03_data {
	const struct device *dev;
	struct k_mutex lock;
	uint8_t control; /* the control register as last written */
	struct gpio_callback flag_callback;
	struct k_work flag_work;
	usbc_ppc_event_cb_t event_handler;
	void *event_data;
};

static int tcpp03_read_register(const struct device *dev, uint8_t reg, uint8_t *value)
{
	const struct tcpp03_config *config = dev->config;
	int ret = i2c_reg_read_byte_dt(&config->bus, reg, value);

	if (ret != 0) {
		LOG_ERR("reading register %u failed (%d)", reg, ret);
	}

	return ret;
}

/* Reads a field of the acknowledge register: what the device applies */
static int tcpp03_read_acknowledged(const struct device *dev, uint8_t mask)
{
	uint8_t acknowledged;
	int ret = tcpp03_read_register(dev, TCPP03_REG_ACKNOWLEDGE, &acknowledged);

	if (ret != 0) {
		return ret;
	}

	return (acknowledged & mask) != 0;
}

/* Writes the control register and remembers the value; call under the lock */
static int tcpp03_write_control(const struct device *dev, uint8_t value)
{
	const struct tcpp03_config *config = dev->config;
	struct tcpp03_data *data = dev->data;
	int ret = i2c_reg_write_byte_dt(&config->bus, TCPP03_REG_CONTROL, value);

	if (ret != 0) {
		LOG_ERR("writing the control register failed (%d)", ret);
		return ret;
	}

	data->control = value;

	return 0;
}

/* Replaces the fields under mask, keeping the others */
static int tcpp03_update_control(const struct device *dev, uint8_t mask, uint8_t value)
{
	struct tcpp03_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tcpp03_write_control(dev, (data->control & ~mask) | (value & mask));
	k_mutex_unlock(&data->lock);

	return ret;
}

/* PPC API */

static int tcpp03_is_dead_battery_mode(const struct device *dev)
{
	struct tcpp03_data *data = dev->data;

	return (data->control & TCPP03_CONTROL_POWER_MASK) == TCPP03_CONTROL_POWER_HIBERNATE;
}

static int tcpp03_exit_dead_battery_mode(const struct device *dev)
{
	return tcpp03_update_control(dev, TCPP03_CONTROL_POWER_MASK, TCPP03_CONTROL_POWER_NORMAL);
}

static int tcpp03_is_vbus_source(const struct device *dev)
{
	return tcpp03_read_acknowledged(dev, TCPP03_CONTROL_PROVIDER_CLOSED);
}

static int tcpp03_is_vbus_sink(const struct device *dev)
{
	int open = tcpp03_read_acknowledged(dev, TCPP03_CONTROL_CONSUMER_OPEN);

	return open < 0 ? open : !open;
}

static int tcpp03_set_snk_ctrl(const struct device *dev, bool enable)
{
	return tcpp03_update_control(dev, TCPP03_CONTROL_CONSUMER_OPEN,
				     enable ? 0 : TCPP03_CONTROL_CONSUMER_OPEN);
}

static int tcpp03_set_src_ctrl(const struct device *dev, bool enable)
{
	return tcpp03_update_control(dev, TCPP03_CONTROL_PROVIDER_CLOSED,
				     enable ? TCPP03_CONTROL_PROVIDER_CLOSED : 0);
}

static int tcpp03_set_vbus_discharge(const struct device *dev, bool enable)
{
	return tcpp03_update_control(dev, TCPP03_CONTROL_VBUS_DISCHARGE,
				     enable ? TCPP03_CONTROL_VBUS_DISCHARGE : 0);
}

static int tcpp03_is_vbus_present(const struct device *dev)
{
	uint8_t flags;
	int ret = tcpp03_read_register(dev, TCPP03_REG_FLAGS, &flags);

	if (ret != 0) {
		return ret;
	}

	return (flags & TCPP03_FLAG_VBUS_OK) != 0;
}

static int tcpp03_set_event_handler(const struct device *dev, usbc_ppc_event_cb_t handler,
				    void *handler_data)
{
	struct tcpp03_data *data = dev->data;

	data->event_handler = handler;
	data->event_data = handler_data;

	return 0;
}

static int tcpp03_dump_regs(const struct device *dev)
{
	struct tcpp03_data *data = dev->data;
	uint8_t acknowledged;
	uint8_t flags;
	int ret = tcpp03_read_register(dev, TCPP03_REG_ACKNOWLEDGE, &acknowledged);

	if (ret != 0) {
		return ret;
	}

	ret = tcpp03_read_register(dev, TCPP03_REG_FLAGS, &flags);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("control %02x, acknowledged %02x, flags %02x", data->control, acknowledged, flags);

	return 0;
}

static DEVICE_API(usbc_ppc, tcpp03_driver_api) = {
	.is_dead_battery_mode = tcpp03_is_dead_battery_mode,
	.exit_dead_battery_mode = tcpp03_exit_dead_battery_mode,
	.is_vbus_source = tcpp03_is_vbus_source,
	.is_vbus_sink = tcpp03_is_vbus_sink,
	.set_snk_ctrl = tcpp03_set_snk_ctrl,
	.set_src_ctrl = tcpp03_set_src_ctrl,
	.set_vbus_discharge = tcpp03_set_vbus_discharge,
	.is_vbus_present = tcpp03_is_vbus_present,
	.set_event_handler = tcpp03_set_event_handler,
	.dump_regs = tcpp03_dump_regs,
};

/* Beyond the PPC API: the VCONN switch and discharge */

int tcpp03_m20_set_vconn(const struct device *dev, enum tc_cc_polarity polarity, bool enable)
{
	uint8_t pin =
		polarity == TC_POLARITY_CC1 ? TCPP03_CONTROL_VCONN_CC1 : TCPP03_CONTROL_VCONN_CC2;

	return tcpp03_update_control(dev, TCPP03_CONTROL_VCONN_MASK, enable ? pin : 0);
}

int tcpp03_m20_set_vconn_discharge(const struct device *dev, bool enable)
{
	return tcpp03_update_control(dev, TCPP03_CONTROL_VCONN_DISCHARGE,
				     enable ? TCPP03_CONTROL_VCONN_DISCHARGE : 0);
}

/* Faults */

static void tcpp03_report(struct tcpp03_data *data, enum usbc_ppc_event event)
{
	if (data->event_handler != NULL) {
		data->event_handler(data->dev, data->event_data, event);
	}
}

/*
 * An overcurrent opened the VCONN switches and both paths and holds
 * them until the recovery word (DS13618 tables 15 and 16): the power
 * mode with both paths open, which stays the written state
 */
static void tcpp03_recover_from_overcurrent(const struct device *dev)
{
	struct tcpp03_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	tcpp03_write_control(dev, (data->control & TCPP03_CONTROL_POWER_MASK) |
					  TCPP03_CONTROL_CONSUMER_OPEN);
	k_mutex_unlock(&data->lock);
}

static void tcpp03_handle_flags(struct k_work *work)
{
	struct tcpp03_data *data = CONTAINER_OF(work, struct tcpp03_data, flag_work);
	const struct device *dev = data->dev;
	/* the role at the fault: the written intent, the device may have opened the gate */
	bool sourcing = (data->control & TCPP03_CONTROL_PROVIDER_CLOSED) != 0;
	uint8_t flags;

	if (tcpp03_read_register(dev, TCPP03_REG_FLAGS, &flags) != 0) {
		return;
	}

	LOG_DBG("flags %02x", flags);

	if (flags & TCPP03_FLAG_OCP_VBUS) {
		LOG_WRN("VBUS overcurrent while %s", sourcing ? "sourcing" : "sinking");
		tcpp03_recover_from_overcurrent(dev);

		if (sourcing) {
			tcpp03_report(data, USBC_PPC_EVENT_SRC_OVERCURRENT);
		}
	}

	if (flags & TCPP03_FLAG_OCP_VCONN) {
		LOG_WRN("VCONN overcurrent");
		tcpp03_recover_from_overcurrent(dev);
	}

	if (flags & TCPP03_FLAG_OVP_VBUS) {
		LOG_WRN("VBUS overvoltage while %s", sourcing ? "sourcing" : "sinking");
		tcpp03_report(data, sourcing ? USBC_PPC_EVENT_SRC_OVERVOLTAGE
					     : USBC_PPC_EVENT_SNK_OVERVOLTAGE);
	}

	if (flags & TCPP03_FLAG_OVP_CC) {
		LOG_WRN("CC overvoltage");
	}

	if (flags & TCPP03_FLAG_OTP) {
		LOG_WRN("overtemperature");
		tcpp03_report(data, USBC_PPC_EVENT_OVER_TEMPERATURE);
	}
}

static void tcpp03_flag_interrupt(const struct device *port, struct gpio_callback *callback,
				  gpio_port_pins_t pins)
{
	struct tcpp03_data *data = CONTAINER_OF(callback, struct tcpp03_data, flag_callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_work_submit(&data->flag_work);
}

static int tcpp03_init_flag_pin(const struct device *dev)
{
	const struct tcpp03_config *config = dev->config;
	struct tcpp03_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&config->flag_gpio)) {
		LOG_ERR("FLGn GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->flag_gpio, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&data->flag_callback, tcpp03_flag_interrupt, BIT(config->flag_gpio.pin));
	ret = gpio_add_callback_dt(&config->flag_gpio, &data->flag_callback);
	if (ret != 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&config->flag_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static int tcpp03_init(const struct device *dev)
{
	const struct tcpp03_config *config = dev->config;
	struct tcpp03_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	k_work_init(&data->flag_work, tcpp03_handle_flags);

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Out of reset: hibernate, the registers cleared */
	if (config->enable_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->enable_gpio)) {
			LOG_ERR("ENABLE GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->enable_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	data->control = TCPP03_CONTROL_POWER_HIBERNATE;

	/* Normal mode with both paths open: leaving hibernate ends the dead-battery sink path */
	ret = tcpp03_write_control(dev, TCPP03_CONTROL_POWER_NORMAL | TCPP03_CONTROL_CONSUMER_OPEN);
	if (ret != 0) {
		return ret;
	}

	if (config->flag_gpio.port != NULL) {
		ret = tcpp03_init_flag_pin(dev);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

#define TCPP03_DEFINE(inst)                                                                        \
	static const struct tcpp03_config tcpp03_config_##inst = {                                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.enable_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),                  \
		.flag_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),                       \
	};                                                                                         \
	static struct tcpp03_data tcpp03_data_##inst = {                                           \
		.dev = DEVICE_DT_INST_GET(inst),                                                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tcpp03_init, NULL, &tcpp03_data_##inst, &tcpp03_config_##inst, \
			      POST_KERNEL, CONFIG_USBC_PPC_INIT_PRIORITY, &tcpp03_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TCPP03_DEFINE)
