/*
 * Copyright (c) 2026 badjeff
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_drv2605_alt_gpio

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stddef.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

LOG_MODULE_REGISTER(drv2605_alt_gpio, CONFIG_DRV2605_ALT_LOG_LEVEL);

#if CONFIG_DRV2605_ALT
#include <zmk/drivers/drv2605_alt.h>
#endif

struct drv2605_alt_gpio_config {
	struct gpio_driver_config common;
	uint16_t dev_reg;
};

struct drv2605_alt_gpio_data {
	struct gpio_driver_data data;
	struct k_spinlock lock;
	uint32_t flags_mask;
	bool busy;
};

static int drv2605_alt_gpio_pin_configure(const struct device *dev,
					   gpio_pin_t pin, gpio_flags_t flags)
{
	ARG_UNUSED(pin);
	struct drv2605_alt_gpio_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == 0) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}
	if ((flags & GPIO_OUTPUT) == 0) {
		k_spin_unlock(&data->lock, key);
		return -ENOTSUP;
	}

	data->flags_mask = flags;

	k_spin_unlock(&data->lock, key);
	return 0;
}

#if CONFIG_DRV2605_ALT

static int drv2605_alt_gpio_set_value(const struct device *dev, uint8_t value) {
	// LOG_DBG("%d", value);

	const struct drv2605_alt_gpio_config *cfg = dev->config;
	struct drv2605_alt_gpio_data *data = dev->data;

	const struct device *fb_dev = drv2605_alt_get_dev_by_reg(cfg->dev_reg);
	if (!fb_dev) {
		LOG_WRN("No drv2605 device assigned");
		return -EIO;
	}

	if (data->busy) {
		LOG_WRN("drv2605 is busy");
		return -EBUSY;
	}
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	data->busy = true;

	int rc = 0;
	int err = 0;
	struct sensor_value val = {.val1 = 0, .val2 = 0};

	if (value) {
		val.val1 = 0;     // slot 0
		val.val2 = data->flags_mask; // waveformm
		err = sensor_attr_set(fb_dev, SENSOR_CHAN_ALL,
													(enum sensor_attribute)DRV2605_ATTR_WAVEFORM, &val);
		if (err) {
			LOG_WRN("Fail to sensor_attr_set DRV2605_ATTR_WAVEFORM");
			rc = -EIO;
			goto exit;
		}

		val.val1 = 1; // slot 1
		val.val2 = 0; // waveformm - empty
		err = sensor_attr_set(fb_dev, SENSOR_CHAN_ALL,
													(enum sensor_attribute)DRV2605_ATTR_WAVEFORM, &val);
		if (err) {
			LOG_WRN("Fail to sensor_attr_set DRV2605_ATTR_WAVEFORM");
			rc = -EIO;
			goto exit;
		}

		err = sensor_attr_set(fb_dev, SENSOR_CHAN_ALL, (enum sensor_attribute)DRV2605_ATTR_GO,
													&val);
		if (err) {
			LOG_WRN("Fail to sensor_attr_set DRV2605_ATTR_GO");
			rc = -EIO;
			goto exit;
		}
	} else {
		err = sensor_attr_set(fb_dev, SENSOR_CHAN_ALL,
													(enum sensor_attribute)DRV2605_ATTR_WAVEFORM, &val);
		if (err) {
			LOG_WRN("Fail to sensor_attr_set DRV2605_ATTR_WAVEFORM");
			rc = -EIO;
			goto exit;
		}
	}

exit:
	data->busy = false;
	k_spin_unlock(&data->lock, key);
	return rc;
}

#endif /* #if CONFIG_DRV2605_ALT */

static int drv2605_alt_gpio_port_set_bits_raw(const struct device *dev,
						gpio_port_pins_t mask)
{
	// const struct drv2605_alt_gpio_config *cfg = dev->config;
	struct drv2605_alt_gpio_data *data = dev->data;

#if CONFIG_DRV2605_ALT
	return drv2605_alt_gpio_set_value(dev, 1);
#else
	return 0;
#endif
}

static int drv2605_alt_gpio_port_clear_bits_raw(const struct device *dev,
						  gpio_port_pins_t mask)
{
	// const struct drv2605_alt_gpio_config *cfg = dev->config;
	struct drv2605_alt_gpio_data *data = dev->data;

#if CONFIG_DRV2605_ALT
	return drv2605_alt_gpio_set_value(dev, 0);
#else
	return 0;
#endif
}

static const struct gpio_driver_api drv2605_alt_gpio_driver_api = {
	.pin_configure = drv2605_alt_gpio_pin_configure,
	.port_set_bits_raw = drv2605_alt_gpio_port_set_bits_raw,
	.port_clear_bits_raw = drv2605_alt_gpio_port_clear_bits_raw,
};

static int drv2605_alt_gpio_init(const struct device *dev)
{
	// ARG_UNUSED(dev);
	struct drv2605_alt_gpio_data *data = dev->data;
	data->busy = false;
	LOG_INF("DRV2605 GPIO initialized");
	return 0;
}

#define DRV2605_GPIO_INST_DEFINE(n) \
	static const struct drv2605_alt_gpio_config drv2605_alt_gpio_config_##n = { \
		.common = GPIO_DT_INST_PORT_PIN_MASK_NGPIOS_EXC(n, 32), \
		.dev_reg = DT_INST_PROP_OR(n, dev_reg, 0), \
	}; \
	static struct drv2605_alt_gpio_data drv2605_alt_gpio_data_##n = { \
		.data = {0}, \
	}; \
	DEVICE_DT_INST_DEFINE(n, drv2605_alt_gpio_init,	NULL,	\
			&drv2605_alt_gpio_data_##n, &drv2605_alt_gpio_config_##n,	\
			POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, \
			&drv2605_alt_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRV2605_GPIO_INST_DEFINE)
