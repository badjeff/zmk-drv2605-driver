/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_drv2605

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zmk/drivers/drv2605.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv2605, CONFIG_DRV2605_LOG_LEVEL);

/* device data structure */
struct drv2605_data {
    const struct device *dev;
    struct k_work_delayable init_work; // the work structure for delayable init steps
    int async_init_step;
    bool ready; // whether init is finished successfully
    int err; // error code during async init
    struct k_work_delayable standby_work;
    bool standby;
};

/* device config data structure */
struct drv2605_config {
    uint16_t library;
    struct i2c_dt_spec i2c_bus;
    bool has_standby_ms;
    uint32_t standby_ms;
};

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum drv2605_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // init and assert power-up reset
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like library selection
    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    // The device will perform a quick startup sequence (250 μs) and go into STANDBY mode.
    // Delay 1ms, make sure the device is startup is finished.
    [ASYNC_INIT_STEP_POWER_UP] = 1,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int drv2605_async_init_power_up(const struct device *dev);
static int drv2605_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = drv2605_async_init_power_up,
    [ASYNC_INIT_STEP_CONFIGURE] = drv2605_async_init_configure,
};

static int set_mode(const struct device *dev, uint8_t mode) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("Setting MODE to 0x%x", mode);
    int err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_MODE, mode);
    if (err) {
        LOG_ERR("Failed to set MODE");
        return err;
    }
    return 0;
}

static int set_standby(const struct device *dev, bool standby) {
    struct drv2605_data *data = dev->data;
    const struct drv2605_config *config = dev->config;
    int err = 0;
    if (!data->standby && standby) {
        LOG_DBG("Setting MODE to 0x%x", DRV2605_MODE_DIAGNOS);
        err = i2c_reg_write_byte_dt(&config->i2c_bus,
                                    DRV2605_REG_MODE, DRV2605_MODE_DIAGNOS);
        if (err) {
            LOG_ERR("Failed to get into standby mode");
            return err;
        }
        data->standby = true;
    }
    else if (data->standby && !standby) {
        LOG_DBG("Setting MODE to 0x%x", DRV2605_MODE_INTTRIG);
        err = i2c_reg_write_byte_dt(&config->i2c_bus,
                                    DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
        if (err) {
            LOG_ERR("Failed to get out standby mode");
            return err;
        }
        data->standby = false;
    }
    return 0;
}

static int set_stop(const struct device *dev) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("");
    int err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_GO, 0);
    if (err) {
        LOG_ERR("Failed to set GO=0");
        return err;
    }
    return 0;
}

static int set_go(const struct device *dev) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("");
    int err = 0;

    if (config->has_standby_ms) {
        err = set_standby(dev, false);
        if (err) {
            LOG_ERR("Failed to toggle standby");
            return err;
        }
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_GO, 1);
    if (err) {
        LOG_ERR("Failed to set GO=1");
        return err;
    }

    if (config->has_standby_ms) {
        // LOG_DBG("schedule standby work after %d ms", config->standby_ms);
        struct drv2605_data *data = dev->data;
        k_work_schedule(&data->standby_work, K_MSEC(config->standby_ms));
    }

    return 0;
}

static int set_waveform(const struct device *dev, uint8_t slot, uint8_t w) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("");
    int err = 0;

    LOG_DBG("Setting WAVESEQ1 [%d] to 0x%x", slot, w);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_WAVESEQ1 + slot, w);
    if (err) {
        LOG_ERR("Failed to set WAVESEQ1");
        return err;
    }

    return 0;
}

static int set_library(const struct device *dev, uint8_t library) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("");
    int err = 0;

    LOG_DBG("Setting LIBRARY to 0x%x", library);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_LIBRARY, library);
    if (err) {
        LOG_ERR("Failed to set LIBRARY");
        return err;
    }

    return 0;
}

static int set_use_lra(const struct device *dev, bool lra_mode) {
    const struct drv2605_config *config = dev->config;
    LOG_DBG("");
    int err = 0;

    LOG_DBG("Setting lra_mode to %s", lra_mode ? "true" : "false");
    // setup N_ERM_LRA bit

    uint8_t feedback;
    err = i2c_burst_read_dt(&config->i2c_bus, DRV2605_REG_FEEDBACK, &feedback, 1);
    if (err) {
        LOG_ERR("Failed to get FEEDBACK");
        return err;
    }
    LOG_DBG("Reg FEEDBACK: 0x%x", feedback);

    uint8_t new_fb = feedback;
    if (lra_mode) {
        new_fb = feedback | 0x80;
    } else {
        new_fb = feedback & 0x7F;
    }
    LOG_DBG("Setting FEEDBACK to 0x%x", new_fb);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_FEEDBACK, new_fb);
    if (err) {
        LOG_ERR("Failed to set FEEDBACK");
        return err;
    }

    uint8_t wrote_fb = 0;
    err = i2c_burst_read_dt(&config->i2c_bus, DRV2605_REG_FEEDBACK, &wrote_fb, 1);
    if (err) {
        LOG_ERR("Failed to get FEEDBACK");
        return err;
    }
    LOG_DBG("Wrote FEEDBACK: 0x%x", wrote_fb);

    // turn on OPEN_LOOP, setup ERM_OPEN_LOOP or LRA_OPEN_LOOP bit

    uint8_t control3;
    err = i2c_burst_read_dt(&config->i2c_bus, DRV2605_REG_CONTROL3, &control3, 1);
    if (err) {
        LOG_ERR("Failed to get CONTROL3");
        return err;
    }
    LOG_DBG("Reg CONTROL3: 0x%x", control3);
    uint8_t new_ctrl3 = feedback;
    if (lra_mode) {
        new_ctrl3 = feedback | 0x05;
    } else {
        new_ctrl3 = feedback | 0x20;
    }
    LOG_DBG("Setting CONTROL3 to 0x%x", new_ctrl3);
    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_CONTROL3, new_ctrl3);
    if (err) {
        LOG_ERR("Failed to set CONTROL3");
        return err;
    }

    uint8_t wrote_ctrl3 = 0;
    err = i2c_burst_read_dt(&config->i2c_bus, DRV2605_REG_CONTROL3, &wrote_ctrl3, 1);
    if (err) {
        LOG_ERR("Failed to get CONTROL3");
        return err;
    }
    LOG_DBG("Wrote CONTROL3: 0x%x", wrote_ctrl3);

    return 0;
}

static int drv2605_async_init_power_up(const struct device *dev) {
    int err = 0;
    // struct drv2605_data *data = dev->data;
    const struct drv2605_config *config = dev->config;
    LOG_DBG("device initialised at 0x%x", config->i2c_bus.addr);

    uint8_t status;
    err = i2c_burst_read_dt(&config->i2c_bus, DRV2605_REG_STATUS, &status, 1);
    if (err) {
        LOG_ERR("Failed to get STATUS");
        return err;
    }
    LOG_DBG("Reg STATUS: 0x%x", status);

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_MODE, 0x00);
    if (err) {
        LOG_ERR("Failed to set MODE");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_RTPIN, 0x00);
    if (err) {
        LOG_ERR("Failed to set RTPIN");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_OVERDRIVE, 0x00);
    if (err) {
        LOG_ERR("Failed to set OVERDRIVE");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_SUSTAINPOS, 0x00);
    if (err) {
        LOG_ERR("Failed to set SUSTAINPOS");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_SUSTAINNEG, 0x00);
    if (err) {
        LOG_ERR("Failed to set SUSTAINNEG");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_BREAK, 0x00);
    if (err) {
        LOG_ERR("Failed to set BREAK");
        return err;
    }

    err = i2c_reg_write_byte_dt(&config->i2c_bus, DRV2605_REG_AUDIOMAX, 0x64);
    if (err) {
        LOG_ERR("Failed to set AUDIOMAX");
        return err;
    }

    return 0;
}

static int drv2605_async_init_configure(const struct device *dev) {
    struct drv2605_data *data = dev->data;
    const struct drv2605_config *config = dev->config;
    int err = 0;

    if (!err) {
        err = set_library(dev, config->library);
    }

    if (!err) {
        err = set_use_lra(dev, config->library == 6);
    }

    if (!err) {
        // make it standby until set_go triggered
        err = set_mode(dev, DRV2605_MODE_DIAGNOS);
        data->standby = true;
    }
    
    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

static void drv2605_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct drv2605_data *data = CONTAINER_OF(work2, struct drv2605_data, init_work);
    const struct device *dev = data->dev;

    LOG_DBG("DRV2605 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("DRV2605 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_DBG("DRV2605 initialized");

            // LOG_DBG("start startup sequence");
            // for (int s = 1, eff = s; eff <= s+(3*2); eff += 3) {
            //     LOG_DBG("start effect %d", eff);
            //     set_waveform(dev, 0, eff);
            //     set_waveform(dev, 1, 0);
            //     set_go(dev);
            //     k_msleep(150);
            // }

        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static void drv2605_standby_cb(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct drv2605_data *data = CONTAINER_OF(work_delayable,
                                             struct drv2605_data,
                                             standby_work);
    // const struct drv2605_config *cfg = data->dev->config; 
    set_standby(data->dev, true);
}

static int drv2605_init(const struct device *dev) {
    struct drv2605_data *data = dev->data;
    const struct drv2605_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_WRN("i2c bus not ready!");
        return -EINVAL;
    }

    if (config->has_standby_ms) {
        k_work_init_delayable(&data->standby_work, drv2605_standby_cb);
    }

    LOG_DBG("device initialised at 0x%x", config->i2c_bus.addr);

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, drv2605_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

static int drv2605_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct drv2605_data *data = dev->data;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case DRV2605_ATTR_LIBRARY:
        err = set_library(dev, DRV2605_SVALUE_TO_LIBRARY(*val));
        break;

    case DRV2605_ATTR_WAVEFORM:
        err = set_waveform(dev, DRV2605_SVALUE_TO_WAVEFORM_SLOT(*val),
                                DRV2605_SVALUE_TO_WAVEFORM_EFFECT(*val));
        break;

    case DRV2605_ATTR_GO:
        err = set_go(dev);
        break;

    case DRV2605_ATTR_STOP:
        err = set_stop(dev);
        break;

    case DRV2605_ATTR_MODE:
        err = set_mode(dev, DRV2605_SVALUE_TO_MODE(*val));
        break;
    
    case DRV2605_ATTR_STANDBY_MODE:
        err = set_standby(dev, DRV2605_SVALUE_TO_BOOL(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api drv2605_driver_api = {
    .attr_set = drv2605_attr_set,
};

#define DRV2605_DEFINE(n)                                                           \
    static struct drv2605_data data##n;                                             \
    static const struct drv2605_config config##n = {                                \
        .library = DT_PROP(DT_DRV_INST(n), library),                                \
        .i2c_bus = I2C_DT_SPEC_INST_GET(n),                                         \
        .has_standby_ms = DT_INST_NODE_HAS_PROP(n, standby_ms),                     \
        .standby_ms = COND_CODE_1(                                                  \
            DT_INST_NODE_HAS_PROP(n, standby_ms),                                   \
            (DT_INST_PROP(n, standby_ms)), (0)),                                    \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, drv2605_init, NULL,                                    \
                          &data##n, &config##n, POST_KERNEL,                        \
                          CONFIG_SENSOR_INIT_PRIORITY, &drv2605_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRV2605_DEFINE)
