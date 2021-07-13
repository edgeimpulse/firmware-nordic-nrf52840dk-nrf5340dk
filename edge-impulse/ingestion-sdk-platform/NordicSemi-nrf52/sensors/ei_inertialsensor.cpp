/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_config_types.h"
#include "ei_inertialsensor.h"
#include "ei_device_nordic_nrf52.h"
#include "sensor_aq.h"

#include <drivers/i2c.h>


#include <zephyr.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

#define ACC_SAMPLE_TIME_MS  0.912f
#define FLASH_WRITE_TIME_MS 0.5036f

#define ACC_SAMPLE_TIME_US  255
#define FLASH_WRITE_TIME_US 543
#define CORRECTION_TIME_US  400

#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
#define I2C_DEV "I2C_0"
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
#define I2C_DEV "I2C_1"
#else 
#error "Unsupported build target was chosen!"
#endif

extern ei_config_t *ei_config_get_config();
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

/* Private function prototypes --------------------------------------------- */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
/* Private variables ------------------------------------------------------- */
stmdev_ctx_t dev_ctx;

sampler_callback  cb_sampler;

static float acceleration_g[N_AXIS_SAMPLED];
int16_t data_raw_acceleration[N_AXIS_SAMPLED];

int32_t sample_interval_real_us = 0;
const struct device *i2c_dev;
static bool device_init_correctly = false;

/**
 * @brief      Setup I2C config and accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_inertial_init(void)
{
    uint8_t wai;

    i2c_dev = device_get_binding(I2C_DEV);
    if (!i2c_dev)
    {
        ei_printf("No device I2C found; did initialization fail?\n");
        return false;
    }

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &i2c_dev;

    /* check chip ID */
    if(iis2dlpc_device_id_get(&dev_ctx, &wai) < 0){
        return false;
    }
    
    /* reset device */
	if (iis2dlpc_reset_set(&dev_ctx, PROPERTY_ENABLE) < 0) { //PROPERTY_ENABLE = 1 
		return false;
	}
	EiDevice.delay_ms(100);

    /* set power mode */
	if(  iis2dlpc_power_mode_set(&dev_ctx, IIS2DLPC_HIGH_PERFORMANCE) < 0){
		return false;
	}

/* set default odr and full scale for acc */
	if(iis2dlpc_data_rate_set(&dev_ctx, IIS2DLPC_XL_ODR_1k6Hz) < 0){ //CONFIG_IIS2DLPC_ODR_1600=y
		return false;
	}

	if(iis2dlpc_full_scale_set(&dev_ctx, IIS2DLPC_2g) < 0){  //CONFIG_IIS2DLPC_ACCEL_RANGE_2G=y
		return false;
	}

    ei_printf("Sensor " ACCEL_DEVICE_LABEL " init OK\n");
    device_init_correctly = true;

    return true;
}

/**
 * @brief      Get data from sensor, convert and call callback to handle
 *
 * @return     0 on success, non-zero on error
 */
int ei_inertial_read_data(void)
{
    uint8_t reg;
    int ret_val = 0;

    if(i2c_dev){
        iis2dlpc_flag_data_ready_get(&dev_ctx, &reg);

        if(reg){
            /* Read acceleration data */
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            iis2dlpc_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            acceleration_g[0] = (iis2dlpc_from_fs2_lp1_to_mg(
                                data_raw_acceleration[0]) * 9.81f) / 1000.f;
            acceleration_g[1] = (iis2dlpc_from_fs2_lp1_to_mg(
                                data_raw_acceleration[1]) * 9.81f) / 1000.f;
            acceleration_g[2] = (iis2dlpc_from_fs2_lp1_to_mg(
                                data_raw_acceleration[2]) * 9.81f) / 1000.f;

            cb_sampler((const void *)&acceleration_g[0], SIZEOF_N_AXIS_SAMPLED);

            k_usleep(sample_interval_real_us);
        }
    }
    //if there is no sensor initialized send all zeros
    else {
        acceleration_g[0] = 0.0f;
        acceleration_g[1] = 0.0f;
        acceleration_g[2] = 0.0f;
        EiDevice.delay_ms((uint32_t)(sample_interval_real_us / 1000));
        cb_sampler((const void *)&acceleration_g[0], SIZEOF_N_AXIS_SAMPLED);
        ret_val = -1;
    }

    return ret_val;
}

/**
 * @brief      Setup timing and data handle callback function
 *
 * @param[in]  callsampler         Function to handle the sampled data
 * @param[in]  sample_interval_ms  The sample interval milliseconds
 *
 * @return     true
 */
bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    if(device_init_correctly == false) {
        ei_printf("\r\nERR: Failed to get data, is your accelerometer connected?\r\n");
        EiDevice.set_state(eiStateFinished);
        return false;
    }

    cb_sampler = callsampler;

    sample_interval_real_us = int32_t(sample_interval_ms * 1000);
    sample_interval_real_us = sample_interval_real_us - (FLASH_WRITE_TIME_US + ACC_SAMPLE_TIME_US + CORRECTION_TIME_US);

    ei_printf("sample_interval_real_us = %d us\n", sample_interval_real_us);

    EiDevice.set_state(eiStateSampling);

    return true;
}

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_inertial_setup_data_sampling(void)
{

    if (ei_config_get_config()->sample_interval_ms < 0.001f) {
        ei_config_set_sample_interval(1.f/62.5f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        EiDevice.get_id_pointer(),
        // Device type (required), use the same device type for similar devices
        EiDevice.get_type_pointer(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" },
        /*{ "gyrX", "dps" }, { "gyrY", "dps" }, { "gyrZ", "dps" } */},
    };

    EiDevice.set_state(eiStateErasingFlash);
    ei_sampler_start_sampling(&payload, SIZEOF_N_AXIS_SAMPLED);
    EiDevice.set_state(eiStateIdle);

    return true;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
    uint8_t temp_buf[10] = {0};
    temp_buf[0] = reg;
    memcpy(&temp_buf[1], bufp, len);
    return i2c_write(i2c_dev, temp_buf, len+1, IIS2DLPC_ADDRESS);;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    return i2c_write_read(i2c_dev, IIS2DLPC_ADDRESS, &reg, 1, bufp, len);
}

/* Static functions -------------------------------------------------------- */
