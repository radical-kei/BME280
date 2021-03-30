/**
[drv_bme280]

Copyright (c) [2021] [radical-kei]

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "drv_bme280.h"

#define DRV_BME280_INIT_TRY_COUNT       (5)

#define DRV_BME280_DEV_ADDR             (0x76)
#define DRV_BME280_ID                   (0x60)

#define DRV_BME280_POWER_MODE_MASK      (0x03)
#define DRV_BME280_OSRS_MASK            (0x07)
#define DRV_BME280_FILTER_MASK          (0x07)
#define DRV_BME280_RESET_CODE           (0xB6)

#define DRV_BME280_CTRL_PRESS_POS       (2)
#define DRV_BME280_CTRL_TEMP_POS        (5)
#define DRV_BME280_CFG_FILTER_POS       (2)

#define DRV_BME280_CALIB_FIRST_LEN      (26)
#define DRV_BME280_CALIB_SECOND_LEN     (7)
#define DRV_BME280_CALIB_TOTAL_LEN      (DRV_BME280_CALIB_FIRST_LEN + DRV_BME280_CALIB_SECOND_LEN)
#define DRV_BME280_MEAS_DATA_LEN        (8)

#define DRV_BME280_CALIB_FIRST_ADDR     (0x88)
#define DRV_BME280_CALIB_SECOND_ADDR    (0xE1)
#define DRV_BME280_ID_ADDR              (0xD0)
#define DRV_BME280_RESET_ADDR           (0xE0)
#define DRV_BME280_CTRL_HUM_ADDR        (0xF2)
#define DRV_BME280_STATUS_ADDR          (0xF3)
#define DRV_BME280_CTRL_MEAS_ADDR       (0xF4)
#define DRV_BME280_CONFIG_ADDR          (0xF5)
#define DRV_BME280_PRESS_MSB_ADDR       (0xF7)
#define DRV_BME280_PRESS_LSB_ADDR       (0xF8)
#define DRV_BME280_PRESS_XLSB_ADDR      (0xF9)
#define DRV_BME280_TEMP_MSB_ADDR        (0xFA)
#define DRV_BME280_TEMP_LSB_ADDR        (0xFB)
#define DRV_BME280_TEMP_XLSB_ADDR       (0xFC)
#define DRV_BME280_HUM_MSB_ADDR         (0xFD)
#define DRV_BME280_HUM_LSB_ADDR         (0xFE)

#define DRV_BME280_MEAS_OFFSET          (1250)
#define DRV_BME280_MEAS_DUR             (2300)
#define DRV_BME280_PRES_HUM_MEAS_OFFSET (575)

#define DRV_BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

struct drv_bme280_uncomp_data
{
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
};

static int32_t drv_bme280_write_register(int fd, uint8_t reg_addr, uint8_t data)
{
    int32_t result = DRV_BME280_OK;
    uint8_t buf[2];

    buf[0] = reg_addr;
    buf[1] = data;
    if(2 != write(fd, buf, 2))
    {
        result = DRV_BME280_ERROR;
    }
    return result;
}

static int32_t drv_bme280_read_register(int fd, uint8_t reg_addr, uint8_t* data, uint32_t len)
{
    int32_t result = DRV_BME280_OK;
    uint8_t buf[1];

    buf[0] = reg_addr;
    if (1 != write(fd, buf, 1))
    {
        return DRV_BME280_ERROR;
    }

    if(len != read(fd, data, len))
    {
        result = DRV_BME280_ERROR;
    }
    return result;
}

static uint32_t drv_bme280_calc_meas_wait_time(struct drv_bme280_settings settings)
{
    uint32_t max_delay;
    uint32_t temp_osrs;
    uint32_t pres_osrs;
    uint32_t hum_osrs;
    uint32_t osrs_sett_to_act[] = { 0, 1, 2, 4, 8, 16 };

    /* Mapping osr settings to the actual osr values e.g. 0b101 -> osr X16  */
    if (settings.osrs_t <= 5)
    {
        temp_osrs = osrs_sett_to_act[settings.osrs_t];
    }
    else
    {
        temp_osrs = 16;
    }

    if (settings.osrs_p <= 5)
    {
        pres_osrs = osrs_sett_to_act[settings.osrs_p];
    }
    else
    {
        pres_osrs = 16;
    }

    if (settings.osrs_h <= 5)
    {
        hum_osrs = osrs_sett_to_act[settings.osrs_h];
    }
    else
    {
        hum_osrs = 16;
    }

    max_delay = DRV_BME280_MEAS_OFFSET
                + (DRV_BME280_MEAS_DUR * temp_osrs)
                + ((DRV_BME280_MEAS_DUR * pres_osrs) + DRV_BME280_PRES_HUM_MEAS_OFFSET)
                + ((DRV_BME280_MEAS_DUR * hum_osrs)  + DRV_BME280_PRES_HUM_MEAS_OFFSET);

    return max_delay;
}

static double compensate_temperature(const struct drv_bme280_uncomp_data *uncomp_data,
                                     struct drv_bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0;
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

static double compensate_pressure(const struct drv_bme280_uncomp_data *uncomp_data,
                                  const struct drv_bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

    /* avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

static double compensate_humidity(const struct drv_bme280_uncomp_data *uncomp_data,
                                  const struct drv_bme280_calib_data *calib_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

static int32_t drv_bme280_compensate_data(const struct drv_bme280_uncomp_data *uncomp_data,
                                          struct drv_bme280_data *comp_data,
                                          struct drv_bme280_calib_data *calib_data)
{
    if ((NULL == uncomp_data) || (NULL == comp_data) || (NULL == calib_data))
    {
        return DRV_BME280_ERROR;
    }

    /* Compensate the temperature data */
    comp_data->temperature = compensate_temperature(uncomp_data, calib_data);

    /* Compensate the pressure data */
    comp_data->pressure = compensate_pressure(uncomp_data, calib_data);

    /* Compensate the humidity data */
    comp_data->humidity = compensate_humidity(uncomp_data, calib_data);

    return DRV_BME280_OK;
}

static void drv_bme280_parse_sensor_data(const uint8_t *reg_data, struct drv_bme280_uncomp_data *uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)reg_data[6] << 8;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

static void drv_bme280_parse_calib_reg(struct drv_bme280_calib_data* calib_data, uint8_t* calib_reg)
{
    /* Store the parsed register values for calibration data */
    calib_data->dig_t1 = DRV_BME280_CONCAT_BYTES(calib_reg[1], calib_reg[0]);
    calib_data->dig_t2 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[3], calib_reg[2]);
    calib_data->dig_t3 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[5], calib_reg[4]);
    calib_data->dig_p1 = DRV_BME280_CONCAT_BYTES(calib_reg[7], calib_reg[6]);
    calib_data->dig_p2 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[9], calib_reg[8]);
    calib_data->dig_p3 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[11], calib_reg[10]);
    calib_data->dig_p4 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[13], calib_reg[12]);
    calib_data->dig_p5 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[15], calib_reg[14]);
    calib_data->dig_p6 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[17], calib_reg[16]);
    calib_data->dig_p7 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[19], calib_reg[18]);
    calib_data->dig_p8 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[21], calib_reg[20]);
    calib_data->dig_p9 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[23], calib_reg[22]);
    calib_data->dig_h1 = calib_reg[25];
    calib_data->dig_h2 = (int16_t)DRV_BME280_CONCAT_BYTES(calib_reg[27], calib_reg[26]);
    calib_data->dig_h3 = calib_reg[28];
    calib_data->dig_h4 = (int16_t)((uint16_t)calib_reg[29] << 4) | (int16_t)(calib_reg[30] & 0x0F);
    calib_data->dig_h5 = (int16_t)((uint16_t)calib_reg[31] << 4) | (int16_t)(calib_reg[30] >> 4);
    calib_data->dig_h6 = (int8_t)calib_reg[32];
}

static int32_t drv_bme280_read_calib_reg(int fd, uint8_t* calib_reg)
{
    int32_t result;

    /* Read the calibration register of first section */
    result = drv_bme280_read_register(fd,
                                      DRV_BME280_CALIB_FIRST_ADDR,
                                      &calib_reg[0],
                                      DRV_BME280_CALIB_FIRST_LEN);
    if(DRV_BME280_OK != result)
    {
        return result;
    }

    /* Read the calibration register of second section */
    result = drv_bme280_read_register(fd,
                                      DRV_BME280_CALIB_SECOND_ADDR,
                                      &calib_reg[26],
                                      DRV_BME280_CALIB_SECOND_LEN);
    if(DRV_BME280_OK != result)
    {
        return result;
    }
    return result;
}

static int32_t drv_bme280_get_calib_data(struct drv_bme280_dev* dev)
{
    int32_t result;
    uint8_t calib_reg[DRV_BME280_CALIB_TOTAL_LEN] = {0};

    /* Read the calibration register */
    result = drv_bme280_read_calib_reg(dev->fd, calib_reg);
    if(DRV_BME280_OK == result)
    {
        /* Parse the calibration data */
        drv_bme280_parse_calib_reg(&dev->calib_data, calib_reg);
    }
    return result;
}

static int32_t drv_bme280_set_power_mode(int fd, uint8_t power_mode)
{
    int32_t result;

    /* Variable to store the value read from power mode register */
    uint8_t ctrl_meas_reg_val;

    /* Read the power mode register */
    result = drv_bme280_read_register(fd, DRV_BME280_CTRL_MEAS_ADDR, &ctrl_meas_reg_val, 1);
    if(DRV_BME280_OK != result)
    {
        return result;
    }

    /* Write the power mode register */
    ctrl_meas_reg_val |= power_mode & DRV_BME280_POWER_MODE_MASK;
    result = drv_bme280_write_register(fd, DRV_BME280_CTRL_MEAS_ADDR, ctrl_meas_reg_val);

    return result;
}

static int32_t drv_bme280_set_osrs(int fd, struct drv_bme280_settings settings)
{
    int32_t result;
    uint8_t temp_and_press_osrs;

    /* Write the oversampling data of humidity */
    result = drv_bme280_write_register(fd, DRV_BME280_CTRL_HUM_ADDR, settings.osrs_h & DRV_BME280_OSRS_MASK);
    if(DRV_BME280_OK != result)
    {
        return result;
    }

    /* Write the oversampling data of temperature and pressure */
    temp_and_press_osrs = (settings.osrs_t & DRV_BME280_OSRS_MASK) << DRV_BME280_CTRL_TEMP_POS;
    temp_and_press_osrs |= (settings.osrs_p & DRV_BME280_OSRS_MASK) << DRV_BME280_CTRL_PRESS_POS;
    result = drv_bme280_write_register(fd, DRV_BME280_CTRL_MEAS_ADDR, temp_and_press_osrs);
    return result;
}

static int32_t drv_bme280_set_iir_filter(int fd, uint8_t filter)
{
    uint8_t t_filter;
    t_filter = (filter & DRV_BME280_FILTER_MASK) << DRV_BME280_CFG_FILTER_POS;
    return drv_bme280_write_register(fd, DRV_BME280_CONFIG_ADDR, t_filter);
}

static int32_t drv_bme280_get_id(int fd, uint8_t* chip_id)
{
    return drv_bme280_read_register(fd, DRV_BME280_ID_ADDR, chip_id, 1);
}

static int32_t drv_bme280_reset(int fd)
{
    return drv_bme280_write_register(fd, DRV_BME280_RESET_ADDR, DRV_BME280_RESET_CODE);
}

static void drv_bme280_device_close(struct drv_bme280_dev* dev)
{
    close(dev->fd);
    dev->fd = -1;
}

int32_t drv_bme280_get_temp_forcemode(struct drv_bme280_dev* dev, struct drv_bme280_data* comp_data)
{
    int32_t result;
    uint8_t reg_data[DRV_BME280_MEAS_DATA_LEN];
    struct drv_bme280_uncomp_data uncomp_data = { 0 };
    uint32_t meas_wait_time;

    if(DRV_BME280_ID != dev->chip_id || NULL == comp_data)
    {
        fprintf(stderr, "null pointer %d %p\n",dev->chip_id, comp_data);
        return DRV_BME280_ERROR;
    }

    result = drv_bme280_set_power_mode(dev->fd, DRV_BME280_FORCE_MODE);
    if(DRV_BME280_OK != result)
    {
        fprintf(stderr, "Failed drv_bme280_set_power_mode()\n");
        return result;
    }

    meas_wait_time = drv_bme280_calc_meas_wait_time(dev->settings);
    usleep(meas_wait_time);

    result = drv_bme280_read_register(dev->fd, DRV_BME280_PRESS_MSB_ADDR, reg_data, DRV_BME280_MEAS_DATA_LEN);
    if(DRV_BME280_OK != result)
    {
        fprintf(stderr, "Failed drv_bme280_read_register()\n");
        return result;
    }

    drv_bme280_parse_sensor_data(reg_data, &uncomp_data);

    result = drv_bme280_compensate_data(&uncomp_data, comp_data, &dev->calib_data);
    return result;
}
void drv_bme280_exit(struct drv_bme280_dev* dev)
{
    drv_bme280_device_close(dev);
}

int32_t drv_bme280_init(struct drv_bme280_dev* dev)
{
    int32_t  ret = DRV_BME280_ERROR;
    uint32_t try_count = DRV_BME280_INIT_TRY_COUNT;
    uint8_t  chip_id;
    int32_t  result;

    /* Open i2c device */
    dev->fd = open(dev->dev_path, O_RDWR);
    if (dev->fd < 0)
    {
        return DRV_BME280_ERROR;
    }

    /* Set slave address */
    if (ioctl(dev->fd, I2C_SLAVE, DRV_BME280_DEV_ADDR) < 0)
    {
        drv_bme280_device_close(dev);
        return DRV_BME280_ERROR;
    }

    while (try_count)
    {
        /* Read the chip-id of bme280 sensor */
        result = drv_bme280_get_id(dev->fd, &chip_id);
        if ((DRV_BME280_OK != result) || (DRV_BME280_ID != chip_id))
        {
            result = DRV_BME280_ERROR;
            fprintf(stderr, "Failed drv_bme280_get_id() result:%d chip_id:%u\n", result, chip_id);
            try_count--;
            continue;
        }

        /* Reset the sensor */
        result = drv_bme280_reset(dev->fd);
        if(DRV_BME280_OK != result)
        {
            fprintf(stderr, "Failed drv_bme280_reset() result:%d\n", result);
            try_count--;
            continue;
        }

        /* Wait for reset to complete */
        usleep(1000);

        /* Read the calibration data */
        result = drv_bme280_get_calib_data(dev);
        if(DRV_BME280_OK != result)
        {
            fprintf(stderr, "Failed drv_bme280_get_calib_data() result:%d\n", result);
            try_count--;
            continue;
        }

        /* Set the oversampling data */
        result = drv_bme280_set_osrs(dev->fd, dev->settings);
        if(DRV_BME280_OK != result)
        {
            fprintf(stderr, "Failed drv_bme280_set_osrs() result:%d\n", result);
            try_count--;
            continue;
        }

        /* Set the IIR filter data */
        result = drv_bme280_set_iir_filter(dev->fd, dev->settings.iir_filter);
        if(DRV_BME280_OK != result)
        {
            fprintf(stderr, "Failed drv_bme280_set_iir_filter() result:%d\n", result);
            try_count--;
            continue;
        }

        ret = DRV_BME280_OK;
        dev->chip_id = chip_id;
        break;
    }

    if(DRV_BME280_OK != ret)
    {
        drv_bme280_device_close(dev);
    }

    return ret;
}
