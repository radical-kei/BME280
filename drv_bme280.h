/**
[drv_bme280]

Copyright (c) [2021] [radical-kei]

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#ifndef __DRV_BME280_H__
#define __DRV_BME280_H__

#include <stdint.h>

#define DRV_BME280_OK               (0)
#define DRV_BME280_ERROR            (-1)

#define DRV_BME280_OSRS_VALUE_SKIP  (0U)
#define DRV_BME280_OSRS_VALUE_1     (1U)
#define DRV_BME280_OSRS_VALUE_2     (2U)
#define DRV_BME280_OSRS_VALUE_4     (3U)
#define DRV_BME280_OSRS_VALUE_8     (4U)
#define DRV_BME280_OSRS_VALUE_16    (5U)

#define DRV_BME280_IIR_VALUE_OFF    (0U)
#define DRV_BME280_IIR_VALUE_2      (1U)
#define DRV_BME280_IIR_VALUE_4      (2U)
#define DRV_BME280_IIR_VALUE_8      (3U)
#define DRV_BME280_IIR_VALUE_16     (4U)

#define DRV_BME280_SLEEP_MODE       (0U)
#define DRV_BME280_FORCE_MODE       (1U)
#define DRV_BME280_NORMAL_MODE      (3U)

struct drv_bme280_calib_data
{
    uint16_t dig_t1;
    int16_t  dig_t2;
    int16_t  dig_t3;
    uint16_t dig_p1;
    int16_t  dig_p2;
    int16_t  dig_p3;
    int16_t  dig_p4;
    int16_t  dig_p5;
    int16_t  dig_p6;
    int16_t  dig_p7;
    int16_t  dig_p8;
    int16_t  dig_p9;
    uint8_t  dig_h1;
    int16_t  dig_h2;
    uint8_t  dig_h3;
    int16_t  dig_h4;
    int16_t  dig_h5;
    int8_t   dig_h6;
    int32_t  t_fine;
};

struct drv_bme280_settings
{
    uint8_t osrs_t;
    uint8_t osrs_p;
    uint8_t osrs_h;
    uint8_t iir_filter;
};

struct drv_bme280_dev
{
    char* dev_path;
    struct drv_bme280_settings settings;

    int fd;
    uint8_t chip_id;
    struct drv_bme280_calib_data calib_data;
};

struct drv_bme280_data
{
    double pressure;
    double temperature;
    double humidity;
};

extern int32_t drv_bme280_init(struct drv_bme280_dev* dev);
extern void drv_bme280_exit(struct drv_bme280_dev* dev);
extern int32_t drv_bme280_get_temp_forcemode(struct drv_bme280_dev* dev, struct drv_bme280_data* comp_data);

#endif /* __DRV_BME280_H__ */