/**
[sample_drv_bme280]

Copyright (c) [2021] [radical-kei]

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <drv_bme280.h>

#define DEV_I2C "/dev/i2c-1"

int main(void)
{
    struct drv_bme280_dev dev;
    struct drv_bme280_data comp_data;

    dev.dev_path = DEV_I2C;
    dev.settings.osrs_p = DRV_BME280_OSRS_VALUE_16;
    dev.settings.osrs_t = DRV_BME280_OSRS_VALUE_2;
    dev.settings.osrs_h = DRV_BME280_OSRS_VALUE_1;
    dev.settings.iir_filter = DRV_BME280_IIR_VALUE_16;

    if (DRV_BME280_OK != drv_bme280_init(&dev))
    {
        fprintf(stderr, "Failed drv_bme280_init()\n");
        return -1;
    }

    if (DRV_BME280_OK != drv_bme280_get_temp_forcemode(&dev, &comp_data))
    {
        drv_bme280_exit(&dev);
        fprintf(stderr, "Failed drv_bme280_get_temp_forcemode()\n");
        return -1;
    }

    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", comp_data.temperature, comp_data.pressure * 0.01, comp_data.humidity);
    drv_bme280_exit(&dev);

    return 0;
}
