#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdlib.h>
#include "motor.h"
#include "light_sensor.h"
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
#define I2CSENSOR DT_ALIAS(lightsensor)


int main(void)
{
    for (int i = 0; i < 5; i++)
    {
        LOG_INF("Wating to ping");
        k_msleep(1000);
    }
    // sensor_init();
    const i2c_dt_spec lightsensor = I2C_DT_SPEC_GET(I2CSENSOR);
    LightSensor sensor(&lightsensor);
    int ret=-5;
    do
    {
        ret = sensor.i2c_ping();
    } while (ret==Sensor_OK);
    
    // k_msleep(100);
    for (int i = 0; i < 20000; i++)
    {
        Sensor_Status ret = sensor.digital_read_to_buffer();
        if (ret != Sensor_OK)
        {
            // k_msleep(100);
            // LOG_INF("digital read error: %d", ret);
            continue;
        }
        LOG_INF("digital data: %d", sensor.byte_buffer);
        k_msleep(100);
    }
    return 0;
}