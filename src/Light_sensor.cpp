#include "Light_sensor.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/ring_buffer.h>

#define I2CSENSOR DT_ALIAS(lightsensor)
#define LIGHTSENSOR_BUF_BYTES 80
RING_BUF_DECLARE(sensor_ring_buf, LIGHTSENSOR_BUF_BYTES);
LOG_MODULE_REGISTER(sensor, CONFIG_LOG_DEFAULT_LEVEL);

Sensor_Status LightSensor::i2c_ping()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        int rett = i2c_reg_read_byte_dt(lightsensor, 0xAA, &byte_buffer);
        if (rett == 0)
        {
            if (byte_buffer == 0x66)
            {
                return Sensor_OK;
            }
            return Sensor_DATA_ERROR;
        }
        else
        {
            return Sensor_IO_ERR;
        }
    }
    return Sensor_DEVICE_INVALID;
}

Sensor_Status LightSensor::digital_read_to_buffer()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        if (i2c_reg_read_byte_dt(lightsensor, 0xDD, &byte_buffer) == 0)
        {
            return Sensor_OK;
        }
        else
        {
            return Sensor_IO_ERR;
        }
    }
    return Sensor_DEVICE_INVALID;
}

Sensor_Status LightSensor::analogs_read_to_buffer()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        uint32_t size;
        uint32_t proc_size = 8;
        uint8_t *data;

        size = ring_buf_put_claim(&sensor_ring_buf, &data, proc_size);
        if (size != proc_size)
        {
            ring_buf_put_finish(&sensor_ring_buf, 0);
            return Sensor_DATA_ERROR;
        }

        if (i2c_burst_read_dt(lightsensor, 0xB1, data, 8) == 0)
        {
            ring_buf_put_finish(&sensor_ring_buf, 8);
            return Sensor_OK;
        }
        else
        {
            ring_buf_put_finish(&sensor_ring_buf, 0);
            return Sensor_IO_ERR;
        }
    }
    return Sensor_DEVICE_INVALID;
}

// Sensor_Status digital_continous_set(const i2c_dt_spec *i2c, uint8_t *buffer)
// {
//     if (i2c_is_ready_dt(i2c) == true)
//     {
//         buffer[0] = 0xDD;
//         if (i2c_write_dt(i2c, buffer, 1) == 0)
//         {
//             return Sensor_OK;
//         }
//         else
//         {
//             return Sensor_IO_ERR;
//         }
//     }
//     return Sensor_DEVICE_INVALID;
// }

// Sensor_Status digital_continous_read(const i2c_dt_spec *i2c, uint8_t *buffer)
// {
//     if (i2c_is_ready_dt(i2c) == true)
//     {
//         if (i2c_read_dt(i2c, buffer, 1) == 0)
//         {
//             return Sensor_OK;
//         }
//         else
//         {
//             return Sensor_IO_ERR;
//         }
//     }
//     return Sensor_DEVICE_INVALID;
// }

Sensor_Status LightSensor::analogs_continous_set()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        int ret = -5;
        byte_buffer = 0xFF;
        do
        {
            ret = i2c_reg_write_byte_dt(lightsensor, 0xCE, byte_buffer);
            // LOG_INF("ret = %d", ret);
            k_usleep(50);
        } while (ret == -5);
        byte_buffer = 0xB0;
        ret = -5;
        do
        {
            ret = i2c_write_dt(lightsensor, &byte_buffer, 1);
        } while (ret == -5);
        return Sensor_OK;
    }
    return Sensor_DEVICE_INVALID;
}

Sensor_Status LightSensor::analogs_continous_read()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        uint32_t size;
        uint32_t proc_size = 8;
        uint8_t *data;

        size = ring_buf_put_claim(&sensor_ring_buf, &data, proc_size);
        if (size != proc_size)
        {
            ring_buf_put_finish(&sensor_ring_buf, 0);
            return Sensor_DATA_ERROR;
        }

        if (i2c_read_dt(lightsensor, data, 8) == 0)
        {
            int valid = 0;
            for (int i = 0; i < 8; i++)
            {
                if (data[i] != 0xFF)
                {
                    valid = 1;
                    break;
                }
            }
            if (valid == 1)
            {
                ring_buf_put_finish(&sensor_ring_buf, 8);
            }
            else
            {
                ring_buf_put_finish(&sensor_ring_buf, 0);
                return Sensor_DATA_ERROR;
            }
            // check if the data is all 255, which is invalid
            return Sensor_OK;
        }
        else
        {
            ring_buf_put_finish(&sensor_ring_buf, 0);
            return Sensor_IO_ERR;
        }
    }
    return Sensor_DEVICE_INVALID;
}

extern void sensor_task(void *, void *, void *)
{
    const i2c_dt_spec lightsensor = I2C_DT_SPEC_GET(I2CSENSOR);
    LightSensor sensor(&lightsensor);
    // make a ping tile ok
    // int ret = -5;
    // do
    // {
    //     ret = sensor.i2c_ping();
    // } while (ret != Sensor_OK);
    sensor.analogs_continous_set();
    LOG_INF("sensor set analogs ok");
    int failed_times = 0;
    while (true)
    {
        failed_times=0;
        while (true){
            Sensor_Status ret = sensor.analogs_continous_read();
            if (ret != Sensor_OK)
            {
                failed_times++;
                if (failed_times > 500){
                    // sensor.analogs_continous_set();
                    failed_times = 0;
                    k_yield();
                }
            }else{
                break;
                LOG_INF("sensor read analogs ok");
            }
        }
        k_msleep(10);
    }
}




// void sensor_init(void)
// {
//     int ret = -5;
//     do
//     {
//         // k_tid_t timeout_thread_handle = k_thread_create(&timeout_thread_data, timeout_stack_area,
//         //                                           K_THREAD_STACK_SIZEOF(timeout_stack_area),
//         //                                           i2c_ping,
//         //                                           &ret, NULL, NULL,
//         //                                           TIMEOUT_PRIORITY, 0, K_NO_WAIT);
//         // if(k_thread_join(&timeout_thread_data,K_MSEC(15))==-EBUSY){
//         //     k_thread_abort(timeout_thread_handle);
//         //     LOG_ERR("I2C PING TIMEOUT");
//         //     continue;
//         // }
//         i2c_ping(&ret);
//         switch (ret)
//         {
//         case Sensor_OK:
//             LOG_INF("I2C PING OK");
//             break;
//         case Sensor_DEVICE_INVALID:
//             LOG_ERR("I2C DEVICE INVALID");
//             break;
//         case Sensor_IO_ERR:
//             LOG_ERR("I2C IO ERROR");
//             break;
//         case Sensor_DATA_ERROR:
//             LOG_ERR("I2C DATA ERROR");
//             break;
//         }
//     } while (ret != Sensor_OK);
// }
