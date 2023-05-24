#include "light_sensor.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/ring_buffer.h>

#define I2CSENSOR DT_ALIAS(lightsensor)
#define SENSOR_BUF_BYTES 80
RING_BUF_DECLARE(sensor_ring_buf, SENSOR_BUF_BYTES);



Sensor_Status LightSensor::i2c_ping()
{
    if (i2c_is_ready_dt(lightsensor) == true)
    {
        int rett = i2c_reg_read_byte_dt(lightsensor, 0xAA, &byte_buffer);
        if(rett == 0){
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
        uint32_t proc_size=8;
        uint8_t *data;

        size = ring_buf_put_claim(&sensor_ring_buf, &data, proc_size);
        if (size != proc_size)
        {
            ring_buf_put_finish(&sensor_ring_buf, 0);
            return Sensor_DATA_ERROR;
        }

        if (i2c_burst_read_dt(lightsensor, 0xB1, data, 8) == 0)
        {
            ring_buf_put_finish(&sensor_ring_buf,8);
            return Sensor_OK;
        }
        else
        {
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

// Sensor_Status analogs_continous_read(const i2c_dt_spec *i2c, uint8_t *buffer)
// {
//     if (i2c_is_ready_dt(i2c) == true)
//     {
//         if (i2c_read_dt(i2c, buffer, 8) == 0)
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

// Sensor_Status analogs_continous_set(const i2c_dt_spec *i2c, uint8_t *buffer)
// {
//     if (i2c_is_ready_dt(i2c) == true)
//     {
//         buffer[0] = 0xB0;
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
