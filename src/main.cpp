#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdlib.h>
#include "motor.h"
#include <exception>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define I2CSENSOR DT_ALIAS(lightsensor)

// RING_BUF_DECLARE(sensor_buf, 100);

enum Sensor_Status
{
    Sensor_OK,
    Sensor_DEVICE_INVALID,
    Sensor_IO_ERR,
    Sensor_DATA_ERROR
};

void info_log_wait(const char *str)
{
    LOG_INF("%s", str);
    k_msleep(1000);
}

void i2c_ping(int* ret)
{
    const i2c_dt_spec i2c = I2C_DT_SPEC_GET(I2CSENSOR);
    uint8_t buffer;
    if (i2c_is_ready_dt(&i2c) == true)
    {
        LOG_INF("buffer width: %d", sizeof(buffer));
        info_log_wait("device Ready");
        try
        {
            buffer = 0xAA;
            if (i2c_write_dt(&i2c,&buffer, 1) == 0)
            {
                info_log_wait("write ok");
                if (i2c_read_dt(&i2c, &buffer, 1) == 0)
                {
                    info_log_wait("read ok");
                    if (buffer == 0xAA)
                    {
                        info_log_wait("data ok");
                        *((int*)ret) = Sensor_OK;
                    }
                    else
                    {
                        info_log_wait("data error");
                        *((int*)ret) = Sensor_DATA_ERROR;
                    }
                }
                else
                {
                    info_log_wait("read error");
                    *((int*)ret) = Sensor_IO_ERR;
                }
            }
            else
            {
                info_log_wait("write error");
                *((int*)ret) = Sensor_IO_ERR;
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERR("Exception: %s", e.what());
        }
    }
}

Sensor_Status digital_read_to_buffer(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        if (i2c_reg_read_byte_dt(i2c, 0xDD, buffer) == 0)
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

Sensor_Status analogs_read_to_buffer(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        if (i2c_burst_read_dt(i2c, 0xB1, buffer, 8) == 0)
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

Sensor_Status digital_continous_set(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        buffer[0] = 0xDD;
        if (i2c_write_dt(i2c, buffer, 1) == 0)
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

Sensor_Status digital_continous_read(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        if (i2c_read_dt(i2c, buffer, 1) == 0)
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

Sensor_Status analogs_continous_read(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        if (i2c_read_dt(i2c, buffer, 8) == 0)
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

Sensor_Status analogs_continous_set(const i2c_dt_spec *i2c, uint8_t *buffer)
{
    if (i2c_is_ready_dt(i2c) == true)
    {
        buffer[0] = 0xB0;
        if (i2c_write_dt(i2c, buffer, 1) == 0)
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

#define TIMEOUT_STACK_SIZE 5000
#define TIMEOUT_PRIORITY 6
// K_THREAD_STACK_DEFINE(timeout_stack_area, TIMEOUT_STACK_SIZE);
// static k_thread timeout_thread_data;

void sensor_init(void)
{
    int ret = -5;
    do
    {
        // k_tid_t timeout_thread_handle = k_thread_create(&timeout_thread_data, timeout_stack_area,
        //                                           K_THREAD_STACK_SIZEOF(timeout_stack_area),
        //                                           i2c_ping,
        //                                           &ret, NULL, NULL,
        //                                           TIMEOUT_PRIORITY, 0, K_NO_WAIT);
        // if(k_thread_join(&timeout_thread_data,K_MSEC(15))==-EBUSY){
        //     k_thread_abort(timeout_thread_handle);
        //     LOG_ERR("I2C PING TIMEOUT");
        //     continue;
        // }
        i2c_ping(&ret);
        switch (ret)
        {
        case Sensor_OK:
            LOG_INF("I2C PING OK");
            break;
        case Sensor_DEVICE_INVALID:
            LOG_ERR("I2C DEVICE INVALID");
            break;
        case Sensor_IO_ERR:
            LOG_ERR("I2C IO ERROR");
            break;
        case Sensor_DATA_ERROR:
            LOG_ERR("I2C DATA ERROR");
            break;
        }
        k_msleep(100);
    } while (ret != Sensor_OK);
}


int main(void)
{
    for (int i = 0; i < 5; i++)
    {
        LOG_INF("Wating to ping");
        k_msleep(1000);
    }
    sensor_init();

    for (int i = 0; i < 20000; i++)
    {
        LOG_INF("I2C OK");
        k_msleep(100);
    }
    return 0;
}