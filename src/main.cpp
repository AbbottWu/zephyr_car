#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/ring_buffer.h>
#include "MiniPID.h"
#include <stdlib.h>
#include "motor.h"
#include "light_sensor.h"
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
#define I2CSENSOR DT_ALIAS(lightsensor)

extern struct ring_buf sensor_ring_buf;

void controller(void *, void *, void *)
{
    set_speed(0, 0, 0, 0);
    uint8_t analogs_data[8];
    MiniPID follow_pid(20, 0, 0);
    follow_pid.setSetpoint(0);
    follow_pid.setOutputLimits(-2400, 2400);
    while (true)
    {
        if (!ring_buf_is_empty(&sensor_ring_buf))
        {
            uint32_t size = ring_buf_size_get(&sensor_ring_buf) / 8;
            
            if(size % 8 == 0)
            {
                while (size > 1)
                {
                    ring_buf_get(&sensor_ring_buf, analogs_data, 8);
                    size--;
                }
                ring_buf_get(&sensor_ring_buf, analogs_data, 8);
            }
            else
            {
                ring_buf_reset(&sensor_ring_buf);
            }
        }
        int delta = analogs_data[4] - analogs_data[3];
        LOG_PRINTK("%d,0\n", delta);
        double output = follow_pid.getOutput(delta);
        set_speed(1200 + output, 1200 - output, 1200 + output, 1200 - output);
        k_msleep(10);
    }
}

#define MOTOR_STACK_SIZE 5000
#define MOTOR_PRIORITY -3
K_THREAD_STACK_DEFINE(motor_stack_area, MOTOR_STACK_SIZE);
static k_thread motor_thread_data;

#define SENSOR_STACK_SIZE 15000
#define SENSOR_PRIORITY -1
K_THREAD_STACK_DEFINE(sensor_stack_area, SENSOR_STACK_SIZE);
static k_thread sensor_thread_data;

#define MAIN_STACK_SIZE 5000
#define MAIN_PRIORITY -2
K_THREAD_STACK_DEFINE(main_stack_area, MAIN_STACK_SIZE);
static k_thread main_thread_data;

int main(void)
{
    k_thread_create(&sensor_thread_data, sensor_stack_area,
                    K_THREAD_STACK_SIZEOF(sensor_stack_area),
                    sensor_task,
                    NULL, NULL, NULL,
                    SENSOR_PRIORITY, 0, K_NO_WAIT);
    // k_thread_create(&motor_thread_data, motor_stack_area,
    //                 K_THREAD_STACK_SIZEOF(motor_stack_area),
    //                 motor_task,
    //                 NULL, NULL, NULL,
    //                 MOTOR_PRIORITY, 0, K_MSEC(10));
    // k_thread_create(&main_thread_data, main_stack_area,
    //                 K_THREAD_STACK_SIZEOF(main_stack_area),
    //                 controller,
    //                 NULL, NULL, NULL,
    //                 MAIN_PRIORITY, 0, K_MSEC(50));
    k_sleep(K_FOREVER);
    return 0;
}