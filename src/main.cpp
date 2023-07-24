#include <stdlib.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "Light_sensor.h"
#include "Motor.h"
#include "MiniPID.h"
#include "CCD_sensor.h"
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

            if (size % 8 == 0)
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

#define MOTOR0 DT_PATH(motor_drivers, motor_0)
#define MOTOR1 DT_PATH(motor_drivers, motor_1)
#define MOTOR2 DT_PATH(motor_drivers, motor_2)
#define MOTOR3 DT_PATH(motor_drivers, motor_3)

int main(void)
{
    const pwm_dt_spec motor_0_port_0 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR0, port_0);
    const pwm_dt_spec motor_0_port_1 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR0, port_1);
    const gpio_dt_spec motor_0_encoder_0 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR0, gpios, 0);
    const gpio_dt_spec motor_0_encoder_1 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR0, gpios, 1);

    const pwm_dt_spec motor_1_port_0 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR1, port_0);
    const pwm_dt_spec motor_1_port_1 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR1, port_1);
    const gpio_dt_spec motor_1_encoder_0 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR1, gpios, 0);
    const gpio_dt_spec motor_1_encoder_1 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR1, gpios, 1);

    const pwm_dt_spec motor_2_port_0 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR2, port_0);
    const pwm_dt_spec motor_2_port_1 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR2, port_1);
    const gpio_dt_spec motor_2_encoder_0 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR2, gpios, 0);
    const gpio_dt_spec motor_2_encoder_1 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR2, gpios, 1);

    const pwm_dt_spec motor_3_port_0 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR3, port_0);
    const pwm_dt_spec motor_3_port_1 =
        PWM_DT_SPEC_GET_BY_NAME(MOTOR3, port_1);
    const gpio_dt_spec motor_3_encoder_0 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR3, gpios, 0);
    const gpio_dt_spec motor_3_encoder_1 =
        GPIO_DT_SPEC_GET_BY_IDX(MOTOR3, gpios, 1);

    // k_thread_create(&sensor_thread_data, sensor_stack_area,
    //                 K_THREAD_STACK_SIZEOF(sensor_stack_area), sensor_task, NULL,
    //                 NULL, NULL, SENSOR_PRIORITY, 0, K_NO_WAIT);
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
    while (true)
    {
        printk("A encoder : %d, %d; B encoder : %d, %d; C encoder : %d, %d; D encoder : %d, %d\n", gpio_pin_get_dt(&motor_0_encoder_0), gpio_pin_get_dt(&motor_0_encoder_1), gpio_pin_get_dt(&motor_1_encoder_0), gpio_pin_get_dt(&motor_1_encoder_1), gpio_pin_get_dt(&motor_2_encoder_0), gpio_pin_get_dt(&motor_2_encoder_1), gpio_pin_get_dt(&motor_3_encoder_0), gpio_pin_get_dt(&motor_3_encoder_1));
    }
    // Motor motors[4] = {
    //     Motor("motor0", &motor_0_port_0, &motor_0_port_1, &motor_0_encoder_0, &motor_0_encoder_1),
    //     Motor("motor1", &motor_1_port_0, &motor_1_port_1, &motor_1_encoder_0, &motor_1_encoder_1),
    //     Motor("motor2", &motor_2_port_0, &motor_2_port_1, &motor_2_encoder_0, &motor_2_encoder_1),
    //     Motor("motor3", &motor_3_port_0, &motor_3_port_1, &motor_3_encoder_0, &motor_3_encoder_1)};
    // motors[2].speed(20);
    // k_sleep(K_SECONDS(3));
    // motors[2].speed(50);
    // k_sleep(K_SECONDS(3));
    // motors[2].speed(0);
    // motors[3].speed(20);
    // k_sleep(K_SECONDS(3));
    // motors[3].speed(50);
    // k_sleep(K_SECONDS(3));
    // motors[3].speed(0);
    // k_sleep(K_SECONDS(3));
    // while(true){
    //     printk("Motor0 pos: %d, Motor1 pos: %d, Motor2 pos: %d, Motor3 pos: %d\n", motors[0].pos, motors[1].pos, motors[2].pos, motors[3].pos);
    // }
    return 0;
}