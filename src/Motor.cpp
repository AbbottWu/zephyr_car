#include "Motor.h"
#include "MiniPID.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel/thread.h>
LOG_MODULE_REGISTER(motor, CONFIG_LOG_DEFAULT_LEVEL);

#define MOTOR0 DT_PATH(motor_drivers, motor_0)
#define MOTOR1 DT_PATH(motor_drivers, motor_1)
#define MOTOR2 DT_PATH(motor_drivers, motor_2)
#define MOTOR3 DT_PATH(motor_drivers, motor_3)

K_MSGQ_DEFINE(my_msgq, sizeof(Motor_Speed), 1, 4);

Motor::Motor(const char *_name, const pwm_dt_spec *_port_0, const pwm_dt_spec *_port_1, const gpio_dt_spec *_encoder_0, const gpio_dt_spec *_encoder_1)
{
    strcpy(name, _name);
    motor_port_0 = _port_0;
    motor_port_1 = _port_1;
    encoder_0 = _encoder_0;
    encoder_1 = _encoder_1;
    motor_period = _port_0->period;
    gpio_init_callback(&encoder_callbacks[0], encoder0_isr, 1U << encoder_0->pin);
    gpio_init_callback(&encoder_callbacks[1], encoder1_isr, 1U << encoder_1->pin);
    gpio_add_callback_dt(encoder_0, &encoder_callbacks[0]);
    gpio_add_callback_dt(encoder_0, &encoder_callbacks[1]);
    gpio_pin_interrupt_configure_dt(encoder_0, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(encoder_1, GPIO_INT_EDGE_BOTH);
}
void Motor::speed(int duty)
{
    int pulse_width = (int)(duty * motor_period / 100);
    if (duty > 0)
    {
        pwm_set_dt(motor_port_0, motor_period, motor_period-pulse_width);
        pwm_set_dt(motor_port_1, motor_period, motor_period);
    }
    else if (duty < 0)
    {
        pwm_set_dt(motor_port_0, motor_period, motor_period);
        pwm_set_dt(motor_port_1, motor_period, motor_period+pulse_width);
    }
    else
    {
        pwm_set_dt(motor_port_0, motor_period, motor_period);
        pwm_set_dt(motor_port_1, motor_period, motor_period);
    }
}
void Motor::encoder0_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins)
{
    Motor *instance = CONTAINER_OF(cb, Motor, encoder_callbacks[0]);
    int encoder1_val = gpio_pin_get_dt(instance->encoder_1);
    if (encoder1_val == 1)
    {
        instance->encoder_his[0] = 1;
        if (instance->encoder_his[1] == 1)
        {
            instance->pos -= 1;
        }
        else if (instance->encoder_his[1] == 0)
        {
            instance->pos += 1;
        }
    }
    else if (encoder1_val == 0)
    {
        instance->encoder_his[0] = 0;
        if (instance->encoder_his[1] == 1)
        {
            instance->pos += 1;
        }
        else if (instance->encoder_his[1] == 0)
        {
            instance->pos -= 1;
        }
    }
    // LOG_INF("%s encoder0_isr, POS: %ld", instance->name, instance->pos);
}
void Motor::encoder1_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins)
{
    Motor *instance = CONTAINER_OF(cb, Motor, encoder_callbacks[1]);
    int encoder0_val = gpio_pin_get_dt(instance->encoder_0);
    if (encoder0_val == 1)
    {
        instance->encoder_his[1] = 1;
        if (instance->encoder_his[0] == 1)
        {
            instance->pos += 1;
        }
        else if (instance->encoder_his[0] == 0)
        {
            instance->pos -= 1;
        }
    }
    else if (encoder0_val == 0)
    {
        instance->encoder_his[1] = 0;
        if (instance->encoder_his[0] == 1)
        {
            instance->pos -= 1;
        }
        else if (instance->encoder_his[0] == 0)
        {
            instance->pos += 1;
        }
    }
}

extern void motor_task(void *, void *, void *)
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
    Motor motors[4] = {
        Motor("motor0", &motor_0_port_0, &motor_0_port_1, &motor_0_encoder_0, &motor_0_encoder_1),
        Motor("motor1", &motor_1_port_0, &motor_1_port_1, &motor_1_encoder_0, &motor_1_encoder_1),
        Motor("motor2", &motor_2_port_0, &motor_2_port_1, &motor_2_encoder_0, &motor_2_encoder_1),
        Motor("motor3", &motor_3_port_0, &motor_3_port_1, &motor_3_encoder_0, &motor_3_encoder_1)};

    MiniPID pid[4] = {MiniPID(0.02, 0.0010, 0,1/234), MiniPID(0.02, 0.0012, 0,1/234), MiniPID(0.02, 0.0012, 0,1/234), MiniPID(0.02, 0.0012, 0,1/234)};
    for (int i = 0; i < 4; i++)
    {
        pid[i].setOutputLimits(-100, 100);
    }
    int ms_between_speed_updates = 100;
    int last_pos[4] = {0, 0, 0, 0};
    int pos[4] = {0, 0, 0, 0};
    int target_speed[4] = {0, 0, 0, 0};
    while (true)
    {
        for (int i = 0; i < 4; i++)
        {
            pos[i] = motors[i].pos;
        }
        Motor_Speed data;
        int ret = k_msgq_get(&my_msgq, &data, K_NO_WAIT);
        if (ret == 0)
        {
            // LOG_WRN("New Message Got");
            target_speed[0] = data.speed0;
            target_speed[1] = data.speed1;
            target_speed[2] = data.speed2;
            target_speed[3] = data.speed3;
        }
        int pulse_per_ms[4];
        for (int i = 0; i < 4; i++)
        {
            int delta_pos = pos[i] - last_pos[i];
            last_pos[i] = pos[i];
            int speed = (int)(delta_pos * (1000/ms_between_speed_updates));
            pulse_per_ms[i] = speed;
            double output = pid[i].getOutput(speed, target_speed[i]);
            // motors[i].speed(output);
        }
        // LOG_PRINTK("%d,%d,%d,%d\n", pulse_per_ms[0], pulse_per_ms[1], pulse_per_ms[2], pulse_per_ms[3]);
        LOG_PRINTK("%d,%d,%d,%d\n", pos[0], pos[1], pos[2], pos[3]);
        k_msleep(ms_between_speed_updates);
    }
}


void set_speed(int speed0, int speed1, int speed2, int speed3)
{
    Motor_Speed speed_msg;
    speed_msg.speed0 = speed0;
    speed_msg.speed1 = speed1;
    speed_msg.speed2 = speed2;
    speed_msg.speed3 = speed3;
    extern struct k_msgq my_msgq; 
    int ret = k_msgq_put(&my_msgq, &speed_msg, K_MSEC(200));
    // if (ret!=0)
    // {
    //     LOG_ERR("Failed to put message");
    // }else{
    //     LOG_WRN("Put New Speed %d %d %d %d", speed0, speed1, speed2, speed3);
    // }
}