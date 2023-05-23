#pragma once
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// #include <zephyr/device.h>

class Motor
{
private:
    char name[10];
    const pwm_dt_spec *motor_port_0;
    const pwm_dt_spec *motor_port_1;
    const gpio_dt_spec *encoder_0;
    const gpio_dt_spec *encoder_1;
    gpio_callback encoder_callbacks[2];
    int motor_period;
    int encoder_his[2] = {0, 0};

public:
    long pos = 0;
    Motor(const char *_name, const pwm_dt_spec *_port_0, const pwm_dt_spec *_port_1, const gpio_dt_spec *_encoder_0, const gpio_dt_spec *_encoder_1);
    void speed(int duty);
    static void encoder0_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins);
    static void encoder1_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins);
};

k_tid_t motor_task_init(void);

struct motor_speed
{
    int speed0;
    int speed1;
    int speed2;
    int speed3;
};

void set_speed(int, int, int, int);
