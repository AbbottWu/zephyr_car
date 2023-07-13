/**
 * @file Motor.h
 * @author AbbottWu (AbbottDescriber@outlook.com)
 * \~chinese @brief 用于描述内置编码器的减速电机的代码
 * @version 0.1
 * @date 2023-07-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * \~chinese @brief 一个用于描述内置编码器的减速电机的类
 *
 */
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
    static void encoder0_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins);
    static void encoder1_isr(const struct device *dev, gpio_callback *cb, gpio_port_pins_t pins);

public:
    /**
     *  \~chinese @brief 电机角度
     *
     */
    long pos = 0;
    /**
     * \~chinese @brief 电机类的构造函数，绑定电机控制接口（PWM）和电机编码器接口（GPIO）
     *
     * @param _name 电机名称
     * @param _port_0 电机接口0，类型为zephyr的PWM设备树绑定
     * @param _port_1 电机接口1，类型为zephyr的PWM设备树绑定
     * @param _encoder_0 电机编码器0，为zephyr的GPIO设备树绑定
     * @param _encoder_1 电机编码器1，为zephyr的GPIO设备树绑定
     */
    Motor(const char *_name, const pwm_dt_spec *_port_0, const pwm_dt_spec *_port_1, const gpio_dt_spec *_encoder_0, const gpio_dt_spec *_encoder_1);
    /**
     * \~chinese @brief 利用占空比设置电机速度和方向
     *
     * @param duty 电机占空比，0-100，正负代表电机转速
     */
    void speed(int duty);
};

/**
 * \~chinese @brief 调控电机转速的线程，初始化电机对象，并每10ms一次查阅接收消息队列my_msgq更新速度，如无新速度则保持。
 *
 */
void motor_task(void *, void *, void *);

/**
 * @brief 电机转速设定所用结构体，在消息队列中被传递
 *
 */
struct Motor_Speed
{
    /**
     * @brief 电机0占空比（正负表方向）
     *
     */
    int speed0;
    /**
     * @brief 电机1占空比（正负表方向）
     *
     */
    int speed1;
    /**
     * @brief 电机2占空比（正负表方向）
     *
     */
    int speed2;
    /**
     * @brief 电机3占空比（正负表方向）
     *
     */
    int speed3;
};

/**
 * \~chinese @brief 通过消息队列my_msgq更新小车四个电机的速度。
 *
 */
void set_speed(int, int, int, int);
