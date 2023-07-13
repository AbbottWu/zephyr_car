/**
 * @file Light_sensor.h
 * @author AbbottWu (AbbottDescriber@outlook.com)
 * \~chinese @brief 感为灰度传感器I2C驱动文件
 * @version 0.1
 * @date 2023-07-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/**
 * \~chinese @brief 传感器I2C状态
 *
 */
enum Sensor_Status
{
    Sensor_OK,
    Sensor_DEVICE_INVALID,
    Sensor_IO_ERR,
    Sensor_DATA_ERROR
};

/**
 * \~chinese @brief 灰度传感器类
 *
 */
class LightSensor
{
private:
    const i2c_dt_spec *lightsensor;

public:
    /**
     * \~chinese @brief 单字节缓冲区，用于i2c_ping、digital_read_to_buffer、analogs_continous_set
     * 
     */
    uint8_t byte_buffer;
    /**
     * \~chinese @brief 一个用于描述感为8路灰度传感器的类
     *
     * \~chinese @param i2c_sensor 传感器I2C设备树绑定
     */
    LightSensor(const i2c_dt_spec *i2c_sensor) : lightsensor(i2c_sensor){};
    /**
     *\~chinese @brief 灰度传感器状态测试
     *
     * \~chinese @return Sensor_Status 灰度传感器ping状态
     */
    Sensor_Status i2c_ping();
    /**
     * \~chinese @brief 从灰度传感器内部读取8路传感器的数字参量，读取成功则数据被保存到环形缓冲区
     *
     * \~chinese @return Sensor_Status 读取状态
     */
    Sensor_Status digital_read_to_buffer();
    /**
     * \~chinese @brief 从传感器内部读取8个8位模拟量，读取成功则数据被保存到环形缓冲区
     *
     * \~chinese @return Sensor_Status 读取状态
     */
    Sensor_Status analogs_read_to_buffer();
    /**
     * \~chinese @brief 设置传感器连续模拟量读取功能
     *
     * \~chinese @return Sensor_Status 传感器状态
     */
    Sensor_Status analogs_continous_set();
    /**
     * \~chinese @brief 在连续读取功能下读取8个8位模拟量，读取成功则数据被保存到环形缓冲区
     *
     * \~chinese @return Sensor_Status 读取状态
     */
    Sensor_Status analogs_continous_read();
};

/**
 * @brief 灰度传感器线程，在线程初始化过程中绑定设备并启动循环，每10ms尝试读取一次
 *
 */
void sensor_task(void *, void *, void *);