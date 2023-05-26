#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

enum Sensor_Status
{
    Sensor_OK,
    Sensor_DEVICE_INVALID,
    Sensor_IO_ERR,
    Sensor_DATA_ERROR
};

class LightSensor
{
private:
    const i2c_dt_spec *lightsensor;
public:
    uint8_t byte_buffer;
    LightSensor(const i2c_dt_spec *i2c_sensor): lightsensor(i2c_sensor) {};
    Sensor_Status i2c_ping();
    Sensor_Status digital_read_to_buffer();
    Sensor_Status analogs_read_to_buffer();
    // Sensor_Status digital_continous_set();
    // Sensor_Status digital_continous_read();
    Sensor_Status analogs_continous_set();
    Sensor_Status analogs_continous_read();
};

// k_tid_t sensor_task_init(void);
void sensor_task(void*,void*,void*);