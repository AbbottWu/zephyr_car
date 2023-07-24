#include "CCD_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/ring_buffer.h>

#define CCDAO DT_PATH(ccd_sensor)
#define CCD_BUF_BYTES 4
static uint32_t CCD_AO_BUFFER[CCD_BUF_BYTES];
static uint16_t CCD_DATA[129];
LOG_MODULE_REGISTER(CCD, CONFIG_LOG_DEFAULT_LEVEL);

extern void ccd_task(void *, void *, void *){
    const adc_dt_spec ccdao = ADC_DT_SPEC_GET(CCDAO);
    const gpio_dt_spec si_pin = GPIO_DT_SPEC_GET_BY_IDX(CCDAO, si_gpios, 0);
    const gpio_dt_spec clk_pin = GPIO_DT_SPEC_GET_BY_IDX(CCDAO, clk_gpios, 0);
    int ret = adc_channel_setup_dt(&ccdao);
    gpio_pin_configure_dt(&si_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&clk_pin, GPIO_OUTPUT);
    if(ret == 0){
        LOG_INF("ADC channel setup success");
    }else{
        LOG_ERR("ADC channel setup failed with code %d", ret);
    }
    LOG_INF("channel_id: %d, oversampling: %d, resulution: %d,acquisition-time: %d ", ccdao.channel_cfg.channel_id, ccdao.oversampling, ccdao.resolution, ccdao.channel_cfg.acquisition_time);
    adc_sequence sequence = {
        .channels = (uint32_t)1<<ccdao.channel_cfg.channel_id,
        .buffer = (void*)CCD_AO_BUFFER,
        .buffer_size = CCD_BUF_BYTES,
        .resolution = ccdao.resolution,
        .oversampling = ccdao.oversampling,
        .calibrate = false,
    };
    gpio_pin_set_dt(&si_pin,1);
    gpio_pin_set_dt(&clk_pin,1);
    gpio_pin_set_dt(&si_pin,0);
    gpio_pin_set_dt(&clk_pin,0);
    for(int i=0;i<128;i++){
        gpio_pin_set_dt(&clk_pin,1);
        gpio_pin_set_dt(&clk_pin,0);
    }
    while(true){
        k_msleep(50);
        gpio_pin_set_dt(&si_pin,1);
        gpio_pin_set_dt(&clk_pin,1);
        gpio_pin_set_dt(&si_pin,0);
        gpio_pin_set_dt(&clk_pin,0);
        for(int i=0;i<128;i++){
            gpio_pin_set_dt(&clk_pin,1);
            k_usleep(2);
            ret = adc_read(ccdao.dev, &sequence);
            gpio_pin_set_dt(&clk_pin,0);
            if(ret==0){
                CCD_DATA[i] = CCD_AO_BUFFER[0];
            }else{
                LOG_ERR("ADC read failed with code %d", ret);
            }
        }
        // print 40-80 data in a line use printk
        for(int i=0;i<128;i++){
            printk("%2d ",(CCD_DATA[i]/64));
        }
        printk("\n");
    }
}

// class CCD_Sensor
// {
// private:
//     const adc_dt_spec* ao_pin;
//     const gpio_dt_spec* clk_pin;
//     const gpio_dt_spec* si_pin;
// public:
//     CCD_Sensor(const adc_channel_cfg *_adc_cfg_dt, const gpio_dt_spec *_clk_pin, const gpio_dt_spec *_si_pin);
//     void init();
// };

// void CCD_Sensor::init()
// {
//     adc_channel_setup(adc_dev, ch0_cfg);
// }
