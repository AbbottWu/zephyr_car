#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include "storage.h"

LOG_MODULE_REGISTER(storage, CONFIG_LOG_DEFAULT_LEVEL);
#define NVS_SECTOR_SIZE flash0
struct nvs_fs fs;
struct flash_pages_info info;

extern int init_nvs(){
    const struct device *flash_dev = device_get_binding("FLASH_ESP32S3");
    flash_get_page_info_by_offs(flash_dev, fs.offset, &info);
    fs.offset = 0x250000;
    fs.sector_size = info.size;
    fs.sector_count = 3U;
    int ret = nvs_mount(&fs);
    if(ret){
        return 0;
    }else{
        LOG_ERR("NVS mount failed");
    }
    return 0;
}