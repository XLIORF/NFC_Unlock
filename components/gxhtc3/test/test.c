#include "gxhtc3.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "GXHTC3 TEST"

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = CONFIG_I2C_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = 40000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void GXHTC3_TEST()
{
    ESP_LOGI(TAG, "测试开始");
    i2c_master_init();
    gxhtc3_init();
    float temperature, humanity;
    while (1)
    {
        gxhtc3_measure(&temperature, &humanity);
        vTaskDelay(500);
    }
    i2c_driver_delete(CONFIG_I2C_NUM);
    ESP_LOGI(TAG, "测试成功");
}