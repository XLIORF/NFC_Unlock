#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nfcAPP.h"
#include "bleAPP.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = 40000,
    };

    i2c_param_config(CONFIG_I2C_NUM, &conf);

    return i2c_driver_install(CONFIG_I2C_NUM, conf.mode, 0, 0, 0);
}

void app_main()
{
    ble_init();
    nrf_app_start();
    // i2c_master_init();
    // esp_deep_sleep_start();
}
