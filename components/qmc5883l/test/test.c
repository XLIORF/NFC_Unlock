#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "qmc5883l.h"
#include "esp_log.h"
#include "driver/i2c.h"

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

void QMC5883L_TEST()
{
    i2c_master_init();
    qmc5883l_init(MODE_Continuous, ODR_10Hz, RNG_2G, OSR_512);
    float angle;
    angle = qmc5883l_get_angle();
    printf("angle:%f°\n", angle);

    i2c_driver_delete(CONFIG_I2C_NUM);
}