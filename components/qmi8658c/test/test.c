#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "qmi8658c.h"

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

void QMI8658C_TEST()
{
    i2c_master_init();
    qmi8658c_init();

    float acce_x, acce_y, acce_z, temp;
    float gyro_x, gyro_y, gyro_z;
    float dqw, dqx, dqy,dqz;

    qmi8658c_get_acce(&acce_x, &acce_y, &acce_z);
    printf("加速度 x: %fm/s², y: %fm/s², z: %fm/s²\t", acce_x, acce_y, acce_z);

    qmi8658c_get_gyro(&gyro_x, &gyro_y, &gyro_z);
    printf("陀螺仪 x: %f°/s, y: %f°/s, z: %f°/s\t", gyro_x, gyro_y, gyro_z);

    qmi8658c_get_quaternion(&dqw, &dqx, &dqy, &dqz);
    printf("四元数: %f, %f, %f, %f \t", dqw, dqx, dqy, dqz);

    qmi8658c_get_temp(&temp);
    printf("温度:%f℃\t", temp);
    i2c_driver_delete(CONFIG_I2C_NUM);
}