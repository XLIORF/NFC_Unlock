#include "qmi8658c.h"
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmi8658c_reg.h"
#include <string.h>

#define QMI8658C_ADDR 0x6A // 0x6B
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_MASTER_NUM CONFIG_I2C_NUM
#define qmi8658_delay(x) vTaskDelay(x / portTICK_PERIOD_MS)
#define qmi8658c_int2 (qmi8658_int() & 0x01)
static const char *TAG = "QMI8658C";
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static int qmi8658c_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, QMI8658C_ADDR, &reg_addr, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static uint8_t qmi8658c_register_read_byte(uint8_t reg_addr)
{
    uint8_t data;
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, QMI8658C_ADDR, &reg_addr, 1, &data, 1,
                                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取寄存器失败");
        esp_restart();
    }
    return data;
}

static int qmi8658c_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t buf[] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, QMI8658C_ADDR, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

static void clear_reg_bit(uint8_t reg, uint8_t bit)
{
    uint8_t data = 0;
    data = qmi8658c_register_read_byte(reg);
    data &= ~(1 << bit);
    qmi8658c_register_write_byte(reg, data);
}

static void set_reg_bit(uint8_t reg, uint8_t bit)
{
    uint8_t data = 0;
    data = qmi8658c_register_read_byte(reg);
    data |= (1 << bit);
    qmi8658c_register_write_byte(reg, data);
}
// *复位设备，可用
void qmi8658c_reset()
{
    qmi8658c_register_write_byte(RESET, 0xB0);
}

//*获取中断状态的函数， 应syncSimp=0
uint8_t qmi8658_int()
{
    return qmi8658c_register_read_byte(STATUSINT);
}

//! 应该是可用的
int qmi8658c_self_test()
{
    ESP_LOGI(TAG, "加速度自检...");
    clear_reg_bit(CTRL7, 0); // 关闭加速度传感器
    qmi8658_delay(1);

    // 启动自测
    qmi8658c_register_write_byte(CTRL2, 0x83);
    while (qmi8658c_int2 == 0)
    {
        qmi8658_delay(10);
    }
    // 关闭自测
    qmi8658c_register_write_byte(CTRL2, 0x03);
    while (qmi8658c_int2 == 1)
    {
        qmi8658_delay(10);
    }
    // 读出自测结果
    uint8_t data[6] = {0};
    qmi8658c_register_read(DVX_L, data, 6);

    // 处理数据
    float dVelocity[3] = {0.0};
    // esp_log_buffer_hexdump_internal(TAG, data, 6, ESP_LOG_INFO);
    dVelocity[0] = (float)(data[1] << 8 | data[0]) / 2048.0;
    dVelocity[1] = (float)(data[3] << 8 | data[2]) / 2048.0;
    dVelocity[2] = (float)(data[5] << 8 | data[4]) / 2048.0;
    // 判断可用性
    for (int i = 0; i < 3; i++)
    {
        if (dVelocity[i] < 0.2)
        {
            ESP_LOGE(TAG, "加速度自测失败，加速度计不可用");
            return -1;
        }
    }
    ESP_LOGI(TAG, "加速度自检完成");

    ESP_LOGI(TAG, "陀螺仪自检...");
    clear_reg_bit(CTRL7, 1); // 关闭陀螺仪传感器
    qmi8658_delay(1);

    // 启动自测
    qmi8658c_register_write_byte(CTRL3, 0x83);
    while (qmi8658c_int2 == 0)
    {
        qmi8658_delay(10);
    }
    // 关闭自测
    qmi8658c_register_write_byte(CTRL3, 0x03);
    while (qmi8658c_int2 == 1)
    {
        qmi8658_delay(10);
    }
    // 读出自测结果
    qmi8658c_register_read(DVX_L, data, 6);

    // 处理数据
    // esp_log_buffer_hexdump_internal(TAG, data, 6, ESP_LOG_INFO);
    dVelocity[0] = (float)(data[1] << 8 | data[0]) / 16.0;
    dVelocity[1] = (float)(data[3] << 8 | data[2]) / 16.0;
    dVelocity[2] = (float)(data[5] << 8 | data[4]) / 16.0;
    // 判断可用性

    for (int i = 0; i < 3; i++)
    {
        if (dVelocity[i] < 300)
        {
            ESP_LOGE(TAG, "陀螺仪自测失败，陀螺仪不可用");
            return -1;
        }
    }
    ESP_LOGI(TAG, "陀螺仪自检完成");
    return 0;
}

int qmi8658c_init()
{
    uint8_t id = qmi8658c_register_read_byte(WHO_AM_I);
    if (id != 0x05)
    {
        ESP_LOGE(TAG, "QMI8658C不存在");
        return -1;
    }
    uint8_t revision = qmi8658c_register_read_byte(REVISION_ID);
    ESP_LOGI(TAG, "Revision:%d", revision);
    ESP_LOGI(TAG, "初始化...");
    qmi8658c_reset();
    qmi8658_delay(1800);
    qmi8658c_self_test();
    // qmi8658_delay(1800);
    qmi8658c_register_write_byte(CTRL1, 0b01000000); // 4线SPI、地址自增、小端模式、使能晶振
    qmi8658c_register_write_byte(CTRL2, 0b00010011); // 使能加速度自测，量程±4g、odr1000hz
    qmi8658c_register_write_byte(CTRL3, 0b00010011); // 使能陀螺仪自测，量程±32dps、odr940hz
    qmi8658c_register_write_byte(
        CTRL5, 0b01110111); // 加速度低通滤波器，带宽2.66% of ODR 、使能陀螺仪低通滤波器，带宽2.66% of ODR
    // qmi8658c_register_write_byte(CTRL6, 0b10000110); // 启动
    qmi8658c_register_write_byte(
        CTRL7, 0b00000011); // 使能同步采样、高速内部时钟 、Gyroscope in Full Mode、 Disable AttitudeEngine orientation
                            // and velocity increment computation 、 Enable Gyroscope、Enable Accelerometer.

    ESP_LOGI(TAG, "初始化完成");
    return 0;
}

//! 读数无反应
void qmi8658c_get_acce(float *x, float *y, float *z)
{
    // 等待数据更新
    uint8_t status = qmi8658c_register_read_byte(STATUS0);
    while ((status & 0x01) == 0)
    {
        status = qmi8658c_register_read_byte(STATUS0);
        qmi8658_delay(5);
    }

    // 等待数据锁定
    // status = qmi8658c_register_read_byte(STATUSINT);
    // while ((status & 0x02) == 0)
    // {
    //     status = qmi8658c_register_read_byte(STATUSINT);
    //     qmi8658_delay(1);
    // }

    // 读取数据
    // qmi8658c_register_read(AX_L,)
    uint8_t accex_l = qmi8658c_register_read_byte(AX_L);
    uint8_t accex_h = qmi8658c_register_read_byte(AX_H);
    uint8_t accey_l = qmi8658c_register_read_byte(AY_L);
    uint8_t accey_h = qmi8658c_register_read_byte(AY_H);
    uint8_t accez_l = qmi8658c_register_read_byte(AZ_L);
    uint8_t accez_h = qmi8658c_register_read_byte(AZ_H);

    *x = ((accex_h << 8 | accex_l) - 128) / 8192.0 * 9.8;
    *y = ((accey_h << 8 | accey_l) - 128) / 8192.0 * 9.8;
    *z = ((accez_h << 8 | accez_l) - 128) / 8192.0 * 9.8;
}

// ! 读数有反应但是不是是否正确
void qmi8658c_get_gyro(float *x, float *y, float *z)
{
    // 等待数据更新
    uint8_t status = qmi8658c_register_read_byte(STATUS0);
    while ((status & 0x02) == 0)
    {
        status = qmi8658c_register_read_byte(STATUS0);
        qmi8658_delay(5);
    }

    // 等待数据锁定
    // status = qmi8658c_register_read_byte(STATUSINT);
    // while ((status & 0x02) == 0)
    // {
    //     status = qmi8658c_register_read_byte(STATUSINT);
    //     qmi8658_delay(1);
    // }

    uint8_t gyrox_l = qmi8658c_register_read_byte(GX_L);
    uint8_t gyrox_h = qmi8658c_register_read_byte(GX_H);
    uint8_t gyroy_l = qmi8658c_register_read_byte(GY_L);
    uint8_t gyroy_h = qmi8658c_register_read_byte(GY_H);
    uint8_t gyroz_l = qmi8658c_register_read_byte(GZ_L);
    uint8_t gyroz_h = qmi8658c_register_read_byte(GZ_H);

    *x = ((gyrox_h << 8 | gyrox_l) - 32768) / 1024.0;
    *y = ((gyroy_h << 8 | gyroy_l) - 32768) / 1024.0;
    *z = ((gyroz_h << 8 | gyroz_l) - 32768) / 1024.0;
}

//! 读数偏差较大
void qmi8658c_get_temp(float *temp)
{
    uint8_t temp_l = qmi8658c_register_read_byte(TEMP_L);
    uint8_t temp_h = qmi8658c_register_read_byte(TEMP_H);
    *temp = (float)(temp_h << 8 | temp_l) / 256.0;
}

//! 未测试
void qmi8658c_set_ODR(QMI_ODR acce_odr, QMI_ODR gyro_odr)
{
    // 关闭同步采样
    clear_reg_bit(CTRL7, 7);
    qmi8658_delay(1);

    // 去掉加速度输出锁定标志
    uint8_t data[6];
    qmi8658c_register_read(AX_L, data, 6);

    // 修改加速度的数据输出速度
    uint8_t reg = qmi8658c_register_read_byte(CTRL2);
    reg &= 0b11110000;
    reg |= acce_odr;
    qmi8658c_register_write_byte(CTRL2, reg);

    // 去掉陀螺仪输出锁定标志
    qmi8658c_register_read(GX_L, data, 6);

    // 修改陀螺仪的数据输出速度
    reg = qmi8658c_register_read_byte(CTRL3);
    reg &= 0b11110000;
    reg |= gyro_odr;
    qmi8658c_register_write_byte(CTRL3, reg);

    // 重新启动
    // qmi8658c_register_write_byte(CTRL7, 0x83);
    set_reg_bit(CTRL7, 0);
    set_reg_bit(CTRL7, 1);
    set_reg_bit(CTRL7, 7);
    set_reg_bit(CTRL7, 3);
}

// ! 未测试
// 根据ODR最高的计算，输出一个样本就加1
uint32_t qmi8658c_get_timestep()
{
    uint32_t timestep = 0;
    uint8_t time_data[3] = {0};
    qmi8658c_register_read(TIMESTAMP_L, time_data, 3);
    timestep = (uint32_t)((time_data[2] << 16) | (time_data[1] << 8) | time_data[0]);
    return timestep;
}

// ! 读数不变动
void qmi8658c_get_quaternion(float *dqw, float *dqx, float *dqy, float *dqz)
{
    // 等待数据更新
    // uint8_t status = qmi8658c_register_read_byte(STATUS0);
    // while ((status & 0x08) == 0)
    // {
    //     status = qmi8658c_register_read_byte(STATUS0);
    //     qmi8658_delay(5);
    // }
    uint8_t data[8] = {0};
    qmi8658c_register_read(DQW_L, data, 8);

    *dqw = (float)(data[1] << 8 | data[0]);
    *dqx = (float)(data[3] << 8 | data[2]);
    *dqy = (float)(data[5] << 8 | data[4]);
    *dqz = (float)(data[7] << 8 | data[6]);
}
