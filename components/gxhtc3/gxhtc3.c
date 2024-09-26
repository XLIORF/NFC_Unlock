#include <stdio.h>
#include "gxhtc3.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gxhtc3_cmd.h"

#define GXHTC3_ADDR 0x70
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_MASTER_NUM CONFIG_I2C_NUM
#define gxhtc_delay(x) vTaskDelay(x / portTICK_PERIOD_MS)
static const char *TAG = "GXHTC3";

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static int gxhtc3_register_read(uint8_t *reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, GXHTC3_ADDR, reg_addr, 2, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static int gxhtc3_register_write_cmd(uint8_t *reg_addr)
{
    int ret;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, GXHTC3_ADDR, reg_addr, 2,
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

int gxhtc3_init()
{
    // 复位
    gxhtc3_register_write_cmd(gxhtc3_cmd_software_reset);
    gxhtc_delay(15);
    uint8_t id[2] = {0};
    gxhtc3_register_read(gxhtc3_cmd_read_id, id, 2);
    if (id[0] == 0 && id[1] == 0)
    {
        ESP_LOGE(TAG, "读取GXHTC3的ID失败，硬件不存在.");
        return -1; //
    }
    return 0;
}

void gxhtc3_reset()
{
    gxhtc3_register_write_cmd(gxhtc3_cmd_software_reset);
    gxhtc_delay(1);
}

void gxhtc3_sleep()
{
    gxhtc3_register_write_cmd(gxhtc3_cmd_sleep);
}

void gxhtc3_wakeup()
{
    gxhtc3_register_write_cmd(gxhtc3_cmd_wakeup);
    gxhtc_delay(10);
}

uint8_t crc8_custom(uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;        // 初始值
    uint8_t polynomial = 0x31; // 生成多项式（去掉隐含的最高位）

    while (len--)
    {
        crc ^= *data++; // 与当前字节进行异或

        for (int i = 0; i < 8; i++)
        { // 处理CRC的每一位
            if (crc & 0x80)
            {                                  // 如果最高位为1
                crc = (crc << 1) ^ polynomial; // 左移并异或多项式
            }
            else
            {
                crc = crc << 1; // 否则只左移
            }
            crc &= 0xFF; // 确保CRC值保持为8位（这步在大多数情况下是多余的，因为crc已经是uint8_t类型）
        }
    }

    return crc;
}

int gxhtc3_measure(float *temperature, float *humanity)
{
    // 唤醒设备
    gxhtc3_wakeup();

    // 测量
    gxhtc3_register_write_cmd(gxhtc3_cmd_measure_normal_th);
    gxhtc_delay(15);

    // 读取结果
    uint8_t data[6] = {0};
    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, GXHTC3_ADDR, data, 6, portMAX_DELAY);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取温湿度失败");
        return -1;
    }

    // crc校验
    uint8_t crc = crc8_custom(data, 2);
    if (crc != data[2])
    {
        ESP_LOGW(TAG, "CRC校验失败");
        return -2;
    }
    crc = crc8_custom(data + 3, 2);
    if (crc != data[5])
    {
        ESP_LOGW(TAG, "CRC校验失败");
        return -2;
    }

    // 数据转换
    uint16_t raw_temperature = ((uint16_t)data[0]) << 8 | data[1];
    uint16_t raw_humanity = ((uint16_t)data[3]) << 8 | data[4];
    *temperature = raw_temperature * 0.00267 - 45;
    *humanity = raw_humanity * 0.00153;

    // 休眠设备
    gxhtc3_register_write_cmd(gxhtc3_cmd_sleep);
    return 0;
}