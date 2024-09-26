#include <stdio.h>
#include "qmc5883l.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmc5883l_reg.h"
#include <stdio.h>  
#include <stdarg.h>  
#include "math.h"


#define QMC5883L_ADDR 0x0D
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_MASTER_NUM CONFIG_I2C_NUM
#define qmc5883l_delay(x) vTaskDelay(x / portTICK_PERIOD_MS)

static const char *TAG = "QMC5883L";
static float coefficient = 0;

static int qmc5883l_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, QMC5883L_ADDR, &reg_addr, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static uint8_t qmc5883l_register_read_byte(uint8_t reg_addr)
{
    uint8_t data;
    esp_err_t ret = 0;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, QMC5883L_ADDR, &reg_addr, 1, &data, 1,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取寄存器失败,重启");
        esp_restart();
    }
    return data;
}

static int qmc5883l_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t buf[] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, QMC5883L_ADDR, buf,  2,
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

static void clear_reg_bit(uint8_t reg, uint8_t bit)
{
    uint8_t data = 0;
    data = qmc5883l_register_read_byte(reg);
    data &= ~(1 << bit);
    qmc5883l_register_write_byte(reg, data);
}

static void set_reg_bit(uint8_t reg, uint8_t bit)
{
    uint8_t data = 0;
    data = qmc5883l_register_read_byte(reg);
    data |= (1 << bit);
    qmc5883l_register_write_byte(reg, data);
}

void qmc5883l_reset()
{
    set_reg_bit(CTRL_REG2, 7);
    qmc5883l_delay(10);
}

void qmc5883l_pointer_rollOver(uint8_t en)
{
    if(en)
        set_reg_bit(CTRL_REG2, 6);
    else
        clear_reg_bit(CTRL_REG2, 6);
}

void qmc5883l_intr(uint8_t en)
{
    if(en)
        set_reg_bit(CTRL_REG2, 0);
    else
        clear_reg_bit(CTRL_REG2, 0);
}

void qmc5883l_init(MODE_e mode, ODR_e odr, RNG_e rng, OSR_e osr)
{
    uint8_t id = 0;
    id = qmc5883l_register_read_byte(CHIP_ID);
    if(id != 0xff)
    {
        printf("QMC5883L不存在\n");
    }
    qmc5883l_reset();
    qmc5883l_register_write_byte(PERIOD_REG, 0x01);//防止芯片被磁化
    qmc5883l_register_write_byte(CTRL_REG1, osr << 6 | rng << 4 | odr << 2 | mode);
    coefficient = (rng == RNG_2G ? 12000 : 3000);
}

void qmc5883l_get_data(float *x, float *y, float *z)
{   
    uint8_t data[6] = {0};
    uint8_t status = qmc5883l_register_read_byte(STATUS_REG);
    while (!(status&0x01))
    {
        status = qmc5883l_register_read_byte(STATUS_REG);
        qmc5883l_delay(5);
    }
    
    qmc5883l_register_read(0x00, data, 6);
    *x = ((float)(((uint16_t)data[1]) << 8| data[0]) -32768 ) / coefficient;
    *y = ((float)(((uint16_t)data[3]) << 8| data[2]) -32768 ) / coefficient;
    *z = ((float)(((uint16_t)data[5]) << 8| data[4]) -32768 ) / coefficient;
}

float qmc5883l_get_angle()
{
    float angle = 0, x, y, z;
    qmc5883l_get_data(&x, &y, &z);
    angle = atan2(y, x) * 57.3 + 180;
    return angle;
}