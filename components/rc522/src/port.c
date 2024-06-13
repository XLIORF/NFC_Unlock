#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "guards.h"
#include "rc522.h"
#include "port.h"

// static const char *TAG = "rc522-port";


// rc522的SPI传输函数
esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t *buffer, uint8_t length)
{
    buffer[0] = (buffer[0] << 1) & 0x7E;

    return spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
                                                      .length = 8 * length,
                                                      .tx_buffer = buffer,
                                                  });
}

/* 按RC522要求的地址格式对SPI接受函数进行封装 */
esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t *buffer, uint8_t length, uint8_t addr)
{
    esp_err_t err = ESP_OK;

    addr = ((addr << 1) & 0x7E) | 0x80;

    if (SPI_DEVICE_HALFDUPLEX & rc522->config->spi.device_flags)
    {
        ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle,
                                              &(spi_transaction_t){
                                                  .flags = SPI_TRANS_USE_TXDATA,
                                                  .length = 8,
                                                  .tx_data[0] = addr,
                                                  .rxlength = 8 * length,
                                                  .rx_buffer = buffer,
                                              }));
    }
    else
    { // Fullduplex
        ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle,
                                              &(spi_transaction_t){
                                                  .flags = SPI_TRANS_USE_TXDATA,
                                                  .length = 8,
                                                  .tx_data[0] = addr,
                                              }));

        ERR_RET_GUARD(
            spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
                                                       .flags = 0x00,
                                                       .length = 8,
                                                       .rxlength = 8 * length,
                                                       .rx_buffer = buffer,
                                                   }));
    }

    return err;
}

inline esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t *buffer, uint8_t length)
{
    return i2c_master_write_to_device(
        rc522->config->i2c.port, RC522_I2C_ADDRESS, buffer, length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS);
}

inline esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t *buffer,
                                          uint8_t length, uint8_t addr)
{
    return i2c_master_write_read_device(
        rc522->config->i2c.port, RC522_I2C_ADDRESS, &addr, 1, buffer, length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS);
}