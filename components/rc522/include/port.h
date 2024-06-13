#pragma once
#include "esp_log.h"
#include "esp_err.h"
#include "rc522.h"
// #define TAG "RC522"

#define rc522_log(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define rc522_logd(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define rc522_loge(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define rc522_hexdump(pBuff, len) esp_log_buffer_hexdump_internal(TAG, pBuff, len, ESP_LOG_INFO);

esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t *buffer, uint8_t length);
esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t *buffer, uint8_t length, uint8_t addr);
esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t *buffer, uint8_t length);
esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t *buffer, uint8_t length, uint8_t addr);