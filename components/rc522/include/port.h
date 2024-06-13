#pragma once
#include "esp_log.h"
// #define TAG "RC522"

#define rc522_log(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define rc522_logd(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define rc522_loge(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define rc522_hexdump(pBuff, len) esp_log_buffer_hexdump_internal(TAG, pBuff, len, ESP_LOG_INFO);