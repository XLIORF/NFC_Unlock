/*
 * @Author: 酒鸢 terraprism@163.com
 * @Date: 2024-06-22 22:13:32
 * @LastEditors: 酒鸢 terraprism@163.com
 * @LastEditTime: 2024-06-25 18:07:47
 * @FilePath: /fingerprint/components/fingerprint/fingerprint.c
 * @Description: 通用指纹驱动库，屏蔽不同指纹识别芯片、库等的差异
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_as608_basic.h"
#include "driver_as608.h"
// #include <stdint.h>
#include "esp_log.h"
#include "fingerprint.h"

static const char *TAG = "AS608";

void fingerprint_input_cb(int8_t status, const char *const fmt, ...)
{
    switch (status)
    {
    case 0:
        ESP_LOGI("指纹录入", "请按压指纹");
        break;
    case 1:
        ESP_LOGI("指纹录入", "请再次按压指纹");
        break;
    
    case 2:
        ESP_LOGI("指纹录入", "指纹录入成功");
        break;

    case -1:
        ESP_LOGE("指纹录入", "指纹录入失败");
        break;
    default:
        break;
    }
}

void fp_init(void)
{
    int ret = as608_advance_init(AS608_ADVANCE_DEFAULT_ADDRESS);
    ESP_LOGI(TAG, "初始化%s", (ret==1?"失败":"成功"));
}

fp_err_t fp_add()
{
    uint16_t score;
    uint16_t page_number;
    as608_status_t status;
    as608_advance_input_fingerprint(fingerprint_input_cb, &score, &page_number, &status);
    switch (status)
    {
    case 0:
        ESP_LOGI("指纹", "指纹录入完成, 编号: %d, 成绩：%d", page_number, score);
        break;
    case 1:
        ESP_LOGE("指纹", "指纹录入失败");
        break;
    case 2:
        ESP_LOGE("指纹", "指纹录入超时");
        break;
    default:
        break;
    }
    return FP_OK;
}

fp_err_t fp_rmove()
{
    uint16_t page_number;
    as608_status_t status;
    fp_verify(&page_number);
    as608_advance_delete_fingerprint(page_number, &status);
    if(status == 0)
        return FP_OK;
    else
        return FP_FAIL;
}

fp_err_t fp_verify(uint16_t *page_number)
{
    uint16_t score;
    as608_status_t status;
    uint16_t very_page_number;
    as608_advance_verify(&very_page_number, &score, &status);
    if(score > 200)
    {
        ESP_LOGI("指纹", "找到指纹：%d, 成绩:%d", very_page_number, score);
        if(page_number != NULL)
        {
            *page_number = very_page_number;
        }
        return FP_OK;
    }
    else
        return FP_VERIFY_FAIL;
}

fp_err_t fp_remove_all()
{
    as608_status_t status;
    as608_advance_empty_fingerprint(&status);
    if(status == 0)
    {
        ESP_LOGD("指纹", "清空指纹数据成功");
        return FP_OK;
    }
    else
    {
        ESP_LOGE("指纹", "清空指纹失败");
        return FP_FAIL;
    }
}