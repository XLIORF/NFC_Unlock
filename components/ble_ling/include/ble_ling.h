#pragma once

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

// gatt app 的ID
enum
{
    HID_APP_ID = 0x1812,
    BETTARY_APP_ID = 0x180F,
    BLEUNLOCK_APP_ID = 0x000F,
};

// GATT服务实例（gatts_profile_inst）序号，方便保存服务相关的接口，回调等参数
enum
{
    HID_GATT_INST,
    BLE_UNLOCK_INST,

    MAX_INST
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
};

typedef struct
{
    int auth_success;
    uint8_t sec_conn;
    struct gatts_profile_inst gatt_inst[MAX_INST];
    esp_ble_adv_data_t hidd_adv_data;
    esp_ble_adv_params_t hidd_adv_params;
} ble_priv_t;

// 初始化蓝牙
void ble_init();
// 获取私有变量
ble_priv_t *ble_app_get_priv();
// 注册应用自己的GATT处理函数，这个处理函数只能收到自己程序相关的gatt事件
void ble_app_registe_gatt_server(esp_gatts_cb_t cb, uint16_t id);