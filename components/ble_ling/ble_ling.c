#include "ble_ling.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "hid_main.h"
#include "hidd_le_prf_int.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bleunlock.h"

#define HIDD_DEVICE_NAME "Ling-A"
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

static const char *TAG = "BLE-APP";
static ble_priv_t *priv = NULL;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&priv->hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
            ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        priv->sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            switch (param->reg.app_id)
            {
            case HID_APP_ID:
                priv->gatt_inst[HID_GATT_INST].gatts_if = gatts_if;
                break;

            case BETTARY_APP_ID:
                priv->gatt_inst[HID_GATT_INST].gatts_if = gatts_if;
                break;

            case BLEUNLOCK_APP_ID:
                priv->gatt_inst[BLE_UNLOCK_INST].gatts_if = gatts_if;
                break;

            default:
                break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < MAX_INST; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == priv->gatt_inst[idx].gatts_if)
            {
                if (priv->gatt_inst[idx].gatts_cb)
                {
                    priv->gatt_inst[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_app_registe_gatt_server(esp_gatts_cb_t cb, uint16_t id)
{
    priv->gatt_inst[id].gatts_cb = cb;
    priv->gatt_inst[id].gatts_if = ESP_GATT_IF_NONE;
}

void ble_init()
{
    // 初始化NVS，ble运行时需要保存相关参数
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化私有变量
    priv = calloc(1, sizeof(ble_priv_t));
    if (priv == NULL)
    {
        ESP_LOGE(TAG, "分配内存失败");
        return;
    }
    // 配置广播参数
    uint8_t hidd_service_uuid128[] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                      0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00};
    priv->hidd_adv_data = (esp_ble_adv_data_t){
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = 0x03c0,   // HID Generic,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(hidd_service_uuid128),
        .p_service_uuid = hidd_service_uuid128,
        .flag = 0x6,
    };
    priv->hidd_adv_params = (esp_ble_adv_params_t){
        .adv_int_min = 0x20,
        .adv_int_max = 0x30,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        //.peer_addr            =
        //.peer_addr_type       =
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
        return;
    }
    /// register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "%s 注册GATTS回调函数失败\n", __func__);
        return;
    }
    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
    esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);
    esp_ble_gap_config_adv_data(&priv->hidd_adv_data);
    // esp_ble_gap_start_advertising(&hidd_adv_params);
    // 初始化其他蓝牙任务
    hid_main();
    bleunlock_init();
}

ble_priv_t *ble_app_get_priv()
{
    return priv;
}