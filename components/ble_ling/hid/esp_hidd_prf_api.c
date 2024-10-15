#include "esp_hidd_prf_api.h"
#include "esp_log.h"
#include "hid_dev.h"
#include "hidd_le_prf_int.h"
#include <stdlib.h>
#include <string.h>
#include "ble_ling.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"


static const char *TAG = "HID-APP";

typedef struct 
{
    bool sec_conn;
    uint16_t hid_conn_id;
} hid_priv_t;

static hid_priv_t * priv = NULL;


// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN 8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN 1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN 5

// HID consumer control input report length
#define HID_CC_IN_RPT_LEN 2

esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks)
{
    esp_err_t hidd_status;

    if (callbacks != NULL)
    {
        hidd_le_env.hidd_cb = callbacks;
    }
    else
    {
        return ESP_FAIL;
    }

    if ((hidd_status = hidd_register_cb()) != ESP_OK)
    {
        return hidd_status;
    }

    esp_ble_gatts_app_register(BATTRAY_APP_ID);

    if ((hidd_status = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK)
    {
        return hidd_status;
    }

    return hidd_status;
}

esp_err_t esp_hidd_profile_init(void)
{
    if (hidd_le_env.enabled)
    {
        ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
        return ESP_FAIL;
    }
    // Reset the hid device target environment
    memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
    hidd_le_env.enabled = true;
    return ESP_OK;
}

esp_err_t esp_hidd_profile_deinit(void)
{
    uint16_t hidd_svc_hdl = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC];
    if (!hidd_le_env.enabled)
    {
        ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
        return ESP_OK;
    }

    if (hidd_svc_hdl != 0)
    {
        esp_ble_gatts_stop_service(hidd_svc_hdl);
        esp_ble_gatts_delete_service(hidd_svc_hdl);
    }
    else
    {
        return ESP_FAIL;
    }

    /* register the HID device profile to the BTA_GATTS module*/
    esp_ble_gatts_app_unregister(hidd_le_env.gatt_if);

    return ESP_OK;
}

uint16_t esp_hidd_get_version(void)
{
    return HIDD_VERSION;
}

void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
{
    if (num_key > HID_KEYBOARD_IN_RPT_LEN - 2)
    {
        ESP_LOGE(HID_LE_PRF_TAG, "%s(), the number key should not be more than %d", __func__, HID_KEYBOARD_IN_RPT_LEN);
        return;
    }

    uint8_t buffer[HID_KEYBOARD_IN_RPT_LEN] = {0};

    buffer[0] = special_key_mask;

    for (int i = 0; i < num_key; i++)
    {
        buffer[i + 2] = keyboard_cmd[i];
    }

    ESP_LOGD(HID_LE_PRF_TAG, "the key vaule = %d,%d,%d, %d, %d, %d,%d, %d", buffer[0], buffer[1], buffer[2], buffer[3],
             buffer[4], buffer[5], buffer[6], buffer[7]);
    hid_dev_send_report(hidd_le_env.gatt_if, conn_id, HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN,
                        buffer);
    return;
}

void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y)
{
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

    buffer[0] = mouse_button; // Buttons
    buffer[1] = mickeys_x;    // X
    buffer[2] = mickeys_y;    // Y
    buffer[3] = 0;            // Wheel
    buffer[4] = 0;            // AC Pan

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id, HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN,
                        buffer);
    return;
}
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH: {
        break;
    }
    case ESP_BAT_EVENT_REG: {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
        ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        priv->hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
        priv->sec_conn = false;
        ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        ble_priv_t *ble_priv = ble_app_get_priv();
        esp_ble_gap_start_advertising(&ble_priv->hidd_adv_params);
        // todo 重新开始广播
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
        ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
        ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
        ESP_LOG_BUFFER_HEX(TAG, param->led_write.data, param->led_write.length);
        break;
    }
    default:
        break;
    }
    return;
}  

void unlock_win_screen()
{
    // 保存密码
    uint8_t pass_pin[] = {HID_KEY_9, HID_KEY_2, HID_KEY_6, HID_KEY_4, HID_KEY_5, HID_KEY_1, 0};
    // 用于激活输入框
    uint8_t key = HID_KEY_SPACEBAR;
    esp_hidd_send_keyboard_value(priv->hid_conn_id, 0, &key, 1);
    // 等待出现输出框
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 输入密码
    for (int i = 0; i < 6; i++)
    {
        esp_hidd_send_keyboard_value(priv->hid_conn_id, 0, pass_pin + i, 1);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    key = 0;                                               // 取消按下的按键
    esp_hidd_send_keyboard_value(priv->hid_conn_id, 0, &key, 1); // 用于激活输入框
}

int hid_main()
{
    priv = calloc(1, sizeof(hid_priv_t));
    if(priv == NULL)
    {
        ESP_LOGE(TAG, "分配内存失败");
        return -1;
    }
    esp_err_t ret;
    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
    }
    esp_hidd_register_callbacks(hidd_event_callback);
    return 0;
}
