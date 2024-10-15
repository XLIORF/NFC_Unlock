#include "ble_ling.h"
#include "esp_log.h"
#include "hidd_le_prf_int.h"
#include <string.h>

static const char *TAG = "ble unlock app";
static QueueHandle_t unlock_out_queue = NULL;
struct prf_char_pres_fmt
{
    /// Unit (The Unit is a UUID)
    uint16_t unit;
    /// Description
    uint16_t description;
    /// Format
    uint8_t format;
    /// Exponent
    uint8_t exponent;
    /// Name space
    uint8_t name_space;
};
enum
{
    UNLOCK_IDX_SVC,

    UNLOCK_IDX_BATT_LVL_CHAR,
    UNLOCK_IDX_BATT_LVL_VAL,
    UNLOCK_IDX_BATT_LVL_NTF_CFG,
    UNLOCK_IDX_BATT_LVL_PRES_FMT,

    UNLOCK_IDX_NB,
};
#define GATT_UUID_BLEUNLOCK_SVC 0x10FB
#define GATT_UUID_UNLOCK_STATUS 0x10FC
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

static const uint16_t unlock_svc = GATT_UUID_BLEUNLOCK_SVC;
static const uint16_t unlock_status_uuid = GATT_UUID_UNLOCK_STATUS;
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
// static const uint16_t include_service_uuid = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_notify_wirte = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static uint8_t unlock_status = 0;
static const uint8_t unlock_status_ccc[2] = {0x00, 0x00};
struct prf_char_pres_fmt unlock_status_fmt = {
    .description = 0,
    .exponent = 0,
    .format = 1,
    .unit = 0,
};
static const esp_gatts_attr_db_t unlock_att_db[UNLOCK_IDX_NB] = {
    // 主要服务描述
    [UNLOCK_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t),
                         sizeof(unlock_svc), (uint8_t *)&unlock_svc}},

    // Battary level Characteristic Declaration
    [UNLOCK_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP},
                                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                   CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify_wirte}},

    // Battary level Characteristic Value
    [UNLOCK_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_16, (uint8_t *)&unlock_status_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, sizeof(uint8_t),
                                  sizeof(uint8_t), &unlock_status}},

    // Battary level Characteristic - Client Characteristic Configuration Descriptor
    [UNLOCK_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
                                      sizeof(unlock_status_ccc), (uint8_t *)unlock_status_ccc}},

    // Battary level report Characteristic Declaration
    [UNLOCK_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP},
                                      {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
                                       sizeof(struct prf_char_pres_fmt), sizeof(unlock_status_fmt),
                                       (void *)&unlock_status_fmt}},
};

void esp_gatt_unlock_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "注册服务");
        esp_ble_gatts_create_attr_tab(unlock_att_db, gatts_if, UNLOCK_IDX_NB, 0);
        break;
    }
    case ESP_GATTS_CONF_EVT: {
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        break;
    }
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(TAG, "写入时间");
        ESP_LOGI(TAG, "收到%d个字符：\n%.*s", param->write.len, param->write.len, param->write.value);
        if(unlock_out_queue)
            xQueueSend(unlock_out_queue, (void*)&param->write.value[0], 10);
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        esp_ble_gatts_start_service(param->add_attr_tab.handles[UNLOCK_IDX_SVC]);
        break;
    }
    default:
        break;
    }
}

void bleunlock_set_queue(QueueHandle_t out)
{
    unlock_out_queue = out;
}

int bleunlock_init()
{
    esp_err_t ret;
    ble_app_registe_gatt_server(esp_gatt_unlock_handler, BLE_UNLOCK_INST);
    ret = esp_ble_gatts_app_register(BLEUNLOCK_APP_ID);
    if(ret)
    {
        ESP_LOGE(TAG, "注册应用失败,%#x",ret);
        return -1;
    }
    return 0;
}