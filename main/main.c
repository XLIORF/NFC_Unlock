#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hid_main.h"
#include "nfcAPP.h"
#include <stdio.h>
#include <string.h>
#include "ble_ling.h"
#include "bleunlock.h"

static const char *TAG = "main";

static QueueHandle_t nrf_queue;
static QueueHandle_t blecmd_queue;
// static esp_err_t i2c_master_init(void)
// {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = CONFIG_I2C_MASTER_SDA,
//         .scl_io_num = CONFIG_I2C_MASTER_SCL,
//         .sda_pullup_en = 0,
//         .scl_pullup_en = 0,
//         .master.clk_speed = 40000,
//     };

//     i2c_param_config(CONFIG_I2C_NUM, &conf);

//     return i2c_driver_install(CONFIG_I2C_NUM, conf.mode, 0, 0, 0);
// }
static QueueSetHandle_t queueSet;
typedef struct authed_uuid
{
    uint8_t buf[8][4];
    uint8_t len;
} authed_uuid_t;

authed_uuid_t authed_uuid = {
    .buf =
        {
            {0xA6, 0xA7, 0xEF, 0x9B},
        },
    .len = 1,
};

//! uid长度必须大于等于4
bool uid_is_valid(uint8_t *uid)
{
    for (int i = 0; i < authed_uuid.len; i++)
    {
        if (memcmp(authed_uuid.buf[i], uid, 4) == 0)
        {
            return 1;
        }
    }
    return 0;
}

void app_main()
{
    // TODO 设备进入低功耗模式
    ble_init();
    nrf_queue = xQueueCreate(1, sizeof(nrf_app_msg_t));
    blecmd_queue = xQueueCreate(1, sizeof(uint8_t));
    if (nrf_queue == NULL || blecmd_queue == NULL)
    {
        ESP_LOGE(TAG, "队列创建失败");
        return;
    }
    nrf_app_start();
    nrf_app_set_tx_queue(nrf_queue);
    bleunlock_set_queue(blecmd_queue);
    queueSet = xQueueCreateSet(2);
    xQueueAddToSet(nrf_queue, queueSet);
    xQueueAddToSet(blecmd_queue, queueSet);
    // i2c_master_init();
    // esp_deep_sleep_start();
    nrf_app_msg_t nrf_msg;
    uint8_t ble_cmd = 0;
    QueueSetMemberHandle_t xActivatedMember;
    while (true)
    {
        xActivatedMember = xQueueSelectFromSet(queueSet, portMAX_DELAY);
        if(xActivatedMember == nrf_queue)
        {
            xQueueReceive(xActivatedMember, (void *)&nrf_msg, portMAX_DELAY);
            if (nrf_msg.picc_id_len >= 4 && uid_is_valid(nrf_msg.picc_id))
            {
                printf("\n\033[1;32m验证成功\033[0\n");
                unlock_win_screen();
            }
            else
            {
                printf("\n\033[0;31m验证失败\033[0\n");
            }
        }
        else if(xActivatedMember == blecmd_queue)
        {
            xQueueReceive(xActivatedMember, (void *)&ble_cmd, portMAX_DELAY);
            printf("BLE命令\n");
            if(ble_cmd == 1)
                printf("进入睡眠模式\n");
                // TODO 休眠天线，休眠设备
            else 
                printf("退出休眠模式\n");
                // TODO 退出休眠，启动天线
        }
    
    }
}
