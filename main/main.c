#include "ble_ling.h"
#include "bleunlock.h"
#include "driver/i2c.h"
#include "driver/rc522_spi.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hid_main.h"
#include "picc/rc522_mifare.h"
#include "rc522.h"
#include "rc522_picc.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "main";

typedef struct authed_uuid
{
    uint8_t buf[8][4];
    uint8_t len;
} authed_uuid_t;

typedef struct
{
    uint8_t status;
    uint8_t picc_id[8];
    uint8_t picc_id_len;
} nrf_app_msg_t;

typedef struct
{
    QueueHandle_t nrf_queue;
    QueueHandle_t blecmd_queue;
    QueueSetHandle_t queueSet;
    rc522_handle_t scanner;
} main_priv_t;
static main_priv_t *priv = NULL;

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

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE)
    {
        rc522_picc_print(picc);
        nrf_app_msg_t msg = {
            .picc_id_len = picc->uid.length,
        };
        memcpy(msg.picc_id, picc->uid.value, picc->uid.length);
        xQueueSend(priv->nrf_queue, (void *)&msg, 500 / portTICK_PERIOD_MS);
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE)
    {
        ESP_LOGI(TAG, "卡片已移开");
    }
}

void nrf_app_start()
{
    rc522_spi_config_t driver_config = {
        .host_id = SPI2_HOST,
        .bus_config =
            &(spi_bus_config_t){
                .miso_io_num = CONFIG_RC522_MISO,
                .mosi_io_num = CONFIG_RC522_MOSI,
                .sclk_io_num = CONFIG_RC522_SCLK,
            },
        .dev_config =
            {
                .spics_io_num = CONFIG_RC522_SDA,
            },
        .rst_io_num = CONFIG_RC522_RST,
    };
    rc522_driver_handle_t driver;
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };
    rc522_create(&scanner_config, &priv->scanner);
    rc522_register_events(priv->scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(priv->scanner);
}

void app_main()
{
    priv = calloc(1, sizeof(main_priv_t));
    if (priv == NULL)
    {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }
    priv->nrf_queue = xQueueCreate(1, sizeof(nrf_app_msg_t));
    priv->blecmd_queue = xQueueCreate(1, sizeof(uint8_t));
    if (priv->nrf_queue == NULL || priv->blecmd_queue == NULL)
    {
        ESP_LOGE(TAG, "队列创建失败");
        return;
    }
    priv->queueSet = xQueueCreateSet(2);
    xQueueAddToSet(priv->nrf_queue, priv->queueSet);
    xQueueAddToSet(priv->blecmd_queue, priv->queueSet);

    esp_pm_config_t pm_cfg = {
        .light_sleep_enable = true,
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
    };
    esp_pm_configure(&pm_cfg);
    ble_init();
    bleunlock_set_queue(priv->blecmd_queue);
    nrf_app_start();
    while (true)
    {
        QueueSetMemberHandle_t xActivatedMember = xQueueSelectFromSet(priv->queueSet, portMAX_DELAY);
        if (xActivatedMember == priv->nrf_queue)
        {
            nrf_app_msg_t nrf_msg;
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
        else if (xActivatedMember == priv->blecmd_queue)
        {
            uint8_t ble_cmd = 0;
            xQueueReceive(xActivatedMember, (void *)&ble_cmd, portMAX_DELAY);
            printf("BLE命令\n");
            if (ble_cmd == 1)
            {
                printf("进入睡眠模式\n");
                rc522_pause(priv->scanner);
            }
            else
            {
                printf("退出休眠模式\n");
                rc522_start(priv->scanner);
            }
        }
    }
}
