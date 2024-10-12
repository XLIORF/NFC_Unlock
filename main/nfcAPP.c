#include "driver/rc522_spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "picc/rc522_mifare.h"
#include "rc522.h"
#include "rc522_picc.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RC522_SPI_BUS_GPIO_MISO (GPIO_NUM_10)
#define RC522_SPI_BUS_GPIO_MOSI (GPIO_NUM_5)
#define RC522_SPI_BUS_GPIO_SCLK (GPIO_NUM_6)
#define RC522_SPI_SCANNER_GPIO_SDA (GPIO_NUM_2)
#define RC522_SCANNER_GPIO_RST (-1) // soft-reset

static const char *TAG = "NRC-APP";
static rc522_handle_t scanner;

static rc522_spi_config_t driver_config = {
    .host_id = SPI2_HOST,
    .bus_config =
        &(spi_bus_config_t){
            .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
            .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
            .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
        },
    .dev_config =
        {
            .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
        },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

typedef struct authed_uuid
{
    uint8_t buf[8][4];
    uint8_t len;
} authed_uuid_t;

authed_uuid_t authed_uuid = {
    .buf = {{0xA6, 0xA7, 0xEF, 0x9B}},
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

// static rc522_mifare_key_t test_key = {
//     .type = RC522_MIFARE_KEY_A,
//     .value = {0},
// };

// int brute_force_key_cracking(rc522_handle_t scanner, rc522_picc_t *picc, uint8_t block, uint64_t *key, uint64_t key_end)
// {
//     esp_err_t ret;

//     for (uint64_t counter = *key; counter < key_end; counter++)
//     {
//         test_key.value[0] = counter & 0xff;
//         test_key.value[1] = (counter >> 8) & 0xff;
//         test_key.value[2] = (counter >> 16) & 0xff;
//         test_key.value[3] = (counter >> 24) & 0xff;
//         test_key.value[4] = (counter >> 32) & 0xff;
//         test_key.value[5] = (counter >> 40) & 0xff;
//         ret = rc522_mifare_auth(scanner, picc, block, &test_key);
//         if (ret == ESP_OK)
//             return 0; // 破解成功
//         vTaskDelay(1);
//         printf("%#2x, %#2x, %#2x, %#2x, %#2x, %#2x\n", test_key.value[0], test_key.value[1], test_key.value[2],
//                test_key.value[3], test_key.value[4], test_key.value[5]);
//         if (ulTaskNotifyTake(pdTRUE, 0))
//         {
//             printf("收到信号，破解任务退出\n");
//             *key = counter;
//             return 1; // 破解失败
//         };
//     }
//     printf("循环结束，破解任务退出\n");
//     return 1; //破解失败
// }
// static uint64_t key = 0;
// void key_task(void *arg)
// {
//     printf("开始破解密码\n");
//     uint8_t ret = brute_force_key_cracking(scanner, (rc522_picc_t *)arg, 4, &key, 281474976710656ULL);
//     if (ret == 0)
//     {
//         printf("破解成功，密码：\n");
//         printf("%#2x, %#2x, %#2x, %#2x, %#2x, %#2x\n", test_key.value[0], test_key.value[1], test_key.value[2],
//                test_key.value[3], test_key.value[4], test_key.value[5]);
//     }
//     vTaskDelete(NULL);
// }

// static uint8_t id[6] = {0};
// static TaskHandle_t keytaskHandle = NULL;
static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    extern int auth_success;
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE)
    {
        rc522_picc_print(picc);
        if (picc->uid.length == 4 && uid_is_valid(picc->uid.value))
        {
            auth_success = 1;
            printf("\n\033[1;32m验证成功\033[0\n");
        }
        else
        {
            printf("\n\033[0;31m验证失败\033[0\n");
        }
        // if (id[0] == 0)
        // {
        //     memcpy(id, picc->uid.value, picc->uid.length);
        //     xTaskCreate(key_task, "key_task", 40960, (void *)picc, 15, &keytaskHandle);
        // }
        // else if (id[0] == picc->uid.value[0])
        // {
        //     // xTaskCreate(key_task, "key_task", 40960, (void *)picc, 15, &keytaskHandle);
        //     vTaskResume(keytaskHandle);
        // }
        // else
        // {
        //     // xTaskNotifyGive(keytaskHandle);
        //     vTaskSuspend(keytaskHandle);
        //     printf("请不要中途换卡\n");
        // }
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE)
    {
        ESP_LOGI(TAG, "Card has been removed");
    }
}

void nrf_app_start()
{
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };
    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}