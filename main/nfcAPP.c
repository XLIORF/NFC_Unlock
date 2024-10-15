#include "nfcAPP.h"
#include "driver/rc522_spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "picc/rc522_mifare.h"
#include "rc522.h"
#include "rc522_picc.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "NRC-APP";

// 数据输出队列
static QueueHandle_t app_tx_queue = NULL;

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
        xQueueSend(app_tx_queue, (void *)&msg, 500/portTICK_PERIOD_MS);
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
    rc522_handle_t scanner;
    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}

void nrf_app_set_tx_queue(QueueHandle_t q)
{
    app_tx_queue = q;
}