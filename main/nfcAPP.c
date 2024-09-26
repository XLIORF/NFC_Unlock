#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include <inttypes.h>
#include "rc522.h"
#include "freertos/semphr.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

#define RC522_SPI_BUS_GPIO_MISO    (GPIO_NUM_10)
#define RC522_SPI_BUS_GPIO_MOSI    (GPIO_NUM_5)
#define RC522_SPI_BUS_GPIO_SCLK    (GPIO_NUM_6)
#define RC522_SPI_SCANNER_GPIO_SDA (GPIO_NUM_2)
#define RC522_SCANNER_GPIO_RST     (-1) // soft-reset

static const char *TAG = "NRC-APP";
static rc522_handle_t scanner;

static rc522_spi_config_t driver_config = {
    .host_id = SPI2_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
    },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

typedef struct authed_uuid {
    uint8_t buf[8][4];
    uint8_t len;
} authed_uuid_t;

authed_uuid_t authed_uuid = {
    .buf = {
        {0xA6, 0xA7, 0xEF, 0x9B}
        },
    .len = 1,
};

//! uid长度必须大于等于4
bool uid_is_valid(uint8_t *uid)
{
    for (int i = 0; i < authed_uuid.len; i++)
    {
        if(memcmp(authed_uuid.buf[i], uid, 4) == 0)
        {
            return 1;
        }
    }
    return 0;
}

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    extern int auth_success;
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        rc522_picc_print(picc);
        if(picc->uid.length == 4 && uid_is_valid(picc->uid.value))
        {
            auth_success = 1;
            printf("\n\033[1;32m验证成功\033[0\n");
        }
        else
        {
            printf("\n\033[0;31m验证失败\033[0\n");
        }
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
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