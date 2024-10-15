#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
typedef struct {
    uint8_t status;
    uint8_t picc_id[8];
    uint8_t picc_id_len;
} nrf_app_msg_t;


#ifdef __cplusplus
extern "C" {
#endif

void nrf_app_start();
void nrf_app_set_tx_queue(QueueHandle_t q);

#ifdef __cplusplus
}
#endif