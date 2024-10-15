#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
int bleunlock_init();
void bleunlock_set_queue(QueueHandle_t out);