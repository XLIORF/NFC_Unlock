#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fingerprint.h"

#define TAG "AS608"

void AS608_TEST()
{
    ESP_LOGI(TAG, "\n开始测试");
    fp_init();
    fp_add();
    while (1)
    {
        uint16_t pn = -1;
        fp_err_t ret = fp_verify(&pn);
        if(ret == FP_OK)
        {
            printf("验证成功，指纹为：%d\n", pn);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "\n测试完成");
}
