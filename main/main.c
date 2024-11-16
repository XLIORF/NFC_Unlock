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
#include "driver/temperature_sensor.h"
#include "gxhtc3.h"

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
    temperature_sensor_handle_t temp_handle;
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

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = 40000,
    };

    i2c_param_config(CONFIG_I2C_NUM, &conf);

    return i2c_driver_install(CONFIG_I2C_NUM, conf.mode, 0, 0, 0);
}

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

void mcu_temparature_sensor_init()
{
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &priv->temp_handle));
}

float get_mcu_temperature()
{
    // 启用温度传感器
    ESP_ERROR_CHECK(temperature_sensor_enable(priv->temp_handle));
    // 获取传输的传感器数据
    float tsens_out;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(priv->temp_handle, &tsens_out));
    // printf("MCU Temperature in %f °C\n", tsens_out);
    // 温度传感器使用完毕后，禁用温度传感器，节约功耗
    ESP_ERROR_CHECK(temperature_sensor_disable(priv->temp_handle));
    return tsens_out;
}

void system_maintain(void *args)
{
    mcu_temparature_sensor_init();
    gxhtc3_init();
    while (true)
    {
        float mcu_temp = get_mcu_temperature();
        float env_temp, env_hum;
        gxhtc3_measure(&env_temp, &env_hum);
        if(env_temp > 90)
        {
            ESP_LOGE(TAG, "环境温度(%f℃)过高", env_temp);
        }
        if(env_hum > 80)
        {
            ESP_LOGE(TAG, "环境湿度(%f%%)过高", env_hum);
        }
        if(mcu_temp >= 75)
        {
            ESP_LOGW(TAG, "核心温度(%f℃)过高", mcu_temp);
        }
        if(mcu_temp >= 95)
        {
            ESP_LOGE(TAG, "核心温度(%f℃)过高，系统休眠.",mcu_temp);
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // 禁用所有唤醒源
            esp_sleep_enable_timer_wakeup(60000000); // 1min唤醒一次
            // esp_sleep_pd_config();
            esp_deep_sleep_start();
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    // 初始化主程序私有变量
    priv = calloc(1, sizeof(main_priv_t));
    if (priv == NULL)
    {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }
    // 初始化通知队列
    priv->nrf_queue = xQueueCreate(1, sizeof(nrf_app_msg_t));
    priv->blecmd_queue = xQueueCreate(1, sizeof(uint8_t));
    if (priv->nrf_queue == NULL || priv->blecmd_queue == NULL)
    {
        ESP_LOGE(TAG, "队列创建失败");
        return;
    }
    // 配置队列集
    priv->queueSet = xQueueCreateSet(2);
    xQueueAddToSet(priv->nrf_queue, priv->queueSet);
    xQueueAddToSet(priv->blecmd_queue, priv->queueSet);

    // 配置电源管理，自动进入轻度睡眠
    esp_pm_config_t pm_cfg = {
        .light_sleep_enable = true,
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
    };
    esp_pm_configure(&pm_cfg);

    i2c_master_init();
    // 初始化内部温度传感器，监控mcu温度
    xTaskCreate(system_maintain, "system maintain", 2048, NULL, 5, NULL);

    // 开始 ble
    ble_init();
    // 设置 ble 上位机控制命令队列
    bleunlock_set_queue(priv->blecmd_queue);
    // 开启 nrf
    nrf_app_start();
    
    // esp_sleep_enable_timer_wakeup(100000);
    // qmi8658c_init(); // PCB中断引脚没有接，还是得主机轮询，没用
    while (true)
    {
        // 等待队列消息
        QueueSetMemberHandle_t xActivatedMember = xQueueSelectFromSet(priv->queueSet, 100 / portTICK_PERIOD_MS);
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
        // 休眠了usb接口就响应主机串口了
        // esp_deep_sleep_start(); // 深度休眠
        // esp_sleep_wakeup_cause_t cause =  esp_sleep_get_wakeup_cause();
        // switch (cause)
        // {
        // case ESP_SLEEP_WAKEUP_TIMER:
        //     ESP_LOGI(TAG, "定时器唤醒");
        //     break;
        // case ESP_SLEEP_WAKEUP_BT:
        //     ESP_LOGI(TAG, "蓝牙唤醒");
        //     break;
        // default:
        //     ESP_LOGI(TAG, "其他原因唤醒");
        //     break;
        // }
    }
}
