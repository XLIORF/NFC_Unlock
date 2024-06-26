#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_event.h>
#include <driver/spi_master.h>
#include <driver/i2c.h> // TODO: Log warning: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`
#include "rc522_def.h"
#define RC522_I2C_ADDRESS (0x28)

#define RC522_DEFAULT_SCAN_INTERVAL_MS (125)
#define RC522_DEFAULT_TASK_STACK_SIZE (4 * 1024)
#define RC522_DEFAULT_TASK_STACK_PRIORITY (4)
#define RC522_DEFAULT_SPI_CLOCK_SPEED_HZ (5000000)
#define RC522_DEFAULT_I2C_RW_TIMEOUT_MS (1000)
#define RC522_DEFAULT_I2C_CLOCK_SPEED_HZ (100000)

ESP_EVENT_DECLARE_BASE(RC522_EVENTS);

typedef struct rc522* rc522_handle_t;

typedef enum {
    RC522_TRANSPORT_SPI,
    RC522_TRANSPORT_I2C,
} rc522_transport_t;

typedef struct {
    uint16_t scan_interval_ms;         /*<! How fast will ESP32 scan for nearby tags, in miliseconds */
    size_t task_stack_size;            /*<! Stack size of rc522 task */
    uint8_t task_priority;             /*<! Priority of rc522 task */
    rc522_transport_t transport;       /*<! Transport that will be used. Defaults to SPI */
    union {
        struct {
            spi_host_device_t host;
            int miso_gpio;
            int mosi_gpio;
            int sck_gpio;
            int sda_gpio;
            int clock_speed_hz;
            uint32_t device_flags;     /*<! Bitwise OR of SPI_DEVICE_* flags */
            /**
             * @brief Set to true if the bus is already initialized. 
             *        NOTE: This property will be removed in future,
             *        once when https://github.com/espressif/esp-idf/issues/8745 is resolved
             * 
             */
            bool bus_is_initialized;
        } spi;
        struct {
            i2c_port_t port;
            int sda_gpio;
            int scl_gpio;
            int rw_timeout_ms;
            uint32_t clock_speed_hz;
        } i2c;
    };
} rc522_config_t;

typedef enum {
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,
    RC522_EVENT_TAG_SCANNED,             /*<! Tag scanned */
    RC522_EVENT_FIND_CARD,
    RC522_EVENT_START_ANTI,
    RC522_EVENT_SELECT_CARD_COMPLETED,
    RC522_EVENT_AUTHENTIATE_COMPLETED,
    RC522_EVENT_CARD_WRITE,
    RC522_EVENT_CARD_READ,
    RC522_EVENT_CARD_HALT,
} rc522_event_t;

typedef struct {
    rc522_handle_t rc522;
    void* ptr;
} rc522_event_data_t;

typedef struct {
    uint64_t serial_number;
    uint8_t* snp;
    uint16_t card_type;
} rc522_tag_t;

/**
 * @brief Create RC522 scanner handle.
 *        To start scanning tags call the rc522_start function.
 * @param config Configuration
 * @param out_rc522 Pointer to resulting new handle
 * @return ESP_OK on success
 */
esp_err_t rc522_create(rc522_config_t* config, rc522_handle_t* out_rc522);

esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg);

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned. Initialization function had to be
 *        called before this one.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_start(rc522_handle_t rc522);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
#define rc522_resume(rc522) rc522_start(rc522)

/**
 * @brief Pause scan tags. If already paused, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_pause(rc522_handle_t rc522);

/**
 * @brief Destroy RC522 and free all resources. Cannot be called from event handler.
 * @param rc522 Handle
 */
esp_err_t rc522_destroy(rc522_handle_t rc522);


/// @brief 读写卡片的某一个扇区
/// @param rc522 句柄
/// @param UID 卡片的UID，暂时没有实现 //todo
/// @param key_type 密钥类型，0表示KeyA，其他表示KeyB
/// @param KEY 密钥指针，长度位6个字节
/// @param RW 0表示写， 1表示度
/// @param data_addr 块地址
/// @param data 数据缓存区，自行创建然后传入
/// @return 
rc522_err_t rc522_card_block_RW(rc522_handle_t rc522, uint8_t *UID, uint8_t key_type, uint8_t *KEY, uint8_t RW, uint8_t data_addr, uint8_t *data);

#ifdef __cplusplus
}
#endif
