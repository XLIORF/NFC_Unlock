#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "guards.h"
#include "rc522.h"
#include "rc522_registers.h"
#include "rc522_def.h"
#include "port.h"

static const char *TAG = "rc522";

/**
 * @brief Macro for safely freeing memory.
 *        This macro checks if the pointer is not NULL before calling the free
 * function. After freeing the memory, it sets the pointer to NULL to prevent
 * dangling pointer issues. This helps in avoiding double free errors and
 * improves the safety of memory management.
 * @param ptr Pointer to the memory to be freed.
 */
#define FREE(ptr)   \
    if (ptr)        \
    {               \
        free(ptr);  \
        ptr = NULL; \
    }

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

static void rc522_task(void *arg);

static esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n,
                               uint8_t *data)
{
    esp_err_t err = ESP_OK;
    uint8_t *buffer = NULL;

    // TODO: Find a way to send address + data without memory allocation
    ALLOC_JMP_GUARD(buffer = (uint8_t *)malloc(n + 1));

    buffer[0] = addr;
    memcpy(buffer + 1, data, n);

    switch (rc522->config->transport)
    {
    case RC522_TRANSPORT_SPI:
        ERR_JMP_GUARD(rc522_spi_send(rc522, buffer, n + 1));
        break;
    case RC522_TRANSPORT_I2C:
        ERR_JMP_GUARD(rc522_i2c_send(rc522, buffer, n + 1));
        break;
    default:
        ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_STATE,
                                  "write: Unknown transport");
        break;
    }

    JMP_GUARD_GATES(
        {
            ESP_LOGE(TAG, "Failed to write data (err: %s)", esp_err_to_name(err));
        },
        {});

    FREE(buffer);

    return err;
}

static inline esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr,
                                    uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

/* 统一底层传输函数，屏蔽传输协议 */
static esp_err_t rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n,
                              uint8_t *buffer)
{
    esp_err_t err;

    switch (rc522->config->transport)
    {
    case RC522_TRANSPORT_SPI:
        ERR_JMP_GUARD(rc522_spi_receive(rc522, buffer, n, addr));
        break;
    case RC522_TRANSPORT_I2C:
        ERR_JMP_GUARD(rc522_i2c_receive(rc522, buffer, n, addr));
        break;
    default:
        ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_STATE, "read: Unknown transport");
        break;
    }

    JMP_GUARD_GATES(
        { ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(err)); },
        {});

    return err;
}

/* 从RC522的addr中读取一个字节 */
static inline esp_err_t rc522_read(rc522_handle_t rc522, uint8_t addr,
                                   uint8_t *value_ref)
{
    return rc522_read_n(rc522, addr, 1, value_ref);
}

// 置位addr的指定位
static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr,
                                   uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp | mask);
}

// 清除addr的指定位
static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr,
                                     uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp & ~mask);
}

// 获取RC522的版本
static inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t *result)
{
    return rc522_read(rc522, RC522_VERSION_REG, result);
}

// 开启天线
static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ERR_RET_GUARD(rc522_read(rc522, RC522_TX_CONTROL_REG, &tmp));

    if (~(tmp & 0x03))
    {
        ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_TX_CONTROL_REG, 0x03));
    }

    return rc522_write(rc522, RC522_RF_CFG_REG, 0x60); // 43dB gain
}

// 克隆配置，带有默认值
static esp_err_t rc522_clone_config(rc522_config_t *config,
                                    rc522_config_t **result)
{
    rc522_config_t *_clone_config = NULL;

    ALLOC_RET_GUARD(_clone_config = calloc(1, sizeof(rc522_config_t)));

    memcpy(_clone_config, config, sizeof(rc522_config_t));

    // defaults
    _clone_config->scan_interval_ms = config->scan_interval_ms < 50
                                          ? RC522_DEFAULT_SCAN_INTERVAL_MS
                                          : config->scan_interval_ms;
    _clone_config->task_stack_size = config->task_stack_size == 0
                                         ? RC522_DEFAULT_TASK_STACK_SIZE
                                         : config->task_stack_size;
    _clone_config->task_priority = config->task_priority == 0
                                       ? RC522_DEFAULT_TASK_STACK_PRIORITY
                                       : config->task_priority;
    _clone_config->spi.clock_speed_hz = config->spi.clock_speed_hz == 0
                                            ? RC522_DEFAULT_SPI_CLOCK_SPEED_HZ
                                            : config->spi.clock_speed_hz;
    _clone_config->i2c.rw_timeout_ms = config->i2c.rw_timeout_ms == 0
                                           ? RC522_DEFAULT_I2C_RW_TIMEOUT_MS
                                           : config->i2c.rw_timeout_ms;
    _clone_config->i2c.clock_speed_hz = config->i2c.clock_speed_hz == 0
                                            ? RC522_DEFAULT_I2C_CLOCK_SPEED_HZ
                                            : config->i2c.clock_speed_hz;

    *result = _clone_config;

    return ESP_OK;
}

// 创建一个传输端口
static esp_err_t rc522_create_transport(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    switch (rc522->config->transport)
    {
    case RC522_TRANSPORT_SPI:
    {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = rc522->config->spi.clock_speed_hz,
            .mode = 0,
            .spics_io_num = rc522->config->spi.sda_gpio,
            .queue_size = 7,
            .flags = rc522->config->spi.device_flags,
        };

        rc522->bus_initialized_by_user = rc522->config->spi.bus_is_initialized;

        if (!rc522->bus_initialized_by_user)
        {
            spi_bus_config_t buscfg = {
                .miso_io_num = rc522->config->spi.miso_gpio,
                .mosi_io_num = rc522->config->spi.mosi_gpio,
                .sclk_io_num = rc522->config->spi.sck_gpio,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
            };

            ERR_RET_GUARD(
                spi_bus_initialize(rc522->config->spi.host, &buscfg, 0));
        }

        ERR_RET_GUARD(spi_bus_add_device(rc522->config->spi.host, &devcfg,
                                             &rc522->spi_handle));
    }
    break;
    case RC522_TRANSPORT_I2C:
    {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = rc522->config->i2c.sda_gpio,
            .scl_io_num = rc522->config->i2c.scl_gpio,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = rc522->config->i2c.clock_speed_hz,
        };

        ERR_RET_GUARD(i2c_param_config(rc522->config->i2c.port, &conf));
        ERR_RET_GUARD(i2c_driver_install(rc522->config->i2c.port, conf.mode,
                                             false, false, 0x00));
    }
    break;
    default:
        ESP_LOGE(TAG, "create_transport: Unknown transport");
        err = ESP_ERR_INVALID_STATE; // unknown transport
        break;
    }

    return err;
}

// 初始化一个RC522对象
esp_err_t rc522_create(rc522_config_t *config, rc522_handle_t *out_rc522)
{
    if (!config || !out_rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    rc522_handle_t rc522 = NULL;

    ALLOC_RET_GUARD(rc522 = calloc(1, sizeof(struct rc522)));

    ERR_LOG_AND_JMP_GUARD(rc522_clone_config(config, &(rc522->config)),
                              "Fail to clone config");

    ERR_LOG_AND_JMP_GUARD(rc522_create_transport(rc522),
                              "Fail to create transport");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    ERR_LOG_AND_JMP_GUARD(
        esp_event_loop_create(&event_args, &rc522->event_handle),
        "Fail to create event loop");

    rc522->running = true;

    CONDITION_LOG_AND_JMP_GUARD(
        pdTRUE != xTaskCreate(rc522_task, "rc522_task",
                              rc522->config->task_stack_size, rc522,
                              rc522->config->task_priority, &rc522->task_handle),
        "Fail to create task");

    JMP_GUARD_GATES(
        {
            rc522_destroy(rc522);
            rc522 = NULL;
        },
        { *out_rc522 = rc522; });

    return err;
}

// 注册事件处理函数
esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event,
                                esp_event_handler_t event_handler,
                                void *event_handler_arg)
{
    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS,
                                           event, event_handler,
                                           event_handler_arg);
}

// 注销事件处理函数
esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

// 序列号转成64位存储
static uint64_t rc522_sn_to_u64(uint8_t *sn)
{
    uint64_t result = 0;

    if (!sn)
    {
        return 0;
    }

    for (int i = 4; i >= 0; i--)
    {
        result |= ((uint64_t)sn[i] << (i * 8));
    }

    return result;
}

// 使用RC522计算CRC
// Buffer should be length of 2, or more
// Only first 2 elements will be used where the result will be stored
// TODO: Use 2+ bytes data type instead of buffer array
static esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data,
                                     uint8_t n, uint8_t *buffer)
{
    esp_err_t err = ESP_OK;
    uint8_t i = 255;
    uint8_t nn = 0;

    // 清除CRC终端
    ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_DIV_INT_REQ_REG, 0x04));
    // 取消正在执行的操作
    ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, CMD_IDLE));
    // 立即清除内部FIFO缓冲区的读写指针
    ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    // 像RC522的FIFO中写入数据
    ERR_RET_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    // 执行计算CRC命令
    ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, CMD_CalcCRC));

    for (;;)
    {
        ERR_RET_GUARD(rc522_read(rc522, RC522_DIV_INT_REQ_REG, &nn));

        i--;

        if (!(i != 0 && !(nn & 0x04)))
        { // 直到i=0或CRC中断被置位
            break;
        }
    }

    uint8_t tmp;

    ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_LSB_REG, &tmp));
    buffer[0] = tmp;

    ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_MSB_REG, &tmp));
    buffer[1] = tmp;

    return ESP_OK;
}

// 与卡片进行通信
static esp_err_t rc522_card_write(rc522_handle_t rc522, uint8_t cmd,
                                  uint8_t *data, uint8_t n, uint8_t *res_n,
                                  uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    uint8_t tmp;

    if (cmd == CMD_MFAuthent)
    {
        irq = 0x12;      // 与下面命令结合，表示允许错误和空闲中断输出到IRQ引脚上
        irq_wait = 0x10; // 与下面命令结合，表示空闲中断标志位
    }
    else if (cmd == CMD_Transceive)
    {
        irq = 0x77;      // 与下面命令结合，表示允许发送中断、接收中断、空闲中断、低级警报中断、错误中断、定时器中断输出到IRQ引脚
        irq_wait = 0x30; // 与下面命令结合，表示接收中断和空闲中断标志位
    }
    // irq | 0x80 表示输出到IRQ上的电平与寄存器反相，即低电平表示中断发生
    ERR_JMP_GUARD(rc522_write(rc522, RC522_COMM_INT_EN_REG, irq | 0x80));
    // 表示ComIrqReg寄存器中的标记位已清除
    ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_COMM_INT_REQ_REG, 0x80));
    // 停止命令执行，进入空闲状态
    ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, 0x00));
    // 立即清除内部FIFO缓冲区的读写指针
    ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    // 像FIFO中写入数据
    ERR_JMP_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    // 执行命令
    ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, cmd));

    if (cmd == CMD_Transceive)
    {
        // 开始数据发送，与CMD_Transmit配合使用
        ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));
    }

    uint16_t i = 1000;

    for (;;)
    {
        ERR_JMP_GUARD(rc522_read(rc522, RC522_COMM_INT_REQ_REG, &nn));

        i--;

        if (!(i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0))))
        { // i==0或触发空闲中断或触发需要的中断之一后退出
            break;
        }
    }
    // 清除开始发送位， 不知何用
    ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));

    if (i != 0)
    {
        // 读取错误寄存器
        ERR_JMP_GUARD(rc522_read(rc522, RC522_ERROR_REG, &tmp));

        if ((tmp & 0x1B) == 0x00)
        {
            err = ESP_OK;
            if (cmd == CMD_Transceive)
            {
                // 读出FIFO中的数据的个数，
                ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_LEVEL_REG, &nn));
                // 获取最后接受的有效位
                ERR_JMP_GUARD(rc522_read(rc522, RC522_CONTROL_REG, &tmp));

                last_bits = tmp & 0x07; // 去掉其他无效的位

                if (last_bits != 0) // 判断是否有无效位
                {
                    _res_n = (nn - 1) + last_bits; // 不明白为什么这样处理
                }
                else
                {
                    _res_n = nn;
                }
                if (nn == 0)
                    nn = 1;
                if (nn > MAXRLEN)
                    nn = MAXRLEN;
                if (_res_n > 0) // 读出有效数据
                {
                    ALLOC_JMP_GUARD(_result = (uint8_t *)malloc(_res_n));

                    for (i = 0; i < _res_n; i++)
                    {
                        ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_DATA_REG, &tmp));
                        _result[i] = tmp;
                    }
                }
            }
        }
    }
    rc522_set_bitmask(rc522, RC522_CONTROL_REG, 0x80);
    rc522_write(rc522, RC522_COMMAND_REG, CMD_IDLE);

    JMP_GUARD_GATES(
        {
            FREE(_result);
            _res_n = 0;
        },
        {
            *res_n = _res_n;
            *result = _result;
        });

    return err;
}

// 卡请求指令，卡片操作的第一步
// card_type[out]:返回卡片类型
static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t *res_n, uint8_t **card_type)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n = 0;
    uint8_t req_mode = 0x26;

    // 清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
    ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08));
    // 接受字节的第一个位开始对齐，在发送过程中最后一个字节的前7个位有效。
    ERR_RET_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x07));
    // 像卡片发送REQA（0x26）请求指令,返回两个字节的卡类型
    ERR_RET_GUARD(rc522_card_write(rc522, CMD_Transceive, &req_mode, 1, &_res_n, &_result));

    if (_res_n * 8 != 0x10)
    {
        free(_result);

        return ESP_ERR_INVALID_STATE;
    }
    *res_n = _res_n;
    *card_type = _result;

    return err;
}

// 防冲突
// sn[out]:保存四个字节序列码的堆栈指针
static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t **sn)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t _res_n;

    // 清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
    ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08));
    // 清理寄存器 停止收发
    ERR_JMP_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x00));
    // 清ValuesAfterColl所有接收的位在冲突后被清除
    ERR_RET_GUARD(rc522_clear_bitmask(rc522, COLL_REG, 0x80));
    // 发送卡指令Anticollision CL2（0x93 0x20），返回值位UID
    ERR_JMP_GUARD(rc522_card_write(rc522, CMD_Transceive, (uint8_t[]){0x93, 0x20}, 2, &_res_n, &_result));

    // TODO: Some cards have length of 4, and some of them have length of 7 bytes
    //       here we are using one extra byte which is not part of UID.
    //       Implement logic to determine the length of the UID and use that info
    //       to retrieve the serial number aka UID
    if (_result && _res_n != 5)
    { // all cards/tags serial numbers is 5 bytes long (??)
        ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_RESPONSE, "invalid length of serial number");
    }

    JMP_GUARD_GATES(
        {
            FREE(_result);
            _res_n = 0;
        },
        { *sn = _result; });

    return err;
}

/// @brief 选择卡
/// @param rc522[in] rc522处理句柄
/// @param sn[in] 卡片序列号
/// @return 成功返回ESP_OK
static esp_err_t rc522_selece_card(rc522_handle_t rc522, uint8_t *sn)
{
    esp_err_t err = ESP_OK;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;
    uint8_t data_buf[MAXRLEN];
    memset(data_buf, '\0', MAXRLEN);
    data_buf[0] = 0x93;
    data_buf[1] = 0x70;
    data_buf[6] = 0x00;
    for (int i = 0; i < 4; i++)
    {
        data_buf[i + 2] = *(sn + i);
        data_buf[6] ^= *(sn + i);
    }
    rc522_calculate_crc(rc522, data_buf, 7, &data_buf[7]);
    rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08);

    err = rc522_card_write(rc522, CMD_Transceive, data_buf, 9, &res_data_n, &res_data);
    if (err != ESP_OK || res_data_n == 0)
    {
        rc522_loge("选卡函数返回失败");
        return err;
    }
    // rc522_log("选卡返回结果：");
    // rc522_hexdump(res_data, res_data_n);
    FREE(res_data);
    if (res_data_n * 8 == 0x18)
        err = ESP_OK;
    else
        err = ESP_FAIL;
    return err;
}

// 功    能：验证卡片密码
// 参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char rc522_card_block_auth(rc522_handle_t rc522, uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pSnr)
{
    esp_err_t err = ESP_OK;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;
    uint8_t i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i = 0; i < 6; i++)
    {
        ucComMF522Buf[i + 2] = *(pKey + i);
    }
    for (i = 0; i < 6; i++)
    {
        ucComMF522Buf[i + 8] = *(pSnr + i);
    }
    ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08));
    ERR_JMP_GUARD(rc522_card_write(rc522, CMD_MFAuthent, ucComMF522Buf, 12, &res_data_n, &res_data));
    uint8_t status = 0;
    ERR_JMP_GUARD(rc522_read(rc522, RC522_STATUS_2_REG, &status));
    rc522_log("密钥验证%s", ((status & 0x08) == 0x08) ? "成功" : "失败");
    // rc522_log("密钥验证返回值：");
    // rc522_hexdump(res_data, res_data_n);
    if (!(status & 0x08))
    {
        err = ESP_FAIL;
    }
    else
    {
        err = ESP_OK;
    }
    JMP_GUARD_GATES({ FREE(res_data); }, {});
    FREE(res_data);
    return err;
}

/*
 * 函数名：rc522_card_block_write
 * 描述  ：写数据到M1卡一块
 * 输入  ：uint8_t ucAddr，块地址
 *         pData，写入的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char rc522_card_block_write(rc522_handle_t rc522, uint8_t ucAddr, uint8_t *pData)
{
    esp_err_t err = ESP_OK;
    uint8_t uc, ucComMF522Buf[MAXRLEN];
    uint8_t ulLen;
    uint8_t *resault = NULL;

    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = ucAddr;

    rc522_calculate_crc(rc522, ucComMF522Buf, 2, &ucComMF522Buf[2]);
    err = rc522_card_write(rc522, CMD_Transceive, ucComMF522Buf, 4, &ulLen, &resault);
    if ((err != ESP_OK) || (ulLen != 4) || ((resault[0] & 0x0F) != 0x0A))
    {
        err = ESP_FAIL;
    }
    FREE(resault);
    if (err == ESP_OK)
    {
        memcpy(ucComMF522Buf, pData, 16);
        for (uc = 0; uc < 16; uc++)
            ucComMF522Buf[uc] = *(pData + uc);

        rc522_calculate_crc(rc522, ucComMF522Buf, 16, &ucComMF522Buf[16]);

        err = rc522_card_write(rc522, CMD_Transceive, ucComMF522Buf, 18, &ulLen, &resault);

        if ((err != ESP_OK) || (ulLen != 4) || ((resault[0] & 0x0F) != 0x0A))
            err = ESP_FAIL;
    }
    FREE(resault);
    return err;
}

/*
 * 函数名：rc522_card_block_read
 * 描述  ：读取M1卡一块数据
 * 输入  ：uint8_t ucAddr，块地址
 *         pData，读出的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char rc522_card_block_read(rc522_handle_t rc522, uint8_t ucAddr, uint8_t **pData)
{
    esp_err_t err = ESP_OK;
    uint8_t ucComMF522Buf[MAXRLEN];
    uint8_t ulLen;
    uint8_t *result = NULL;

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = ucAddr;

    rc522_calculate_crc(rc522, ucComMF522Buf, 2, &ucComMF522Buf[2]);

    err = rc522_card_write(rc522, CMD_Transceive, ucComMF522Buf, 4, &ulLen, &result);
    esp_log_buffer_hexdump_internal(TAG, result, ulLen, ESP_LOG_INFO);
    if ((err == ESP_OK) && (ulLen * 8 == 0x90))
    {
        *pData = result;
    }
    else
    {
        ESP_LOGE(TAG, "读取扇区失败");
        err = ESP_FAIL;
        *pData = NULL;
        FREE(result);
    }
    return err;
}

// 将第sector个扇区的第block个块转换为地址，从零开始
static inline uint8_t self_to_addr(uint8_t sector, uint8_t block)
{
    return sector * 4 + block;
}

rc522_err_t rc522_card_halt(rc522_handle_t rc522)
{
    rc522_err_t err = RC522_OK;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;
    uint8_t buf[] = {0x50, 0x00, 0x00, 0x00};
    // 计算CRC，填充指令
    err = rc522_calculate_crc(rc522, buf, 2, &buf[2]);
    // 发送卡指令 Halt（0x50 0x00）
    err = rc522_card_write(rc522, CMD_Transceive, buf, 4, &res_data_n, &res_data);
    FREE(res_data);
    // 表示MIFARE
    // Crypto1单元已打开，因此与卡的所有数据通信都已加密。只能通过成功执行仅在MIFARE标准卡的读/写模式下有效的MFAuthent命令将其设置为逻辑1。此位由软件清除
    err = rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08);
    return err;
}

// UID为你要修改的卡的UID key_type：0为KEYA，非0为KEYB KEY为密钥 RW:1是读，0是写 data_addr为修改的地址 data为数据内容
rc522_err_t rc522_card_block_RW(rc522_handle_t rc522, uint8_t *UID, uint8_t key_type, uint8_t *KEY, uint8_t RW, uint8_t data_addr, uint8_t *data)
{
    if (rc522 == NULL || KEY == NULL)
    {
        return RC522_INVALID_ARG;
    }
    uint8_t len = 0;
    uint8_t *card_type;
    esp_err_t err = rc522_request(rc522, &len, &card_type); // 寻卡
    if (err != ESP_OK)
    {
        return RC522_NO_CARD;
    }
    uint8_t *sn;
    err = rc522_anticoll(rc522, &sn); // 防冲撞
    if (err != ESP_OK)
        return RC522_ANTICOLL_FAILD;

    err = rc522_selece_card(rc522, sn); // 选定卡
    if (err != ESP_OK)
    {
        rc522_loge("序列码不匹配\r\n");
        return RC522_UID_NO_MATCH;
    }

    if (0 == key_type)
        err = rc522_card_block_auth(rc522, AUTH_KEYA, data_addr, KEY, sn); // 校验
    else
        err = rc522_card_block_auth(rc522, AUTH_KEYB, data_addr, KEY, sn); // 校验
    if (err != ESP_OK)
    {
        rc522_loge("密钥不正确\r\n");
        return RC522_AUTH_FAILD;
    }

    uint8_t *_data;
    if (RW) // 读写选择，1是读，0是写
    {
        err = rc522_card_block_read(rc522, data_addr, &_data);
        if (err == ESP_OK)
        {
            // rc522_log("data:");
            // rc522_hexdump(_data, 16);
            memcpy(data, _data, 16);
        }
        else
        {
            rc522_loge("rc522_card_block_read() failed\r\n");
            return RC522_CARD_BLOCK_READ_FAILD;
        }
    }
    else
    {
        err = rc522_card_block_write(rc522, data_addr, data);
        if (err == ESP_OK)
        {
            rc522_log("rc522_card_block_write() finished\r\n");
        }
        else
        {
            rc522_loge("rc522_card_block_write() failed\r\n");
            return RC522_CARD_BLOCK_WRITE_FAILD;
        }
    }

    err = rc522_card_halt(rc522);
    if (err == ESP_OK)
    {
        rc522_log("PcdHalt() finished\r\n");
    }
    else
    {
        rc522_loge("PcdHalt() failed\r\n");
        return RC522_CARD_FAILD;
    }

    return RC522_OK;
}

// 搜卡，防冲突，暂停，返回序列号
static esp_err_t rc522_get_tag(rc522_handle_t rc522, uint8_t **result)
{
    esp_err_t err = ESP_OK;
    uint8_t *_result = NULL;
    uint8_t *res_data = NULL;
    uint8_t res_data_n;
    ERR_JMP_GUARD(rc522_request(rc522, &res_data_n, &res_data));

    if (res_data != NULL)
    {
        FREE(res_data);
        ERR_JMP_GUARD(rc522_anticoll(rc522, &_result));

        if (_result != NULL)
        {
            FREE(res_data);
            rc522_card_halt(rc522);
        }
    }

    JMP_GUARD_GATES(
        {
            FREE(_result);
            FREE(res_data);
        },
        { *result = _result; });

    return err;
}

// 初始化RC522并开始扫描
esp_err_t rc522_start(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (rc522->scanning)
    { // Already in scan mode
        return ESP_OK;
    }

    uint8_t tmp = 0;

    if (!rc522->initialized)
    {
        // Initialization will be done only once, on the first call of start
        // function

        // TODO: Extract test in dedicated function
        // ---------- RW test ------------
        // TODO: Use less sensitive register for the test, or return the value
        //       of this register to the previous state at the end of the test
        const uint8_t test_addr = RC522_MOD_WIDTH_REG, test_val = 0x25;
        uint8_t pass = 0;

        for (uint8_t i = test_val; i < test_val + 2; i++)
        {
            err = rc522_write(rc522, test_addr, i);

            if (err == ESP_OK)
            {
                err = rc522_read(rc522, test_addr, &tmp);

                if (err == ESP_OK && tmp == i)
                {
                    pass = 1;
                }
            }

            if (pass != 1)
            {
                ESP_LOGE(TAG, "Read/write test failed");
                rc522_destroy(rc522);

                return err;
            }
        }
        // ------- End of RW test --------
        // 软件复位
        ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, CMD_SoftResert));
        ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_MODE_REG, 0x8D));
        ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_PRESCALER_REG, 0x3E));
        ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_RELOAD_LSB_REG, 0x1E));
        ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_RELOAD_MSB_REG, 0x00));
        ERR_RET_GUARD(rc522_write(rc522, RC522_TX_ASK_REG, 0x40));
        ERR_RET_GUARD(rc522_write(rc522, RC522_MODE_REG, 0x3D));

        ERR_RET_GUARD(rc522_antenna_on(rc522));
        ERR_RET_GUARD(rc522_firmware(rc522, &tmp));

        rc522->initialized = true;

        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (tmp & 0x03));
    }

    rc522->scanning = true;

    return ESP_OK;
}

// 暂停扫描
esp_err_t rc522_pause(rc522_handle_t rc522)
{
    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!rc522->scanning)
    {
        return ESP_OK;
    }

    rc522->scanning = false;

    return ESP_OK;
}

// 销毁传输接口
static esp_err_t rc522_destroy_transport(rc522_handle_t rc522)
{
    esp_err_t err;

    switch (rc522->config->transport)
    {
    case RC522_TRANSPORT_SPI:
        err = spi_bus_remove_device(rc522->spi_handle);
        if (rc522->bus_initialized_by_user)
        {
            err = spi_bus_free(rc522->config->spi.host);
        }
        break;
    case RC522_TRANSPORT_I2C:
        err = i2c_driver_delete(rc522->config->i2c.port);
        break;
    default:
        ESP_LOGW(TAG, "destroy_transport: Unknown transport");
        err = ESP_ERR_INVALID_STATE;
    }

    return err;
}

// 销毁RC522对象
esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xTaskGetCurrentTaskHandle() == rc522->task_handle)
    {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler");

        return ESP_ERR_INVALID_STATE;
    }

    err = rc522_pause(rc522); // stop task
    rc522->running = false;   // stop rc522 -> task will delete itself

    // TODO: Wait here for task to exit

    err = rc522_destroy_transport(rc522);

    if (rc522->event_handle)
    {
        err = esp_event_loop_delete(rc522->event_handle);
        rc522->event_handle = NULL;
    }

    FREE(rc522->config);
    FREE(rc522);

    return err;
}

// 触发一个事件
static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void *data)
{
    esp_err_t err;

    if (!rc522)
    {
        return ESP_ERR_INVALID_ARG;
    }

    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };

    ERR_RET_GUARD(esp_event_post_to(rc522->event_handle, RC522_EVENTS, event,
                                        &e_data, sizeof(rc522_event_data_t),
                                        portMAX_DELAY));

    return esp_event_loop_run(rc522->event_handle, 0);
}

void block_rw_test(rc522_handle_t rc522)
{
    unsigned char KeyA_default[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    uint8_t data[16];
    data[0] = 'H';
    data[1] = 'e';
    data[2] = 'l';
    data[3] = 'l';
    data[4] = 'o';
    data[5] = ',';
    data[6] = 'f';
    data[7] = 'r';
    data[8] = 'o';
    data[9] = 'm';
    data[10] = ' ';
    data[11] = 'w';
    data[12] = 'l';
    data[13] = 'x';
    data[14] = '.';
    data[14] = 0;
    rc522_card_block_RW(rc522, NULL, 0, KeyA_default, 0, self_to_addr(3, 0), data);
    rc522_card_block_RW(rc522, NULL, 0, KeyA_default, 1, self_to_addr(3, 0), data);
    rc522_log("读出扇区数据：");
    rc522_hexdump(data, 16);
}

// RC522的任务
static void rc522_task(void *arg)
{
    rc522_handle_t rc522 = (rc522_handle_t)arg;

    while (rc522->running)
    {
        if (rc522->scanning)
        {
            uint8_t *serial_no_array = NULL;
            if (ESP_OK != rc522_get_tag(rc522, &serial_no_array))
            {
                // Tag is not present
                //
                // TODO: Implement logic to know when the error is due to
                //       tag absence or some other protocol issue
            }

            if (!serial_no_array)
            {
                rc522->tag_was_present_last_time = false;
            }
            else if (!rc522->tag_was_present_last_time)
            {
                rc522_tag_t tag = {
                    .serial_number = rc522_sn_to_u64(serial_no_array),
                    .snp = (uint8_t[]){serial_no_array[0], serial_no_array[1], serial_no_array[2], serial_no_array[3]}};
                FREE(serial_no_array);
                rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
                rc522->tag_was_present_last_time = true;
                // block_rw_test(rc522);
            }
            else
            {
                FREE(serial_no_array);
            }

            int delay_interval_ms = rc522->config->scan_interval_ms;

            if (rc522->tag_was_present_last_time)
            {
                delay_interval_ms *= 2; // extra scan-bursting prevention
            }
            vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
            continue;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
