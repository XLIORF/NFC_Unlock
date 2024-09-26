/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_as608_interface_template.c
 * @brief     driver as608 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2023-09-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/09/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_as608_interface.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <stdarg.h>  
#include <stdio.h>

static const int RX_BUF_SIZE = 1024;
#define UART_HOST UART_NUM_1
#define TXD_PIN (GPIO_NUM_3)
#define RXD_PIN (GPIO_NUM_4)
/**
 * @brief  interface uart init
 * @return status code
 *         - 0 success
 *         - 1 uart init failed
 * @note   none
 */
uint8_t as608_interface_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_HOST, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_HOST, &uart_config);
    uart_set_pin(UART_HOST, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return 0;
}

/**
 * @brief  interface uart deinit
 * @return status code
 *         - 0 success
 *         - 1 uart deinit failed
 * @note   none
 */
uint8_t as608_interface_uart_deinit(void)
{
    
    return -1 * uart_driver_delete(UART_HOST);
}

/**
 * @brief      interface uart read
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint16_t as608_interface_uart_read(uint8_t *buf, uint16_t len)
{
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_HOST, (size_t*)&length));
    // if(length < len)
    // {
    //     ESP_LOGE("uart接口", "长度不够(已有:%d, 需要:%d)，通信失败", length, len);
    //     // return 1;
    // }
    const int rxBytes = uart_read_bytes(UART_HOST, buf, length, 1000 / portTICK_PERIOD_MS);
    // esp_log_buffer_hexdump_internal("Debug uart读", buf, len, ESP_LOG_WARN);
    return rxBytes;
}

/**
 * @brief  interface uart flush
 * @return status code
 *         - 0 success
 *         - 1 uart flush failed
 * @note   none
 */
uint8_t as608_interface_uart_flush(void)
{
    // return 0;
    return -1 * uart_flush(UART_HOST);
}

/**
 * @brief     interface uart write
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t as608_interface_uart_write(uint8_t *buf, uint16_t len)
{
    // ESP_LOGW("验证", "uart写：%")
    // esp_log_buffer_hexdump_internal("Debug uart写", buf, len, ESP_LOG_WARN);
    const int txBytes = uart_write_bytes(UART_HOST, buf, len);
    // const int txBytes = uart_write_bytes_with_break(UART_HOST, buf, len, 500);
    esp_err_t ret = uart_wait_tx_done(UART_HOST, 500);
    return ret == ESP_OK ? (txBytes > 0 ? 0 : 1) : 1;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void as608_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void as608_interface_debug_print(const char *const fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);
}
