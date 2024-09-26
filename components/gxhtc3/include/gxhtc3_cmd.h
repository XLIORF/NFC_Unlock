#pragma once
#include <stdint.h>

static uint8_t cmd[][2] = {
    {0xEF, 0xC8}, // 读ID
    {0xB0, 0x98}, // 进入休眠模式
    {0x35, 0x17}, // 从休眠中唤醒
    {0x7C, 0xA2}, // 正常模式温度在前， Clock stretching开启
    {0x5C, 0x24}, // 正常模式湿度在前， Clock stretching开启
    {0x64, 0x58}, // 低功耗模式温度在前， Clock stretching开启
    {0x44, 0xDE}, // 低功耗模式湿度在前， Clock stretching开启

    {0x78, 0x66}, // 正常模式温度在前， Clock stretching关闭
    {0x58, 0xE0}, // 正常模式湿度在前， Clock stretching关闭
    {0x60, 0x90}, // 低功耗模式温度在前， Clock stretching关闭
    {0x40, 0x1A}, // 低功耗模式湿度在前， Clock stretching关闭
    {0x80, 0x5D}, // 软复位
};

uint8_t *gxhtc3_cmd_read_id = cmd[0];
uint8_t *gxhtc3_cmd_sleep = cmd[1];
uint8_t *gxhtc3_cmd_wakeup = cmd[2];
uint8_t *gxhtc3_cmd_measure_normal_th_c = cmd[3];
uint8_t *gxhtc3_cmd_measure_lowpower_th_c = cmd[5];
uint8_t *gxhtc3_cmd_measure_normal_th = cmd[7];
uint8_t *gxhtc3_cmd_measure_lowpower_th = cmd[9];
uint8_t *gxhtc3_cmd_software_reset = cmd[11];
