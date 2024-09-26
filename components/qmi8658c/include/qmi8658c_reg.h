#pragma once
// The accelerometer low power mode is only available when the gyroscope is disabled  

// In 6DOF mode (accelerometer and gyroscope are both enabled), the ODR is derived from the nature frequency of gyroscope, refer to section 3.5 for more information. 

// General Purpose Registers 
#define WHO_AM_I 0x00 
#define REVISION_ID 0x01

// Setup and Control Registers
#define CTRL1 0x02 // 通信和传感器使能
#define CTRL2 0x03 // 加速度配置
#define CTRL3 0x04 // 陀螺仪配置
#define CTRL4 0x05 //保留
#define CTRL5 0x06 // 低通滤波器配置
#define CTRL6 0x07 // 姿态引擎配置
#define CTRL7 0x08 // Enable Sensors and Configure Data Reads
#define CTRL8 0x09 // 运动检测控制器配置
#define CTRL9 0x0A // 扩展接口主机控制器

// Host Controlled Calibration Registers
#define CAL1_L 0x0B
#define CAL1_H 0x0C
#define CAL2_L 0x0D
#define CAL2_H 0x0E
#define CAL3_L 0x0F
#define CAL3_H 0x10
#define CAL4_L 0x11
#define CAL4_H 0x12

// FIFO Registers 
#define FIFO_WTM_TH 0x13 // FIFO Watermark Register 
#define FIFO_CTRL 0x14 // FIFO Control Register
#define FIFO_SMPL_CNT 0x15 // FIFO Sample Count Register
#define FIFO_STATUS 0x16 // FIFO Status
#define FIFO_DATA 0x17 // fifo data output register

// Status Registers
#define STATUSINT 0x2D // Sensor Data Available and Lock Register
#define STATUS0 0x2E // 输出数据状态寄存器
#define STATUS1 0x2F // 杂项状态

// Timestamp Register
#define TIMESTAMP_L 0x30 //0x30-0x32, 时间戳寄存器
#define TIMESTAMP_M 0x31 //0x30-0x32, 时间戳寄存器
#define TIMESTAMP_H 0x32 //0x30-0x32, 时间戳寄存器
#define TEMP_H 0x51 //温度高位输出
#define TEMP_L 0x52

// Data Output Registers
#define AX_L 0x53 // X轴加速度低位
#define AX_H 0x54
#define AY_L 0x55
#define AY_H 0x56
#define AZ_L 0x57
#define AZ_H 0x58
#define GX_L 0x3B // X轴角速度低位
#define GX_H 0x3C
#define GY_L 0x3D
#define GY_H 0x3E
#define GZ_L 0x3F
#define GZ_H 0x40
#define DQW_L 0x49 // Quaternion Increment dQW in two’s complement. 
#define DQW_H 0x4A
#define DQX_L 0x4B
#define DQX_H 0x4C
#define DQY_L 0x4D
#define DQY_H 0x4E
#define DQZ_L 0x4F
#define DQZ_H 0x50
#define DVX_L 0x51 // Delta Velocity Output.
#define DVX_H 0x52
#define DVY_L 0x53
#define DVY_H 0x54
#define DVZ_L 0x55
#define DVZ_H 0x56
#define AE_REG1 0x57 // AttitudeEngine Register 1
#define AE_REG2 0x58 // AttitudeEngine Register 2

// Reset Register 
#define RESET 0x60 // 向此寄存器写入0xB0将复位


// #define 