#pragma once
// The accelerometer low power mode is only available when the gyroscope is disabled

// In 6DOF mode (accelerometer and gyroscope are both enabled), the ODR is derived from the nature frequency of
// gyroscope, refer to section 3.5 for more information.

// General Purpose Registers
#define WHO_AM_I 0x00
#define REVISION_ID 0x01

// Setup and Control Registers
#define CTRL1 0x02 // 通信和传感器使能
#define CTRL2 0x03 // 加速度配置
#define CTRL3 0x04 // 陀螺仪配置
#define CTRL4 0x05 // 保留
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
#define FIFO_WTM_TH 0x13   // FIFO Watermark Register
#define FIFO_CTRL 0x14     // FIFO Control Register
#define FIFO_SMPL_CNT 0x15 // FIFO Sample Count Register
#define FIFO_STATUS 0x16   // FIFO Status
#define FIFO_DATA 0x17     // fifo data output register

// Status Registers
#define STATUSINT 0x2D // Sensor Data Available and Lock Register
#define STATUS0 0x2E   // 输出数据状态寄存器
#define STATUS1 0x2F   // 杂项状态

// Timestamp Register
#define TIMESTAMP_L 0x30 // 0x30-0x32, 时间戳寄存器
#define TIMESTAMP_M 0x31 // 0x30-0x32, 时间戳寄存器
#define TIMESTAMP_H 0x32 // 0x30-0x32, 时间戳寄存器
#define TEMP_H 0x51      // 温度高位输出
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
// CTRL1 寄存器可选配置
#define SIM (1U << 7)     // 使能3-wire SPI 否则4-wire SPI
#define ADDR_AI (1U << 6) // 使能穿行地址自增,否则不自增
#define BE (1U << 5)      // 使能大端模式,否则小端模式
// Reserved
#define Sensor_Disable (1U) // 启动2MHz内部有源晶振

// CTRL2 寄存器可选配置
#define aST (1U << 7)        // 使能加速度计自测
#define aFS_2g (0b000U << 4) // 加速度满量程为正负两倍重力加速度
#define aFS_4g (0b001U << 4)
#define aFS_8g (0b010U << 4)
#define aFS_16g (0b011U << 4)
#define aODR_6D_7520N (0) // 陀螺仪 + 加速度计 在正常模式(高精度模式)下加速度计的输出频率为7520Hz
#define aODR_6D_3760N (1)
#define aODR_6D_1880N (2)
#define aODR_6D_940N (3)
#define aODR_6D_470N (4)
#define aODR_6D_235N (5)
#define aODR_6D_117_5N (6) // 陀螺仪 + 加速度计 在正常模式(高精度模式)下加速度计的输出频率为117.5Hz
#define aODR_6D_58_75N (7)
#define aODR_6D_29_375N (8)
#define aODR_Only_1000N (3) // 只有加速度计 在正常模式(高精度模式)下加速度计的输出频率为1kHz
#define aODR_Only_500N (4)
#define aODR_Only_250N (5)
#define aODR_Only_125N (6)
#define aODR_Only_62_5N (7)
#define aODR_Only_31_25N (8)
#define aODR_Only_128LP (12) // 只有加速度计 在低功耗模式下加速度计的输出频率为128Hz
#define aODR_Only_21LP (13)
#define aODR_Only_11LP (14)
#define aODR_Only_3LP (15)

// CTRL3 寄存器可选配置
#define gST (1U << 7)         // 使能陀螺仪自测
#define gFS_16dps (0U << 4)   // 设置陀螺仪满量程为正负16度每秒
#define gFS_32dps (1U << 4)   //
#define gFS_64dps (2U << 4)   //
#define gFS_128dps (3U << 4)  //
#define gFS_256dps (4U << 4)  //
#define gFS_512dps (5U << 4)  //
#define gFS_1024dps (6U << 4) //
#define gFS_2048dps (7U << 4) //
#define gODR_7520N (0U)       // 设置陀螺仪的输出频率为7520Hz
#define gODR_3760N (1U)       //
#define gODR_1880N (2U)       //
#define gODR_940N (3U)        //
#define gODR_470N (4U)        //
#define gODR_235N (5U)        //
#define gODR_117_5N (6U)      // 设置陀螺仪的输出频率为117.5Hz
#define gODR_58_75N (7U)      //
#define gODR_29_375N (8U)     //

// CTRL4 寄存器配置
// 未使用

// CTRL5
#define gLPF_BW_2_66 (0U << 5) // 设置陀螺仪的低通滤波频宽为输出频率的2.66%
#define gLPF_BW_3_63 (1U << 5) // 设置陀螺仪的低通滤波频宽为输出频率的3.63%
#define gLPF_BW_5_39 (2U << 5)
#define gLPF_BW_13_7 (3U << 5)
#define gLPF_EN (1U << 4)      // 以给定的频宽使能陀螺仪低通滤波器
#define aLPF_BW_2_66 (0U << 1) // 设置加速度计的低通滤波频宽为输出频率的2.66%
#define aLPF_BW_3_63 (1U << 1) // 设置加速度计的低通滤波频宽为输出频率的3.63%
#define aLPF_BW_5_39 (2U << 1)
#define aLPF_BW_13_7 (3U << 1)
#define aLPF_EN (1U << 0) // 以给定的频宽使能加速度计低通滤波器

// CTRL6
#define sMoD (1U << 7) // 使能Motion on Demand (需要 sEN=1)
#define sODR_1 (0)     // 设置姿态引擎的输出频率为1Hz
#define sODR_2 (1)
#define sODR_4 (2)
#define sODR_8 (3)
#define sODR_16 (4)
#define sODR_32 (5)
#define sODR_64 (6)

// CTRL7
#define syncSmpl (1U << 7) // 使能同步采样模式
#define sys_hs (1U << 6)   // 高速内部时钟,否则基于ODR的时钟
#define gSN (1U << 4)      // 使能贪睡模式陀螺仪(仅启动Driver),否则全模式陀螺仪(Driver+Sense)
#define sEN (1U << 3)      // 使能姿态引擎(AE)方向和速度增量计算, 否则失能
#define gEN (1U << 1)      // 使能陀螺仪,否则失能
#define aEN (1U)           // 使能加速度计,否则失能

// CTRL8 运动检测控制
#define C9_HandshakingType_INT1 (1U << 7) // 使用INT1作为CTRL9 handshake, 否则使用STATUSINT.bit7
#define INT_PIN_4MDE                                                                                                   \
    (1U << 6) // INT2 用于移动侦测事件中断, 否则...手册有问题 //Note: this bit will influence the any/no/sig-motion,
              // pedometer,  tap detection interrupt
#define PedometerEN (1U << 4)         // 使能计步引擎
#define SignificantMotionEN (1U << 3) // 显著运动引擎
#define NoMotionEN (1U << 2)          // 使能无运动引擎
#define AnyMotionEN (1U << 1)         // 使能任意运动引擎
#define TapEN (1U)                    // 使能Tap引擎

// CTRL9 CTRL9寄存器命令
#define CTRL_CMD_NOP 0x00                     // Ctrl9 无操作
#define CTRL_CMD_GYRO_BIAS 0x01               // WCtrl9 从CAL寄存器复制biasg[x/y/z]到FIFO并设置GYROBIAS_PEND
#define CTRL_CMD_REQ_SDI 0x03                 // Ctrl9R SDI MOD 请求读取SDI数据
#define CTRL_CMD_RST_FIFO 0x04                // Ctrl9 复位FIFO
#define CTRL_CMD_REQ_FIFO 0x05                // Ctrl9R 从设备获取FIFO数据
#define CTRL_CMD_WRITE_WOM_SETTING 0x08       // WCtrl9 设置和使能WoM
#define CTRL_CMD_ACCEL_HOST_DELTA_OFFSET 0x09 // WCtrl9 改变加速度计偏移量
#define CTRL_CMD_GYRO_HOST_DELTA_OFFSET 0x0A  // WCtrl9 改变陀螺仪偏移量
#define CTRL_CMD_COPY_USID 0x10               // Ctrl9R 读取 USID_Bytes 和 FW_Version 字节
#define CTRL_CMD_SET_RPU 0x11                 // WCtrl9 配置 IO 上拉
#define CTRL_CMD_AHB_CLOCK_GATING 0x12        // WCtrl9 内部AHB时钟门控开关
#define CTRL_CMD_ON_DEMAND_CALIBRATION 0xA2   // WCtrl9 陀螺仪按需校准

// FIFO_CTRL
#define FIFO_RD_MODE (1U << 7)  // 读取完成后必须清除以恢复向FIFO写入
#define FIFO_SIZE_16 (0U)       // 16个采样
#define FIFO_SIZE_32 (1U << 2)  // 16个采样
#define FIFO_SIZE_64 (2U << 2)  // 16个采样
#define FIFO_SIZE_128 (3U << 2) // 16个采样
#define FIFO_MODE_Bypass (0U)   // 使能FIFO
#define FIFO_MODE_FIFO (1U)
#define FIFO_MODE_Stream (2U)
#define FIFO_MODE_Stream2FIFO (3U) // 一旦发生运动/手势/中断,FIFO将被清空

// FIFO_STATUS 位掩码
#define BIT_FIFO_FULL (1U << 7)      // FIFO已满
#define BIT_FIFO_WTM (1U << 6)       // FIFO 中数据量达到水位线(一个预警设置)
#define BIT_FIFO_OVFLOW (1U << 5)    // FIFO溢出
#define BIT_FIFO_NOT_EMPTY (1U << 4) // FIFO非空
#define BIT_FIFO_SMPL_CNT_MSB (11U)

// STATUSINT 掩码
#define BIT_CTRL9_CmdDone                                                                                              \
    (1U << 7) // 完成,否则未完成 //主机处理器作为 CTRL9 寄存器协议的一部分读取位。用于指示 ctrl9
              // 命令已完成。有关详细信息，请参阅 5.9.3 和 5.9.4。
#define BIT_Locked (1U << 1) // syncSmpl=1:传感器数据锁定; syncSmpl=0:与INT1拥有相同的值
#define BIT_Avail (1U)       // syncSmpl=1:传感器数据可读; syncSmpl=0:与INT2拥有相同的值

// STATUS0 掩码
#define BIT_sDA (1U << 3) // 姿态引擎有新数据
#define BIT_gDA (1U << 1) // 陀螺仪有新数据
#define BIT_aDA (1U << 0) // 加速度计有新数据

// STATUS1 掩码
#define BIT_SignificantMotion (1U << 7) // Signifit Motion 触发
#define BIT_NoMotion (1U << 6)
#define BIT_AnyMotion (1U << 5)
#define BIT_Pedometer (1U << 4)
#define BIT_TAP (1U << 1)