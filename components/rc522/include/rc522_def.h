#pragma once
#ifndef __RC522_DEF_H__
#define __RC522_DEF_H__
#define CMD_IDLE                0x00 // 0b0000 空操作 让RC522处于空闲状态
#define CMD_Mem                 0x01 // 0b0001
#define CMD_GenerateRandomID    0x02 // 0b0010
#define CMD_CalcCRC             0x03 // 0b0011
#define CMD_Transmit            0x04 // 0b0100
#define CMD_NoCmdChange         0x07 // 0b0111
#define CMD_Receive             0x08 // 0b1000
#define CMD_Transceive          0x0C // 0b1100
#define CMD_AutoColl            0x0D // 0b1101
#define CMD_MFAuthent           0x0E // 0b1110
#define CMD_SoftResert          0x0F // 0b1111
/*第零页    命令和状态寄存器组*/
#define COMMAND_REG             0x01
#define COMML_EN_REG            0x02
#define DIVL_EN_REG             0x03
#define COM_IRQ_REG             0x04
#define DIV_IRQ_REG             0x05
#define ERROR_REG               0x06
#define STATUS_1_REG            0x07
#define STATUS_2_REG            0x08
#define FIFO_DATA_REG           0x09
#define FIFO_LEVEL_REG          0x0A
#define WATER_LEVEL_REG         0x0B
#define CONTROL_REG             0x0C
#define BIT_FRAMING_REG         0x0D
#define COLL_REG                0x0E
/*第一页    命令寄存器组*/
#define MODE_REG                0x11
#define TX_MODE_REG             0x12
#define RX_MODE_REG             0x13
#define TX_CONTROL_REG          0x14
#define TX_AUTO_REG             0x15
#define TX_SEL_REG              0x16
#define RX_SEL_REG              0x17
#define RX_THRESHOLD_REG        0x18
#define DEMOD_REG               0x19
#define FEL_NFC_1_REG           0x1A
#define FEL_NFC_2_REG           0x1B
#define MIF_NFC_REG             0x1C
#define MANUAL_RCV_REG          0x1D
#define TYPE_B_REG              0x1E
#define SERIAL_SPEED_REG        0x1F
/*第二页*/
#define CRC_RESULT_H_REG        0x21
#define CRC_RESULT_L_REG        0x22
#define GS_N_OFF_REG            0x23
#define MODE_WIDTH_REG          0x24
#define TX_BIT_PHASE_REG        0x25
#define RFC_FG_REG              0x26
#define GS_N_ON_REG             0x27
#define CWGSP_REG               0x28
#define MOD_GS_REG              0x29
#define TMode_REG               0x2A
#define TPrescaler_REG          0x2B
#define T_RELOAD_H_REG          0x2C
#define T_RELOAD_L_REG          0x2D
#define T_COUNTER_VAL_H_REG     0x2E
#define T_COUNTER_VAL_L_REG     0x2F 
/*第三页    测试寄存器*/
#define TestSel1_REG            0x31
#define TestSel2_REG            0x32
#define TestPinEn_REG           0x33
#define TestPinVal_REG          0x34
#define TestBus_REG             0x35
#define AutoTest_REG            0x36
#define Version_REG             0x37
#define AnalogTest_REG          0x38
#define TestDAC1_REG            0x39
#define TestDAC2_REG            0x3A
#define TestADC_REG             0x3B

//错误寄存器位 0x06
#define BIT_WrErr               0x80
#define BIT_TempErr             0x40
#define BIT_BufferOvfl          0x10
#define BIT_CollErr             0x08
#define BIT_CRCErr              0x04
#define BIT_ParityErr           0x02 //奇偶校验失败
/* 如果SOF不正确，在接收器启动阶段自动清除，则设置为逻辑1。在MFAuthent命令期间，该位仅对106kBd有效。如果在一个数据流中接收的字节数不正确，则ProtocolErr位设置为逻辑1 */
#define BIT_ProtocolErr         0x01

/**
 * 下面是一些错误定义
 */

typedef enum {
    RC522_OK,
    RC522_ERR,
    RC522_FAILD,
    RC522_INVALID_ARG,
    RC522_NO_CARD,
    RC522_ANTICOLL_FAILD,
    RC522_UID_NO_MATCH,
    RC522_AUTH_FAILD,
    RC522_CARD_FAILD,
    RC522_CARD_BLOCK_READ_FAILD,
    RC522_CARD_BLOCK_WRITE_FAILD,
} rc522_err_t;



#endif