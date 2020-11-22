/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */


#ifndef COMMON_H_
#define COMMON_H_

#include <serial.h>
#include <fprintf.h>

#include "ToCoNet.h"

void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep);
void vResetWithMsg(tsFILE *psSerStream, string str);

bool_t bTransmitToParent(tsToCoNet_Nwk_Context *pNwk, uint8 *pu8Data, uint8 u8Len);

bool_t bRegAesKey(uint32 u32seed);

#ifdef ENDDEVICE
bool_t bGetPALOptions( void );
#endif

extern const uint8 au8EncKey[];
extern uint32 u32DioPortWakeUp;

/*
 * パケット識別子
 */
#define PKT_ID_NOCONNECT 0x00
#define PKT_ID_MAG 0x01
#define PKT_ID_AMB 0x02
#define PKT_ID_MOT 0x03
#define PKT_ID_IRC 0x04

/*
 * パケット識別子 (App_Tag)
 */
#define PKT_ID_STANDARD 0x10
#define PKT_ID_LM61 0x11
#define PKT_ID_SHT21 0x31
#define PKT_ID_ADT7410 0x32
#define PKT_ID_MPL115A2 0x33
#define PKT_ID_LIS3DH 0x34
#define PKT_ID_ADXL345 0x35
#define PKT_ID_TSL2561 0x36
#define PKT_ID_L3GD20 0x37
#define PKT_ID_S1105902 0x38
#define PKT_ID_BME280 0x39
#define PKT_ID_IO_TIMER 0x51
#define PKT_ID_MAX31855 0x61
#define PKT_ID_ADXL362 0x62
#define PKT_ID_UART 0x81
#define PKT_ID_ADXL345_LOWENERGY 0xA1
#define PKT_ID_MULTISENSOR 0xD1
#define PKT_ID_BUTTON 0xFE

/*
 * 標準ポート定義 (TWELITE PAL)
 */
// 子機用配置
//#warning "IO CONF IS FOR ENDDEVICE"
#define INPUT_DIP1 1
#define INPUT_DIP2 2
#define INPUT_DIP3 3
#define INPUT_DIP4 4

#define EH_BOOT 4

#ifdef USE_MONOSTICK
#define OUTPUT_LED 16
#else
#define OUTPUT_LED 5
#endif

#define INPUT_D0 0
#define INPUT_D8 8
#define INPUT_PC 8

#define CLK_IN 9
#define CLK_EN 10

#define CLD_IN 11
#define INPUT_SWSET 12
#define INPUT_SW 12
#define INPUT_SET 12
#define WDT_OUT 13

#define SNS_EN 16
#define SNS_INT 17

#endif /* COMMON_H_ */