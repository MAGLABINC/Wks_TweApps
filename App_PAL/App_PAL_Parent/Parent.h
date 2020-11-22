/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#ifndef  SLAVE_H_INCLUDED
#define  SLAVE_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "config.h"
#include "appdata.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define HALLIC	0x00
#define TEMP	0x01
#define HUM		0x02
#define ILLUM	0x03
#define ACCEL	0x04

#define ADC		0x30
#define DIO		0x31
#define EEPROM	0x32

#define TYPE_CHAR		0x00
#define TYPE_SHORT		0x01
#define TYPE_LONG		0x02
#define TYPE_VARIABLE	0x03

#define TYPE_SIGNED		0x04
#define TYPE_UNSIGNED	0x00

#define USE_EXBYTE		0x10
#define UNUSE_EXBYTE	0x00

#define ERROR			0x80


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
//	受信したパケットから取得した基本的な情報
typedef struct _tsRxPktInfo{
	uint8	u8lqi_1st;		//	LQI
	uint32	u32addr_1st;	//	アドレス
	uint32	u32addr_rcvr;	//	受信機アドレス
	uint8	u8id;			//	ID
	uint16	u16fct;			//	FCT
	uint8	u8pkt;			//	子機のセンサモード
} tsRxPktInfo;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* SLAVE_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
