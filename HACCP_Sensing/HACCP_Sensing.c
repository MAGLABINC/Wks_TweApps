/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"

#include "HACCP_Sensing.h"
#include "config.h"

// DEBUG options
#define SERIAL_DEBUG
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

#include "SMBus.h"
#include "TMP1075.h"

#include "sercmd_gen.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
//#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
//#define ToCoNet_USE_MOD_NBSCAN_SLAVE

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

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

typedef struct
{
    // MAC
    uint8 u8channel;
    uint16 u16addr;

    // LED Counter
    uint32 u32LedCt;
	uint16 u16LedDur_ct; //! LED点灯カウンタ

    // シーケンス番号
    uint32 u32Seq;

    // スリープカウンタ
    uint8 u8SleepCt;
} tsAppData;

#define LED	5
#define SW	12

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
void vSerOutput_PAL(tsRxPktInfo sRxPktInfo, uint8 *p);

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);
void vHandleSerialInput(void);

int16 i16TransmitPingMessage(uint8 *pMsg);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Version/build information. This is not used in the application unless we
   are in serial debug mode. However the 'used' attribute ensures it is
   present in all binary files, allowing easy identifaction... */

/* Local data used by the tag during operation */
static tsAppData sAppData;
static teModuleType sModuleType;

PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

// Wakeup port
#define PORT_SW (1UL << SW)		// SW Port
#define PORT_RX (1UL << 7) 		// UART Rx Port
const uint32 u32DioPortWakeUp = PORT_SW | PORT_RX;

// センサー状況
#define KICKED_SENSOR_TMP1075 1

static uint8 u8KickedSensor; //!< 開始されたセンサーの種類
static uint32 u32KickedTimeStamp; //! 開始されたタイムスタンプ
static int16 tempResult;		//! 温度計測結果

tsSerCmd_Context sSerCmdOut; //!< シリアル出力用

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static bool_t bWakeupBySw;
static bool_t bWakeupByRx;
void cbAppColdStart(bool_t bAfterAhiInit)
{
	//static uint8 u8WkState;
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.
		bWakeupBySw = FALSE;
		bWakeupByRx = FALSE;
		if(u8AHI_WakeTimerFiredStatus()) {
			// wake up timer
		}
		else if(u32AHI_DioWakeStatus() & PORT_RX) {
			bWakeupByRx = TRUE;
		} else {
			bWakeupBySw = TRUE;
		}

		// Register modules
		ToCoNet_REG_MOD_ALL();

	} else {
		// disable brown out detect
		vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
				FALSE,
				FALSE,
				FALSE,
				FALSE);

		// clear application context
		memset (&sAppData, 0x00, sizeof(sAppData));
		sAppData.u8channel = CHANNEL;

		// ToCoNet configuration
		sToCoNet_AppContext.u32AppId = APP_ID;
		sToCoNet_AppContext.u8Channel = CHANNEL;

		sToCoNet_AppContext.bRxOnIdle = TRUE;

		// others
		SPRINTF_vInit128();

		// Register
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// Others
		vInitHardware(FALSE);

		// MAC start
		ToCoNet_vMacStart();

#ifdef SERIAL_DEBUG
		if (bWakeupBySw)
			vfPrintf(&sSerStream, LB "woke up from SW");
		else if (bWakeupByRx)
			vfPrintf(&sSerStream, LB "woke up from RX");
		else
			vfPrintf(&sSerStream, LB "woke up from Timer");
#endif
	}
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbAppWarmStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.

	} else {
		// Initialize hardware
		vInitHardware(TRUE);

		// MAC start
		ToCoNet_vMacStart();

	}
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/**
 * UART形式の出力 (PAL)
 */
void vSerOutput_PAL(tsRxPktInfo sRxPktInfo, uint8 *p) {
	uint8 u8buff[256], *q = u8buff; // 出力バッファ

	// 受信機のアドレス
	S_BE_DWORD(sRxPktInfo.u32addr_rcvr);

	// LQI
	S_OCTET(sRxPktInfo.u8lqi_1st);

	// フレーム
	S_BE_WORD(sRxPktInfo.u16fct);

	// 送信元子機アドレス
	S_BE_DWORD(sRxPktInfo.u32addr_1st);
	S_OCTET(sRxPktInfo.u8id);

	// パケットの種別により処理を変更
	S_OCTET(0x80);
	S_OCTET(sRxPktInfo.u8pkt);

	uint8 u8Length = G_OCTET();
	S_OCTET(u8Length);
	uint8 i = 0;

	while( i<u8Length ){
		uint8 u8Sensor = G_OCTET();

		switch(u8Sensor){
			case HALLIC:
				_C{
					uint8 u8num = G_OCTET();(void)u8num;
					uint8 u8Status = G_OCTET();
					S_OCTET(UNUSE_EXBYTE|TYPE_UNSIGNED|TYPE_CHAR);
					S_OCTET(u8Sensor);
					S_OCTET(0x00);
					S_OCTET(0x01);
					S_OCTET(u8Status);
				}
				break;
			case TEMP:
				_C{
					uint8 u8num = G_OCTET();(void)u8num;
					int16 i16temp = G_BE_WORD();

					if(i16temp == -32767 || i16temp == -32768){
						S_OCTET(ERROR|( (i16temp == -32767)?0x01:0x00 ));
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x00);
					}else{
						S_OCTET(UNUSE_EXBYTE|TYPE_SIGNED|TYPE_SHORT);
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x02);
						S_BE_WORD(i16temp);
					}
				}
				break;
			case HUM:
				_C{
					uint8 u8num = G_OCTET();(void)u8num;
					uint16 u16hum = G_BE_WORD();

					if( u16hum == 0x8001 || u16hum == 0x8000 ){
						S_OCTET(ERROR|( (u16hum == 0x8001)?0x01:0x00 ));
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x00);
					}else{
						S_OCTET(UNUSE_EXBYTE|TYPE_UNSIGNED|TYPE_SHORT);
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x02);
						S_BE_WORD(u16hum);
					}
				}
				break;
			case ILLUM:
				_C{
					uint8 u8num = G_OCTET();(void)u8num;
					uint32 u32illum = G_BE_DWORD();

          	    	if(u32illum == 0xFFFFFFFE || u32illum == 0xFFFFFFFF ){
						S_OCTET(ERROR|((u32illum == 0xFFFFFFFE)?0x01:0x00));
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x00);
					}else{
						S_OCTET(UNUSE_EXBYTE|TYPE_UNSIGNED|TYPE_LONG);
						S_OCTET(u8Sensor);
						S_OCTET(0x00);
						S_OCTET(0x04);
						S_BE_DWORD(u32illum);
					}
				}
				break;

			case ACCEL:
				_C{
					uint8 u8Int = G_OCTET();(void)u8Int;
					uint8 u8Num = G_OCTET();
					uint8 u8Sampling = G_OCTET();
					u8Sampling = (u8Sampling<<5)&0xFF;		// 5bitシフトしておく
					uint8 u8Bit = G_OCTET();(void)u8Bit;

					uint8 j = 0;
					while( j < u8Num ){
						int16 X[2], Y[2], Z[2];

						uint8 tmp = G_OCTET(); X[0] = tmp<<4;
						tmp = G_OCTET(); X[0] |= (tmp>>4); Y[0] = (tmp&0x0F)<<8;
						tmp = G_OCTET(); Y[0] |= tmp;
						tmp = G_OCTET(); Z[0] = tmp<<4;
						tmp = G_OCTET(); Z[0] |= (tmp>>4); X[1] = (tmp&0x0F)<<8;
						tmp = G_OCTET(); X[1] |= tmp;
						tmp = G_OCTET(); Y[1] = tmp<<4;
						tmp = G_OCTET(); Y[1] |= (tmp>>4); Z[1] = (tmp&0x0F)<<8;
                    	tmp = G_OCTET(); Z[1] |= tmp;

						uint8 k;
						for( k=0; k<2; k++ ){
							S_OCTET(USE_EXBYTE|TYPE_SIGNED|TYPE_SHORT);
							S_OCTET(u8Sensor);
							S_OCTET((u8Sampling|(j+k)));
							S_OCTET(0x06);

							// 符号があれば上位4ビットをFで埋める
							X[k] = (X[k]&0x0800) ? (X[k]|0xF000)*8:X[k]*8;
							Y[k] = (Y[k]&0x0800) ? (Y[k]|0xF000)*8:Y[k]*8;
							Z[k] = (Z[k]&0x0800) ? (Z[k]|0xF000)*8:Z[k]*8;
							S_BE_WORD(X[k]);
							S_BE_WORD(Y[k]);
							S_BE_WORD(Z[k]);
						}


						j += 2;
					}
					i += (u8Num-1);
				}
				break;
			case ADC:
				_C{
					uint8 u8num = G_OCTET();
					uint16 u16ADC = 0;
					if(u8num == 0x01 || u8num == 0x08){
						u8num = 0x08;
						uint8 u8Pwr = G_OCTET();
						u16ADC = DECODE_VOLT(u8Pwr);
					}else{
						u8num--;
						u16ADC = G_BE_WORD();
					}
					S_OCTET(USE_EXBYTE|TYPE_UNSIGNED|TYPE_SHORT);
					S_OCTET(u8Sensor);
					S_OCTET(u8num);
					S_OCTET(0x02);
					S_BE_WORD(u16ADC);
				}
				break;
			case DIO:
				_C{
					uint8	u8num = G_OCTET();
					uint32	u32DIO;
					if(u8num <= 8){
						u32DIO = G_OCTET();
						S_OCTET(USE_EXBYTE|TYPE_UNSIGNED|TYPE_CHAR);
					}else if(u8num<=16){
						u32DIO = G_BE_WORD();
						S_OCTET(USE_EXBYTE|TYPE_UNSIGNED|TYPE_SHORT);
					}else{
						u32DIO = G_BE_DWORD();
                    	S_OCTET(USE_EXBYTE|TYPE_UNSIGNED|TYPE_LONG);
					}
					S_OCTET(u8Sensor);
					S_OCTET(u8num);
					if(u8num <= 8){
						S_OCTET(0x01);
						S_OCTET(u32DIO&0xFF);
					}else if(u8num<=16){
						S_OCTET(0x02);
						S_BE_WORD(u32DIO&0xFFFF);
					}else{
						S_OCTET(0x04);
						S_BE_DWORD(u32DIO);
					}				
				}
				break;
			case EEPROM:
				_C{
					uint8 u8num = G_OCTET();
					uint8 u8Status = G_OCTET();
					S_OCTET(0x80|(u8Status&0x7F));
					S_OCTET(u8Sensor);
					S_OCTET(u8num);
					S_OCTET(0x00);
				}
				break;
			default:
				break;
		}

		i++;
	}
	uint8 u8crc = (uint8)u8CCITT8( u8buff, q-u8buff );
	S_OCTET(u8crc);

	sSerCmdOut.u16len = q - u8buff;
	sSerCmdOut.au8data = u8buff;

	// if(!Interactive_bGetMode()) sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);

	sSerCmdOut.au8data = NULL;
}

/****************************************************************************
 *
 * NAME: vMain
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vMain(void)
{
	/* handle uart input */
	vHandleSerialInput();
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vNwkEvent
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch(eEvent) {
	default:
		break;
	}
}

/****************************************************************************
 *
 * NAME: cbvMcRxHandler
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
	tsRxPktInfo sRxPktInfo;

	uint8 *p = pRx->auData;

	// // 暗号化対応時に平文パケットは受信しない
	// if (IS_APPCONF_OPT_SECURE()) {
	// 	if (!pRx->bSecurePkt) {
	// 		return;
	// 	}
	// }

	// パケットの表示
	if (pRx->u8Cmd == TOCONET_PACKET_CMD_APP_DATA) {
		// 基本情報
		sRxPktInfo.u8lqi_1st = pRx->u8Lqi;
		sRxPktInfo.u32addr_1st = pRx->u32SrcAddr;

		// データの解釈
		uint8 u8b = G_OCTET();

		// PALからのパケットかどうかを判定する
		u8b = u8b&0x7F;

		// 違うデータなら表示しない
		if( u8b != 'T' && u8b != 'R' ){
			return;
		}

		// LED の点灯を行う
		sAppData.u16LedDur_ct = 25;

		// 受信機アドレス
		sRxPktInfo.u32addr_rcvr = TOCONET_NWK_ADDR_PARENT;
		if (u8b == 'R') {
			// ルータからの受信
			sRxPktInfo.u32addr_1st = G_BE_DWORD();
			sRxPktInfo.u8lqi_1st = G_OCTET();

			sRxPktInfo.u32addr_rcvr = pRx->u32SrcAddr;
		}

		// ID などの基本情報
		sRxPktInfo.u8id = G_OCTET();
		sRxPktInfo.u16fct = G_BE_WORD();

		// パケットの種別により処理を変更
		sRxPktInfo.u8pkt = G_OCTET();

		// 出力用の関数を呼び出す
		vSerOutput_PAL(sRxPktInfo, p);
	}

	// int i;

	// //!< Master基板（センサなし）
	// if (sModuleType == E_MODULE_MASTER) {
	// 	// print coming payload
	// 	vfPrintf(&sSerStream, LB"[PKT Ad:%04x,Ln:%03d,Seq:%03d,Lq:%03d,Tms:%05d [",
	// 			pRx->u32SrcAddr,
	// 			pRx->u8Len+4, // Actual payload byte: the network layer uses additional 4 bytes.
	// 			pRx->u8Seq,
	// 			pRx->u8Lqi,
	// 			pRx->u32Tick & 0xFFFF);
	// 	for (i = 0; i < pRx->u8Len; i++) {
	// 		vfPrintf(&sSerStream, "%02X", pRx->auData[i]);
	// 	}
	// 	vfPrintf(&sSerStream, "]");
	// }
}

/****************************************************************************
 *
 * NAME: cbvMcEvTxHandler
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	return;
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
		// // LED BLINK
   		// vPortSet_TrueAsLo(LED, u32TickCount_ms & 0x100);

		if (u8KickedSensor && (u32TickCount_ms - u32KickedTimeStamp) < 0x80000000) { // タイムアウトした
			SPRINTF_vRewind();

			if (u8KickedSensor == KICKED_SENSOR_TMP1075) {
				u8KickedSensor = 0;
				tempResult = i16TMP1075readResult();
			}
			u8KickedSensor = 0;
			i16TransmitPingMessage(SPRINTF_pu8GetBuff());
		}

   		//LED ON when receive
   		if (u32TickCount_ms - sAppData.u32LedCt < 300) {
   			vPortSetLo(LED);
   		} else {
  			vPortSetHi(LED);
   		}
		break;

    default:
    	break;
    }
}

int16 i16TransmitPingMessage(uint8 *pMsg) {
	// transmit Ack back
	tsTxDataApp tsTx;
	memset(&tsTx, 0, sizeof(tsTxDataApp));
	uint8 *q = tsTx.auData;

	sAppData.u32Seq++;

	tsTx.u32SrcAddr = ToCoNet_u32GetSerial(); // 自身のアドレス
	tsTx.u32DstAddr = 0xFFFF; // ブロードキャスト

	tsTx.bAckReq = FALSE;
	tsTx.u8Retry = 0;
	tsTx.u8CbId = sAppData.u32Seq & 0xFF;
	tsTx.u8Seq = sAppData.u32Seq & 0xFF;
	tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

	S_OCTET(5);		// データ数

	S_OCTET(0x30);					// 電圧
	S_OCTET(0x01);					// 電源電圧
//	S_OCTET(sAppData.u8Batt);
	S_OCTET(0x80);					// 仮に 3.37V

	S_OCTET(0x30);					// 電圧
	S_OCTET(0x02);					// ADC1(AI1)
//	S_BE_WORD(sAppData.u16Adc[0]);
	S_BE_WORD(1647);				// 仮に 1647

	S_OCTET(0x01);			// Temp
	S_OCTET(0x00);
	S_BE_WORD(tempResult);

	S_OCTET(0x02);			// Hum
	S_OCTET(0x00);
	S_BE_WORD(0);

	S_OCTET(0x03);			// Lux
	S_OCTET(0x00);
	S_BE_DWORD(0);

	tsTx.u8Len = q-tsTx.auData;
	// 送信
	if (ToCoNet_bMacTxReq(&tsTx)) {
		// LEDの制御
		sAppData.u32LedCt = u32TickCount_ms;

		// ＵＡＲＴに出力
		// vfPrintf(&sSerStream, LB "Fire PING Broadcast Message.");

		return tsTx.u8CbId;
	} else {
		return -1;
	}
}

/****************************************************************************
 *
 * NAME: cbToCoNet_u8HwInt
 *
 * DESCRIPTION:
 *   called during an interrupt
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 *                  FALSE -  interrupt is not handled, escalated to further
 *                           event call (cbToCoNet_vHwEvent).
 *                  TRUE  -  interrupt is handled, no further call.
 *
 * NOTES:
 *   Do not put a big job here.
 ****************************************************************************/
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	return FALSE;
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vInitHardware(int f_warm_start)
{
	// Serial Initialize
#if 0
	// UART の細かい設定テスト
	tsUartOpt sUartOpt;
	memset(&sUartOpt, 0, sizeof(tsUartOpt));
	sUartOpt.bHwFlowEnabled = FALSE;
	sUartOpt.bParityEnabled = E_AHI_UART_PARITY_ENABLE;
	sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
	sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
	sUartOpt.u8WordLen = 7;

	vSerialInit(UART_BAUD, &sUartOpt);
#else
	vSerialInit(UART_BAUD, NULL);
#endif


	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(3);

	/// IOs
	vPortSetHi(LED);
	vPortAsOutput(LED);
	vPortAsInput(SW);

	// SMBUS
	vSMBusInit();
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[96];
	static uint8 au8SerialRxBuffer[32];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_SLAVE;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_SLAVE;
}

/****************************************************************************
 *
 * NAME: vHandleSerialInput
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void vHandleSerialInput(void)
{
    // handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

	    SERIAL_vFlush(sSerStream.u8Device);

		//!< Slave基板（センサあり）
		if (i16Char == 't' && sModuleType == E_MODULE_SLAVE) {
			if (!u8KickedSensor) {
				// TMP1075 start
				bool_t bres = bTMP1075startRead();
				if (bres) {
#ifdef SERIAL_DEBUG
					vfPrintf(&sSerStream, LB "sModuleType = E_MODULE_SLAVE");
#endif
					sModuleType = E_MODULE_SLAVE;	//!< Slave基板（センサあり）
					u8KickedSensor = KICKED_SENSOR_TMP1075;
					u32KickedTimeStamp = u32TickCount_ms + TMP1075_CONVTIME;
				} else {
#ifdef SERIAL_DEBUG
					vfPrintf(&sSerStream, LB "sModuleType = E_MODULE_MASTER");
#endif
					sModuleType = E_MODULE_MASTER;  //!< Master基板（センサなし）
				}
			}
		}

		vfPrintf(&sSerStream, LB);
	    SERIAL_vFlush(sSerStream.u8Device);
	}
}

/****************************************************************************
 *
 * NAME: vProcessEvent
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if (eEvent == E_EVENT_START_UP) {
		// ここで UART のメッセージを出力すれば安全である。
		if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
			vfPrintf(&sSerStream, LB "RAMHOLD");
		}
	    if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
			vfPrintf(&sSerStream, LB "Wake up by %s. SleepCt=%d",
					bWakeupByRx ? "UART PORT" : (bWakeupBySw ? "SW PORT" : "WAKE TIMER"),
					sAppData.u8SleepCt);
	    } else {
	    	vfPrintf(&sSerStream, "\r\n*** TWELITE NET PINGPONG SAMPLE %d.%02d-%d ***", VERSION_MAIN, VERSION_SUB, VERSION_VAR);
	    	vfPrintf(&sSerStream, "\r\n*** %08x ***", ToCoNet_u32GetSerial());
			//!< Slave基板（センサあり）
			if (!u8KickedSensor) {
				// SMBUS の初期化
				vSMBusInit();
				// TMP1075 start
				bool_t bres = bTMP1075startRead();
				if (bres) {
#ifdef SERIAL_DEBUG
					vfPrintf(&sSerStream, LB "sModuleType = E_MODULE_SLAVE");
#endif
					sModuleType = E_MODULE_SLAVE;	//!< Slave基板（センサあり）
					u8KickedSensor = KICKED_SENSOR_TMP1075;
					u32KickedTimeStamp = u32TickCount_ms + TMP1075_CONVTIME;
				} else {
#ifdef SERIAL_DEBUG
					vfPrintf(&sSerStream, LB "sModuleType = E_MODULE_MASTER");
#endif
					sModuleType = E_MODULE_MASTER;  //!< Master基板（センサなし）
				}
			}
	    }
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
