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

#include "Parent.h"
#include "config.h"

#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

#include "btnMgr.h"

#include "Interactive.h"
#include "sercmd_gen.h"

#include "common.h"
#include "AddrKeyAry.h"
#include "ccitt8.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
#define ToCoNet_USE_MOD_NWK_LAYERTREE // Network definition
#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
#define ToCoNet_USE_MOD_NBSCAN_SLAVE // Neighbour scan slave module
//#define ToCoNet_USE_MOD_CHANNEL_MGR
#define ToCoNet_USE_MOD_NWK_MESSAGE_POOL
#define ToCoNet_USE_MOD_DUPCHK

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define TOCONET_DEBUG_LEVEL 0

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vInitHardware(int f_warm_start);
static void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);

void vSerOutput_PAL(tsRxPktInfo sRxPktInfo, uint8 *p);
void vSerOutput_Tag(tsRxPktInfo sRxPktInfo, uint8 *p);

void vSerInitMessage();
void vProcessSerialCmd(tsSerCmd_Context *pCmd);

void vLED_Toggle( void );

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsAppData_Pa sAppData; // application information
tsFILE sSerStream; // serial output context
tsSerialPortSetup sSerPort; // serial port queue

tsSerCmd_Context sSerCmdOut; //!< シリアル出力用

tsAdrKeyA_Context sEndDevList; // 子機の発報情報を保存するデータベース

static uint32 u32TempCount_ms = 0;

#ifdef USE_MONOSTICK
static bool_t bVwd = FALSE;
#endif

/****************************************************************************/
/***        ToCoNet Callback Functions                                    ***/
/****************************************************************************/
/**
 * アプリケーションの起動時の処理
 * - ネットワークの設定
 * - ハードウェアの初期化
 */
void cbAppColdStart(bool_t bAfterAhiInit) {
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.

		// Register modules
		ToCoNet_REG_MOD_ALL();

	} else {
		// disable brown out detect
		vAHI_BrownOutConfigure(0, //0:2.0V 1:2.3V
				FALSE, FALSE, FALSE, FALSE);
		
		sAppData.u32DIO_startup = ~u32PortReadBitmap(); // この時点では全部入力ポート

		// clear application context
		memset(&sAppData, 0x00, sizeof(sAppData));
		ADDRKEYA_vInit(&sEndDevList);
		SPRINTF_vInit128();

		// フラッシュメモリからの読み出し
		//   フラッシュからの読み込みが失敗した場合、ID=15 で設定する
		sAppData.bFlashLoaded = Config_bLoad(&sAppData.sFlash);

		// ToCoNet configuration
		sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;
		sToCoNet_AppContext.u8Channel = sAppData.sFlash.sData.u8ch;
		sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;

		sToCoNet_AppContext.u8TxPower = sAppData.sFlash.sData.u8pow&0x0F;

		sToCoNet_AppContext.bRxOnIdle = TRUE;
		sToCoNet_AppContext.u8TxMacRetry = 1;

		// Register
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// Others
		vInitHardware(FALSE);
		Interactive_vInit();

		// シリアルの書式出力のため
		if (IS_APPCONF_OPT_UART_BIN()) {
			SerCmdBinary_vInit(&sSerCmdOut, NULL, 128); // バッファを指定せず初期化
		} else {
			SerCmdAscii_vInit(&sSerCmdOut, NULL, 128); // バッファを指定せず初期化
		}
	}
}

/**
 * スリープ復帰時の処理（本アプリケーションでは処理しない)
 * @param bAfterAhiInit
 */
void cbAppWarmStart(bool_t bAfterAhiInit) {
	cbAppColdStart(bAfterAhiInit);
}

/**
 * メイン処理
 * - シリアルポートの処理
 */
void cbToCoNet_vMain(void) {
	/* handle uart input */
	vHandleSerialInput();
}

/**
 * ネットワークイベント。
 * - E_EVENT_TOCONET_NWK_START\n
 *   ネットワーク開始時のイベントを vProcessEvCore に伝達
 *
 * @param eEvent
 * @param u32arg
 */
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch (eEvent) {
	case E_EVENT_TOCONET_NWK_START:
		// send this event to the local event machine.
		ToCoNet_Event_Process(eEvent, u32arg, vProcessEvCore);
		break;
	default:
		break;
	}
}

/**
 * 子機または中継機を経由したデータを受信する。
 *
 * - アドレスを取り出して、内部のデータベースへ登録（メッセージプール送信用）
 * - UART に指定書式で出力する
 *   - 出力書式\n
 *     ::(受信元ルータまたは親機のアドレス):(シーケンス番号):(送信元アドレス):(LQI)<CR><LF>
 *
 * @param pRx 受信データ構造体
 */
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
	tsRxPktInfo sRxPktInfo;

	uint8 *p = pRx->auData;

	// 暗号化対応時に平文パケットは受信しない
	if (IS_APPCONF_OPT_SECURE()) {
		if (!pRx->bSecurePkt) {
			return;
		}
	}

	// パケットの表示
	if (pRx->u8Cmd == TOCONET_PACKET_CMD_APP_DATA) {
		// 基本情報
		sRxPktInfo.u8lqi_1st = pRx->u8Lqi;
		sRxPktInfo.u32addr_1st = pRx->u32SrcAddr;

		// データの解釈
		uint8 u8b = G_OCTET();

		// PALからのパケットかどうかを判定する
		bool_t bPAL = u8b&0x80 ? TRUE:FALSE;
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
		if(bPAL){
			vSerOutput_PAL(sRxPktInfo, p);
		}else{
			vSerOutput_Tag(sRxPktInfo, p);
		}

		// データベースへ登録（線形配列に格納している）
		ADDRKEYA_vAdd(&sEndDevList, sRxPktInfo.u32addr_1st, 0); // アドレスだけ登録。
	}
}

/**
 * 送信完了時のイベント
 * @param u8CbId
 * @param bStatus
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	return;
}

/**
 * ハードウェア割り込みの遅延実行部
 *
 * - BTM による IO の入力状態をチェック\n
 *   ※ 本サンプルでは特別な使用はしていない
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {

	switch (u32DeviceId) {
	case E_AHI_DEVICE_TICK_TIMER:
		// LED の点灯消灯を制御する
		if (sAppData.u16LedDur_ct) {
			sAppData.u16LedDur_ct--;
			vAHI_DoSetDataOut( 0, 0x01<<1 );
		} else {
			vAHI_DoSetDataOut( 0x01<<1, 0 );
		}

#ifdef USE_MONOSTICK
		bVwd = !bVwd;
		vPortSet_TrueAsLo(9, bVwd);
#endif
		break;

	default:
		break;
	}

}

/**
 * ハードウェア割り込み
 * - 処理なし
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 * @return
 */
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	return FALSE;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/**
 * ハードウェアの初期化
 * @param f_warm_start
 */
static void vInitHardware(int f_warm_start) {
	// BAUD ピンが GND になっている場合、かつフラッシュの設定が有効な場合は、設定値を採用する (v1.0.3)
	tsUartOpt sUartOpt;
	memset(&sUartOpt, 0, sizeof(tsUartOpt));
	uint32 u32baud = UART_BAUD;
//	if (sAppData.bFlashLoaded && bPortRead(PORT_BAUD)) {
	if (sAppData.bFlashLoaded && IS_APPCONF_OPT_UART_FORCE_SETTINGS() ) {
		u32baud = sAppData.sFlash.sData.u32baud_safe;
		sUartOpt.bHwFlowEnabled = FALSE;
		sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
		sUartOpt.u8ParityType = UART_PARITY_TYPE;
		sUartOpt.u8StopBit = UART_STOPBITS;

		// 設定されている場合は、設定値を採用する (v1.0.3)
		switch(sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_PARITY_MASK) {
		case 0:
			sUartOpt.bParityEnabled = FALSE;
			break;
		case 1:
			sUartOpt.bParityEnabled = TRUE;
			sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
			break;
		case 2:
			sUartOpt.bParityEnabled = TRUE;
			sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
			break;
		}

		// ストップビット
		if (sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_STOPBIT_MASK) {
			sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
		} else {
			sUartOpt.u8StopBit = E_AHI_UART_1_STOP_BIT;
		}

		// 7bitモード
		if (sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_WORDLEN_MASK) {
			sUartOpt.u8WordLen = 7;
		} else {
			sUartOpt.u8WordLen = 8;
		}

		vSerialInit(u32baud, &sUartOpt);
	} else {
		vSerialInit(u32baud, NULL);
	}

	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(TOCONET_DEBUG_LEVEL);

	bAHI_DoEnableOutputs(TRUE);
	vAHI_DoSetDataOut( 0x01<<1, 0 );

	vPortSetHi(OUTPUT_LED);
	vPortAsOutput(OUTPUT_LED);			// DIO11を出力として使用する。

#ifdef USE_MONOSTICK
	vPortSetLo(11);				// 外部のウォッチドッグを有効にする。
	vPortSet_TrueAsLo(9, bVwd);	// VWDをいったんHiにする。
	vPortAsOutput(11);			// DIO11を出力として使用する。
	vPortAsOutput(9);			// DIO9を出力として使用する。
#endif

}

/**
 * UART の初期化
 */
static void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[1532];
	static uint8 au8SerialRxBuffer[512];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
//	sSerPort.u32BaudRate = UART_BAUD;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT;
}


/**
 * アプリケーション主要処理
 * - E_STATE_IDLE\n
 *   ネットワークの初期化、開始
 *
 * - E_STATE_RUNNING\n
 *   - データベースのタイムアウト処理
 *   - 定期的なメッセージプール送信
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			vSerInitMessage();

			V_PRINTF(LB"[E_STATE_IDLE]");

			if (IS_APPCONF_OPT_SECURE()) {
				bool_t bRes = bRegAesKey(sAppData.sFlash.sData.u32EncKey);
				V_PRINTF(LB "*** Register AES key (%d) ***", bRes);
			}

			// Configure the Network
			sAppData.sNwkLayerTreeConfig.u8Layer = 0;
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_PARENT;

#ifndef OLDNET
			sAppData.sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON;
			sAppData.sNwkLayerTreeConfig.u8Second_To_Beacon = TOCONET_MOD_LAYERTREE_DEFAULT_BEACON_DUR;
#endif
			sAppData.pContextNwk =
					ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
			if (sAppData.pContextNwk) {
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}

		} else if (eEvent == E_EVENT_TOCONET_NWK_START) {
			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		} else {
			;
		}
		break;

	case E_STATE_RUNNING:
		if (eEvent == E_EVENT_NEW_STATE) {
			V_PRINTF(LB"[E_STATE_RUNNING]");
		} else if (eEvent == E_EVENT_TICK_SECOND) {
			// 定期クリーン（タイムアウトしたノードを削除する）
			ADDRKEYA_bFind(&sEndDevList, 0, 0);
		} else {
			;
		}
		break;

	default:
		break;
	}
}

/**
 * 初期化メッセージ
 */
void vSerInitMessage() {
	A_PRINTF(LB LB"!INF MONO WIRELESS APP_PAL(Parent) V%d-%02d-%d", VERSION_MAIN, VERSION_SUB, VERSION_VAR);
	A_PRINTF(LB"!INF AID:%08x, SID:%08x, SADD:%04x", 
			sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(), sToCoNet_AppContext.u16ShortAddress);
	A_PRINTF(LB"!INF DIO --> %020b", sAppData.u32DIO_startup);
	if (sAppData.bFlashLoaded == 0) {
		A_PRINTF(LB"!INF Default config (no save info)...");
	}
	A_PRINTF(LB);
}

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
	uint8 u8crc = u8CCITT8( u8buff, q-u8buff );
	S_OCTET(u8crc);

	sSerCmdOut.u16len = q - u8buff;
	sSerCmdOut.au8data = u8buff;

	if(!Interactive_bGetMode()) sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);

	sSerCmdOut.au8data = NULL;
}

/**
 * UART形式の出力 (TAG)
 */
void vSerOutput_Tag(tsRxPktInfo sRxPktInfo, uint8 *p) {
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
	S_OCTET(sRxPktInfo.u8pkt);

	switch(sRxPktInfo.u8pkt) {
	//	温度センサなど
	case PKT_ID_STANDARD:
	case PKT_ID_LM61:
	case PKT_ID_SHT21:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			int16	i16temp = G_BE_WORD();
			uint16	u16humi = G_BE_WORD();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(i16temp);
			S_BE_WORD(u16humi);
		}

		if (sRxPktInfo.u8pkt == PKT_ID_LM61) {
			int16	bias = G_BE_WORD();
			S_BE_WORD( bias );
		}
		break;

	case PKT_ID_MAX31855:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			int32	i32temp = G_BE_DWORD();
			int32	i32itemp = G_BE_DWORD();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(i32temp);
			S_BE_WORD(i32itemp);
		}
		break;

	case PKT_ID_ADT7410:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			int16	i16temp = G_BE_WORD();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(i16temp);
		}
		break;

	case PKT_ID_BME280:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			int16	i16temp = G_BE_WORD();
			uint16	u16hum = G_BE_WORD();
			uint16	u16atmo = G_BE_WORD();

			S_OCTET(u8batt);		// batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(i16temp);		//	Result
			S_BE_WORD(u16hum);		//	Result
			S_BE_WORD(u16atmo);		//	Result
		}
		break;

	case PKT_ID_MPL115A2:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			uint16	u16atmo = G_BE_WORD();

			S_OCTET(u8batt);		// batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(u16atmo);		//	Result
		}
		break;

	case PKT_ID_LIS3DH:
	case PKT_ID_ADXL345:
	case PKT_ID_L3GD20:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			int16	i16x = G_BE_WORD();
			int16	i16y = G_BE_WORD();
			int16	i16z = G_BE_WORD();

			uint8 u8ActTapSource = ( u16adc0>>12 )|((u16adc1>>8)&0xF0);(void)u8ActTapSource;

			u16adc0 = u16adc0&0x0FFF;
			u16adc1 = u16adc1&0x0FFF;

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);

			if( sRxPktInfo.u8pkt == PKT_ID_ADXL345 ){
				uint8 u8mode = G_OCTET();
				S_OCTET(u8mode);

				if(u8mode == 0xfa){
					uint8 u8num = G_OCTET();
					S_OCTET(u8num);
					S_BE_WORD(i16x);		//	1回目は先に表示
					S_BE_WORD(i16y);		//
					S_BE_WORD(i16z);		//
					uint8 i;
					for( i=0; i<u8num-1; i++ ){
						i16x = G_BE_WORD();
						i16y = G_BE_WORD();
						i16z = G_BE_WORD();
						S_BE_WORD(i16x);		//	Result
						S_BE_WORD(i16y);		//	Result
						S_BE_WORD(i16z);		//	Result
					}
				}else if(u8mode == 0xF9 ){
					uint16 u16Sample = G_BE_WORD();
					S_BE_WORD(i16x);		//	average
					S_BE_WORD(i16y);		//
					S_BE_WORD(i16z);		//
					i16x = G_BE_WORD();
					i16y = G_BE_WORD();
					i16z = G_BE_WORD();
					S_BE_WORD(i16x);		//	minimum
					S_BE_WORD(i16y);		//
					S_BE_WORD(i16z);		//
					i16x = G_BE_WORD();
					i16y = G_BE_WORD();
					i16z = G_BE_WORD();
					S_BE_WORD(i16x);		//	maximum
					S_BE_WORD(i16y);		//
					S_BE_WORD(i16z);		//
					S_BE_WORD(u16Sample);	// 今回使用したサンプル数
				}else{
					S_BE_WORD(i16x);		//	Result
					S_BE_WORD(i16y);		//	Result
					S_BE_WORD(i16z);		//	Result
				}
			}else{
				S_BE_WORD(i16x);		//	Result
				S_BE_WORD(i16y);		//	Result
				S_BE_WORD(i16z);		//	Result
			}
		}
		break;

	case PKT_ID_TSL2561:
		_C {
			uint8 u8batt = G_OCTET();

			uint16	u16adc1 = G_BE_WORD();
			uint16	u16adc2 = G_BE_WORD();
			uint32	u32lux = G_BE_DWORD();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc1);
			S_BE_WORD(u16adc2);
			S_BE_DWORD(u32lux);		//	Result
		}
		break;

	case PKT_ID_S1105902:
		_C {
			uint8 u8batt = G_OCTET();

			uint16 u16adc1 = G_BE_WORD();
			uint16 u16adc2 = G_BE_WORD();
			int16 u16R = G_BE_WORD();
			int16 u16G = G_BE_WORD();
			int16 u16B = G_BE_WORD();
			int16 u16I = G_BE_WORD();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc1);
			S_BE_WORD(u16adc2);
			S_BE_WORD(u16R);		//	Result
			S_BE_WORD(u16G);		//	Result
			S_BE_WORD(u16B);
			S_BE_WORD(u16I);
		}
		break;

	case PKT_ID_ADXL362:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			uint16 u16bitmap = G_BE_WORD();
			uint8 u8Interrupt = G_OCTET();
			uint8 u8num = G_OCTET();
			uint8 u8Freq = G_OCTET();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_BE_WORD(u16bitmap);
			S_OCTET(u8Interrupt);
			S_OCTET(u8num);
			S_OCTET(u8Freq);

			uint8 i;
			int16 i16x,i16y,i16z;
			if( u16bitmap&0x10 ){
				uint16 SampleNum = G_BE_WORD();
				S_BE_WORD(SampleNum);

				int16 min,ave,max;
				uint32 dave;
				for( i=0; i<3; i++ ){
					min = G_BE_WORD();
					ave = G_BE_WORD();
					dave = G_BE_DWORD();
					max = G_BE_WORD();
					S_BE_WORD(min);
					S_BE_WORD(ave);
					S_BE_DWORD(dave);
					S_BE_WORD(max);
				}
			}else{
				if( u16bitmap&0x01 ){
					uint8 au8accel[9];
					for( i=0; i<u8num; i+=2 ){
						uint8 j;
						for( j=0; j<9; j++ ){
							au8accel[j] = G_OCTET();
						}
						i16x = (au8accel[0]<<4) + (au8accel[1]>>4);
						if( i16x&0x0800 ) i16x = (int16)(i16x|0xF000);
						i16y = ((au8accel[1]&0x0F)<<8) + au8accel[2];
						if( i16y&0x0800 ) i16y = (int16)(i16y|0xF000);
						i16z = (au8accel[3]<<4) + (au8accel[4]>>4);
						if( i16z&0x0800 ) i16z = (int16)(i16z|0xF000);
						S_BE_WORD(i16x*4);		//	Result
						S_BE_WORD(i16y*4);		//	Result
						S_BE_WORD(i16z*4);		//	Result

						i16x = ((au8accel[4]&0x0F)<<8) + au8accel[5];
						if( i16x&0x0800 ) i16x = (int16)(i16x|0xF000);
						i16y = (au8accel[6]<<4) + (au8accel[7]>>4);
						if( i16y&0x0800 ) i16y = (int16)(i16y|0xF000);
						i16z = ((au8accel[7]&0x0F)<<8) + au8accel[8];
						if( i16z&0x0800 ) i16z = (int16)(i16z|0xF000);
						S_BE_WORD(i16x*4);		//	Result
						S_BE_WORD(i16y*4);		//	Result
						S_BE_WORD(i16z*4);		//	Result
					}
				}else{
					for( i=0; i<u8num; i++ ){
						if( u16bitmap&0x02 ){
							i16x = G_OCTET();
							i16y = G_OCTET();
							i16z = G_OCTET();

							i16x = (i16x&0x0080) ? (i16x|0xFF00)<<6:i16x<<6;
							i16y = (i16y&0x0080) ? (i16y|0xFF00)<<6:i16y<<6;
							i16z = (i16z&0x0080) ? (i16z|0xFF00)<<6:i16z<<6;
						}else{
							i16x = G_BE_WORD();
							i16y = G_BE_WORD();
							i16z = G_BE_WORD();
						}
						S_BE_WORD(i16x);		//	Result
						S_BE_WORD(i16y);		//	Result
						S_BE_WORD(i16z);		//	Result
					}
				}
			}
		}
		break;

	//	磁気スイッチ
	case PKT_ID_IO_TIMER:
		_C {
			uint8	u8batt = G_OCTET();
			uint8	u8stat = G_OCTET();
			uint32	u32dur = G_BE_DWORD();

			S_OCTET(u8batt); // batt
			S_OCTET(u8stat); // stat
			S_BE_DWORD(u32dur); // dur
		}
		break;

	case PKT_ID_UART:
		_C {
			uint8 u8len = G_OCTET();
			S_OCTET(u8len);

			uint8	tmp;
			while (u8len--) {
				tmp = G_OCTET();
				S_OCTET(tmp);
			}
		}
		break;

	//	押しボタン
	case PKT_ID_BUTTON:
		_C {
			uint8	u8batt = G_OCTET();
			uint16	u16adc0 = G_BE_WORD();
			uint16	u16adc1 = G_BE_WORD();
			uint8	u8mode = G_OCTET();
			uint8	u8bitmap = G_OCTET();

			bool_t bRegularTransmit = ((u8mode&0x80)>>7);

			if( !bRegularTransmit ){
				if( (u8mode&0x04) || (u8mode&0x02) ){
					if(u8bitmap){
						vPortSetLo(OUTPUT_LED);
						sAppData.u8DO_State = 1;
					}else{
						vPortSetLo(OUTPUT_LED);
						sAppData.u8DO_State = 0;
						sAppData.u32LedCt = 0;
					}
				}else{
					vLED_Toggle();
				}
			}

			S_OCTET(u8batt);		// batt
			S_BE_WORD(u16adc0);
			S_BE_WORD(u16adc1);
			S_OCTET( u8mode );
			S_OCTET( u8bitmap );
			S_OCTET( sAppData.u8DO_State );
		}
		break;

	case PKT_ID_MULTISENSOR:
		_C {
			uint8 u8batt = G_OCTET();
			uint16 u16adc1 = G_BE_WORD();
			uint16 u16adc2 = G_BE_WORD();
			uint8 u8SnsNum = G_OCTET();

			S_OCTET(u8batt); // batt
			S_BE_WORD(u16adc1);
			S_BE_WORD(u16adc2);
			S_OCTET(u8SnsNum);

			uint8 u8Sensor, i;
			for( i=0; i<u8SnsNum; i++ ){
				u8Sensor = G_OCTET();
				S_OCTET(u8Sensor); // batt
				switch( u8Sensor){
					case PKT_ID_SHT21:
					{
						int16 i16temp = G_BE_WORD();
						int16 i16humd = G_BE_WORD();
						S_BE_WORD(i16temp);
						S_BE_WORD(i16humd);
					}
					break;
					case PKT_ID_ADT7410:
					{
						int16 i16temp = G_BE_WORD();
						S_BE_WORD(i16temp);
					}
					break;
					case PKT_ID_MPL115A2:
					{
						int16 i16atmo = G_BE_WORD();
						S_BE_WORD(i16atmo);
					}
					break;
					case PKT_ID_LIS3DH:
					case PKT_ID_L3GD20:
					{
						int16 i16x = G_BE_WORD();
						int16 i16y = G_BE_WORD();
						int16 i16z = G_BE_WORD();
						S_BE_WORD(i16x);
						S_BE_WORD(i16y);
						S_BE_WORD(i16z);
					}
					break;
					case PKT_ID_ADXL345:
					{
						uint8 u8mode = G_OCTET();
						S_OCTET(u8mode);
						int16 i16x, i16y, i16z;
						if( u8mode == 0xfa ){
							uint8 u8num = G_OCTET();
							uint8 j;
							for( j=0; j<u8num; j++ ){
								i16x = G_BE_WORD();
								i16y = G_BE_WORD();
								i16z = G_BE_WORD();
								S_BE_WORD(i16x);
								S_BE_WORD(i16y);
								S_BE_WORD(i16z);
							}
						}else{
							i16x = G_BE_WORD();
							i16y = G_BE_WORD();
							i16z = G_BE_WORD();
							S_BE_WORD(i16x);
							S_BE_WORD(i16y);
							S_BE_WORD(i16z);
						}
					}
					break;
					case PKT_ID_TSL2561:
					{
						uint32	u32lux = G_BE_DWORD();
						S_BE_DWORD(u32lux);
					}
					break;
					case PKT_ID_S1105902:
					{
						int16 u16R = G_BE_WORD();
						int16 u16G = G_BE_WORD();
						int16 u16B = G_BE_WORD();
						int16 u16I = G_BE_WORD();
						S_BE_WORD(u16R);
						S_BE_WORD(u16G);
						S_BE_WORD(u16B);
						S_BE_WORD(u16I);
					}
					break;
					case PKT_ID_BME280:
					{
						int16	i16temp = G_BE_WORD();
						uint16	u16hum = G_BE_WORD();
						uint16	u16atmo = G_BE_WORD();
						S_BE_WORD(i16temp);
						S_BE_WORD(u16hum);
						S_BE_WORD(u16atmo);
					}
					break;
					default:
						break;
				}
			}
		}
		break;
	default:
		break;
	}

	sSerCmdOut.u16len = q - u8buff;
	sSerCmdOut.au8data = u8buff;

	if(!Interactive_bGetMode()) sSerCmdOut.vOutput(&sSerCmdOut, &sSerStream);

	sSerCmdOut.au8data = NULL;
}

/**
 * コマンド受け取り時の処理
 * @param pCmd
 */
void vProcessSerialCmd(tsSerCmd_Context *pCmd) {
	V_PRINTF(LB "! cmd len=%d data=", pCmd->u16len);
	int i;
	for (i = 0; i < pCmd->u16len && i < 8; i++) {
		V_PRINTF("%02X", pCmd->au8data[i]);
	}
	if (i < pCmd->u16len) {
		V_PRINTF("...");
	}

	return;
}

/**
 * DO1をトグル動作させる
 */
void vLED_Toggle( void )
{
	if( u32TickCount_ms-u32TempCount_ms > 500 ||	//	前回切り替わってから500ms以上たっていた場合
		u32TempCount_ms == 0 ){						//	初めてここに入った場合( u32TickTimer_msが前回切り替わった場合はごめんなさい )
		sAppData.u8DO_State = !sAppData.u8DO_State;
		//	DO1のLEDがトグル動作する
		vPortSet_TrueAsLo( OUTPUT_LED, sAppData.u8DO_State );
		u32TempCount_ms = u32TickCount_ms;
	}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
