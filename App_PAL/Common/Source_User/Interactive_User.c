/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

// 本ファイルは Interactive.c から include される

/**
 * フラッシュ設定構造体をデフォルトに巻き戻します。
 * - ここでシステムのデフォルト値を決めています。
 *
 * @param p 構造体へのアドレス
 */
#include "config.h"

static void Config_vSetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;

#ifdef ENDDEVICE
	p->u8pow = 0x13;
#else
	p->u8pow = 0x3;
#endif
	p->u8id = 0;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0;

#ifdef ENDDEVICE
	p->u16RcClock = 0;
	p->u32Slp = DEFAULT_SLEEP_DUR;
	p->u32param = 0;
#endif

	p->u32Opt = 1; // デフォルトの設定ビット

#ifdef ROUTER
	p->u8layer = 4; // Layer:1
	p->u32AddrHigherLayer = 0; // 指定送信先なし
#endif

	p->u32EncKey = DEFAULT_ENC_KEY;
}
