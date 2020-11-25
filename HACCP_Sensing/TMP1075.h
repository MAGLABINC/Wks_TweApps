/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

#ifndef  HUM_TMP1075_INCLUDED
#define  HUM_TMP1075_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "sensor_driver.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define TMP1075_ADDRESS     (0x48)

#define TMP1075_WRITE_REG   (0x01)
#define TMP1075_READ_REG    (0x00)

#define TMP1075_STARTUP_H	(0x80)
#define TMP1075_STARTUP_L	(0xFF)

#define TMP1075_START_H		(0x80)
#define TMP1075_START_L		(0xFF)

#define TMP1075_CONVTIME	(26) // 26ms

#define TMP1075_DATA_NOTYET  (-32768)
#define TMP1075_DATA_ERROR   (-32767)

#define TMP1075_TRIG_TEMP   (0xf3)
#define TMP1075_TRIG_HUMID  (0xf5)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions (state machine)                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions (primitive funcs)                          ***/
/****************************************************************************/
PUBLIC bool_t bTMP1075reset();
PUBLIC bool_t bTMP1075startRead();
PUBLIC int16 i16TMP1075readResult();

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif  /* HUM_TMP1075_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

