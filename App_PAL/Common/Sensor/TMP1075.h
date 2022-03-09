#ifndef  TMP1075_H
#define  TMP1075_H

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "sensor_driver.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct {
	// data
	int16 ai16Result;

	// working
	uint8 u8TickCount, u8TickWait;
	uint8 u8IdxMeasuruing;
} tsObjData_TMP1075;

/****************************************************************************/
/***        Exported Functions (state machine)                            ***/
/****************************************************************************/
void vTMP1075_Init(tsObjData_TMP1075 *pData, tsSnsObj *pSnsObj);
void vTMP1075_Final(tsObjData_TMP1075 *pData, tsSnsObj *pSnsObj);

#define i16TMP1075_GetTemp(pSnsObj) ((tsObjData_TMP1075 *)(pSnsObj->pData)->ai16Result)

#define TMP1075_DATA_NOTYET	(-32768)
#define TMP1075_DATA_ERROR	(-32767)

#define DEV_POWER_EN() vPortSetHi(DEVPWR_EN)
#define DEV_POWER_DI() vPortSetLo(DEVPWR_EN)

/****************************************************************************/
/***        Exported Functions (primitive funcs)                          ***/
/****************************************************************************/
PUBLIC bool_t bTMP1075reset();
PUBLIC bool_t bTMP1075startRead();
PUBLIC int16 i16TMP1075readResult(int16*);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif  /* TMP1075_H */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

