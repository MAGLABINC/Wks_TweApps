/* Copyright (C) 2017 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT).                                                   */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "AppHardwareApi.h"
#include "string.h"

#include "sensor_driver.h"
#include "SHTC3.h"
#include "SMBus.h"

#include "ccitt8.h"

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
# include <serial.h>
# include <fprintf.h>
PUBLIC tsFILE sDebugStream;
#endif

#include <fprintf.h>		// ********** for Debugging **********
#include "utils.h"
#include "Interactive.h"
extern tsFILE sSerStream;

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

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessSnsObj_SHTC3(void *pvObj, teEvent eEvent);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
void vSHTC3_Init(tsObjData_SHTC3 *pData, tsSnsObj *pSnsObj) {
	vSnsObj_Init(pSnsObj);

	pSnsObj->pvData = (void*)pData;
	pSnsObj->pvProcessSnsObj = (void*)vProcessSnsObj_SHTC3;

	memset((void*)pData, 0, sizeof(tsObjData_SHTC3));

	bSHTC3wakeup();
	vWait(600);
}

void vSHTC3_Final(tsObjData_SHTC3 *pData, tsSnsObj *pSnsObj) {
	pSnsObj->u8State = E_SNSOBJ_STATE_INACTIVE;
}

/****************************************************************************
 *
 * NAME: vSHTC3reset
 *
 * DESCRIPTION:
 *   to reset SHTC3 device
 *
 * RETURNS:
 * bool_t	fail or success
 *
 ****************************************************************************/
PUBLIC bool_t bSHTC3reset()
{
	bool_t bOk = TRUE;
	uint8 u8reg[2] = {TMP1075_STARTUP_H, TMP1075_STARTUP_L};

	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_WRITE_REG, 2, u8reg);
	// then will need to wait at least 15ms

	return bOk;
}

/****************************************************************************
 *
 * NAME: vHTSstartReadTemp
 *
 * DESCRIPTION:
 * Wrapper to start a read of the temperature sensor.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC bool_t bSHTC3startRead()
{
	bool_t bOk = TRUE;

	// start conversion (will take some ms according to bits accuracy)
	uint8 u8reg[2] = {TMP1075_START_H, TMP1075_START_L};
	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_WRITE_REG, 2, u8reg);

//	vWait(TMP1075_CONVTIME);

	return bOk;
}

/****************************************************************************
 *
 * NAME: u16SHTC3readResult
 *
 * DESCRIPTION:
 * Wrapper to read a measurement, followed by a conversion function to work
 * out the value in degrees Celcius.
 *
 * RETURNS:
 * int16: temperature in degrees Celcius x 100 (-4685 to 12886)
 *        0x8000, error
 *
 * NOTES:
 * the data conversion fomula is :
 *      TEMP:  -46.85+175.72*ReadValue/65536
 *      HUMID: -6+125*ReadValue/65536
 *
 *    where the 14bit ReadValue is scaled up to 16bit
 *
 ****************************************************************************/
PUBLIC int16 i16SHTC3readResult(int16 *pi16Temp, int16 *pi16Humid)
{
	bool_t bOk = TRUE;
    uint8 au8data[2];
	uint16 u16data;
	int16 i16result;
	double sign = 100.0;
	uint8 u8reg;

	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_READ_REG, 0, &u8reg);
    bOk &= bSMBusSequentialRead(TMP1075_ADDRESS, 2, au8data);
    if(!bOk) return SHTC3_DATA_NOTYET; // error

 	u16data = (au8data[0] << 8) | au8data[1];
	if (u16data > 0x7fff) {
		u16data ^= 0xffff;
		u16data ++;
		sign = -100.0;
	}
	au8data[0] = u16data >> 8;
	au8data[1] = u16data & 0xff;
	i16result = (int16)(((double)au8data[0] + (double)au8data[1] / 256.0) * sign);
    *pi16Temp = i16result; 
    *pi16Humid = 0x8000; 
    
    return i16result;
}

PUBLIC bool_t bSHTC3sleep(){
	bool_t bOk = TRUE;

	return bOk;
}

PUBLIC bool_t bSHTC3wakeup(){
	bool_t bOk = TRUE;

	uint8 u8reg[2] = {TMP1075_STARTUP_H, TMP1075_STARTUP_L};
	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_WRITE_REG, 2, u8reg);
	
	return bOk;
}
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
// the Main loop
void vProcessSnsObj_SHTC3(void *pvObj, teEvent eEvent) {
	tsSnsObj *pSnsObj = (tsSnsObj *)pvObj;
	tsObjData_SHTC3 *pObj = (tsObjData_SHTC3 *)pSnsObj->pvData;
A_PRINTF(LB"!*** vProcessSnsObj_SHTC3 eEvent (%d) ***", eEvent);

	// general process
	switch (eEvent) {
		case E_EVENT_TICK_TIMER:
			if (pObj->u8TickCount < 100) {
A_PRINTF(LB"!*** vProcessSnsObj_SHTC3 u8TickDelta (%d) ***", pSnsObj->u8TickDelta);
				pObj->u8TickCount += pSnsObj->u8TickDelta;
#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "+");
#endif
			}
			break;
		case E_EVENT_START_UP:
			pObj->u8TickCount = 100; // expire immediately
#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "\n\rSHTC3 WAKEUP");
#endif
		break;

		default:
			break;
	}

	// state machine
	switch(pSnsObj->u8State)
	{
	case E_SNSOBJ_STATE_INACTIVE:
		// do nothing until E_ORDER_INITIALIZE event
		break;

	case E_SNSOBJ_STATE_IDLE:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			break;

		case E_ORDER_KICK:
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_MEASURING);

			#ifdef SERIAL_DEBUG
			vfPrintf(&sDebugStream, "\n\rSHTC3 KICKED");
			#endif
			break;

		default:
			break;
		}
		break;

	case E_SNSOBJ_STATE_MEASURING:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			pObj->ai16Result[SHTC3_IDX_TEMP] = SENSOR_TAG_DATA_ERROR;
			pObj->ai16Result[SHTC3_IDX_HUMID] = SENSOR_TAG_DATA_ERROR;
			pObj->u8TickWait = TMP1075_CONVTIME;

			// kick I2C communication
			if (!bSHTC3startRead()) {
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE); // error
			}

			pObj->u8TickCount = 0;
			break;

		default:
			break;
		}

		// wait until completion
		if (pObj->u8TickCount > pObj->u8TickWait) {
			int16 i16ret;
			i16ret = i16SHTC3readResult(
					&(pObj->ai16Result[SHTC3_IDX_TEMP]),
					&(pObj->ai16Result[SHTC3_IDX_HUMID]) );

			if (i16ret == SENSOR_TAG_DATA_ERROR) {
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE); // error
			} else
			if (i16ret == SENSOR_TAG_DATA_NOTYET) {
				// still conversion
				#ifdef SERIAL_DEBUG
				vfPrintf(&sDebugStream, "\r\nSHT_ND");
				#endif

				pObj->u8TickCount /= 2; // wait more...
			} else {
				// data arrival
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
			}
		}
		break;

	case E_SNSOBJ_STATE_COMPLETE:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			#ifdef SERIAL_DEBUG
			vfPrintf(&sDebugStream, "\n\rSHT_CP: T%d H%d",
					pObj->ai16Result[SHTC3_IDX_TEMP],
					pObj->ai16Result[SHTC3_IDX_HUMID]);
			#endif

			break;

		case E_ORDER_KICK:
			// back to IDLE state
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_IDLE);
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
