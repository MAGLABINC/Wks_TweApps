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
#include "TMP1075.h"
#include "SMBus.h"

// DEBUG options
#define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#include "utils.h"
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"
PUBLIC tsFILE sSerStream;
#endif
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

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vTMP1075reset
 *
 * DESCRIPTION:
 *   to reset TMP1075 device
 *
 * RETURNS:
 * bool_t	fail or success
 *
 ****************************************************************************/
PUBLIC bool_t bTMP1075reset()
{
	bool_t bOk = TRUE;
	uint8 u8reg[2] = {TMP1075_STARTUP_H, TMP1075_STARTUP_L};

	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_WRITE_REG, 2, u8reg);
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, LB"TMP1075 WRITE STARTUP bOk(%d)", bOk);
#endif

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
PUBLIC bool_t bTMP1075startRead()
{
	bool_t bOk = TRUE;
	uint8 u8reg[2] = {TMP1075_START_H, TMP1075_START_L};

	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_WRITE_REG, 2, u8reg);
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, LB"TMP1075 WRITE START bOk(%d)", bOk);
#endif

	vWait(TMP1075_CONVTIME);

	return bOk;
}

/****************************************************************************
 *
 * NAME: u16TMP1075readResult
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
 *
 *    where the 14bit ReadValue is scaled up to 16bit
 *
 ****************************************************************************/
PUBLIC int16 i16TMP1075readResult()
{
	bool_t bOk = TRUE;
    uint8 au8data[2];
	uint16 u16data;
	int16 i16result;
	double sign = 100.0;
	uint8 u8reg;

	bOk &= bSMBusWrite(TMP1075_ADDRESS, TMP1075_READ_REG, 0, &u8reg);
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, LB"TMP1075 WRITE READ REG bOk(%d)", bOk);
#endif
    bOk &= bSMBusSequentialRead(TMP1075_ADDRESS, 2, au8data);
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, LB"TMP1075 READ TEMP DATA bOk(%d)", bOk);
	vfPrintf(&sSerStream, LB"TMP1075 %x %x", au8data[0], au8data[1]);
#endif
    if(!bOk) return TMP1075_DATA_NOTYET; // error
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, LB"TMP1075 %x %x", au8data[0], au8data[1]);
#endif

 	u16data = (au8data[0] << 8) | au8data[1];
	if (u16data > 0x7fff) {
		u16data ^= 0xffff;
		u16data ++;
		sign = -100.0;
	}
	au8data[0] = u16data >> 8;
	au8data[1] = u16data & 0xff;
	i16result = (int16)(((double)au8data[0] + (double)au8data[1] / 256.0) * sign);
#ifdef SERIAL_DEBUG
	vfPrintf(&sSerStream, "au8data = %X, %X", au8data[0], au8data[1]);
	vfPrintf(&sSerStream, "i16result = %d(%f)", i16result, (double)i16result / 100.0);
#endif

    return i16result;
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
