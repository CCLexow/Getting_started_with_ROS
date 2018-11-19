/*
 * IR_Distance_Scan.c
 *
 *  Created on: 19.11.2018
 *      Author: Carl Christian Lexow
 */



#include <IR_Distance_Scan.h>

#define LUT_LENGTH	40

/* Distance in mm / 100 */
const uint16_t cau16LUTDistance[LUT_LENGTH] = {
		7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000, 15000, 16000,
		17000, 18000, 19000, 20000, 21000, 22000, 23000, 24000, 25000, 26000,
		27000, 28000, 29000, 30000, 31000, 32000, 33000, 34000, 35000, 36000,
		37000, 38000, 39000, 40000, 42500, 45000, 47500, 50000, 52500, 55000
};

/* register values for 12 Bit ADC */
const uint16_t cau16LUTRegVal[LUT_LENGTH] = {
		3733, 3393, 3077, 2798, 2584, 2361, 2181, 2044, 1926, 1811,
		1708, 1646, 1553, 1482, 1411, 1341, 1294, 1231, 1190, 1124,
		1099, 1051, 1004, 980, 933, 909, 882, 858, 833, 808,
		784, 760, 736, 733, 687, 645, 615, 591, 568, 562
};

const uint16_t cu16MinRegVal = 562;
const uint16_t cu16MaxRegVal = 3773;


/* performs linear interpolation to calculate the distance
 * for the given ADC value
 * returns: -1 if scan is invalid
 * 			distance in mm if scan is valid*/
int32_t i32GetDistanceFromADCVal(uint16_t u16ADCValue)
{
	int32_t i32RetVal=-1;
	int32_t i32Dist0, i32Dist1;
	int32_t i32Reg0, i32Reg1;
	uint16_t u16ReadIdx0;
	uint16_t u16ReadIdx1 = 0;

	/* verify if value is within limits of the LUT */
	if((cu16MinRegVal <= u16ADCValue) && (cu16MaxRegVal >= u16ADCValue))
	{
		/* get index of pair of values below and above given value */
		for(u16ReadIdx0=0;u16ReadIdx0 < LUT_LENGTH-1; u16ReadIdx0++)
		{
			/* given value between LUT values? */
			if((cau16LUTRegVal[u16ReadIdx0] >= u16ADCValue) &&
					(cau16LUTRegVal[u16ReadIdx0+1] <= u16ADCValue))
			{
				/* found pair of values */
				u16ReadIdx1 = u16ReadIdx0+1;
				break;
			}
		}

		if(0 != u16ReadIdx1)
		{
			/* get values for linear interpolation */
			i32Dist0 = (int32_t)cau16LUTDistance[u16ReadIdx0];
			i32Dist1 = (int32_t)cau16LUTDistance[u16ReadIdx1];
			i32Reg0  = (int32_t)cau16LUTRegVal[u16ReadIdx0];
			i32Reg1  = (int32_t)cau16LUTRegVal[u16ReadIdx1];
			/* perform linear interpolation */
			i32RetVal  = i32Dist1 - i32Dist0;
			i32RetVal *= ((int32_t)u16ADCValue - i32Reg0);
			i32RetVal /= (i32Reg1 - i32Reg0);
			i32RetVal += i32Dist0;
		}

	}

	return i32RetVal;
}
