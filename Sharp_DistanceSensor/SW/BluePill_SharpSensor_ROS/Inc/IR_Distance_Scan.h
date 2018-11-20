/*
 * IR_Distance_Scan.h
 *
 *  Created on: 19.11.2018
 *      Author: cc
 */

#ifndef IR_DISTANCE_SCAN_H_
#define IR_DISTANCE_SCAN_H_

#include <stdint.h>
#define IR_DISTANCE_SCAN_UNIT 100000
/* performs linear interpolation to calculate the distance
 * for the given ADC value
 * returns: -1 if scan is invalid
 * 			distance in mm if scan is valid*/
int32_t i32GetDistanceFromADCVal(uint16_t u16ADCValue);


#endif /* IR_DISTANCE_SCAN_H_ */
