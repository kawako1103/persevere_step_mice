/*
 * PL_sensor.h
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_
#define LIPO_LIMIT 10.5

#include "stm32l4xx_hal.h"

extern uint16_t g_ADCBuffer[5];
extern uint16_t g_sensor[4][4];

float pl_getbatt();

void pl_callback_getSensor();

void pl_interupt_getSensor();

#endif /* INC_PL_SENSOR_H_ */
