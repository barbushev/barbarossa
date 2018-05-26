/*
 * Door_Sensor.h
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#ifndef ACCESSORIES_DOOR_SENSOR_H_
#define ACCESSORIES_DOOR_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "barbarossa_config.h"

void Door_Sensor_Init(void);
uint8_t Door_Sensor_GetState();


#endif /* ACCESSORIES_DOOR_SENSOR_H_ */
