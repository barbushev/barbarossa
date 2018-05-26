/*
 * fluid_sensor.h
 *
 *  Created on: Dec 28, 2016
 *      Author: Ted
 */
#include <barbarossa_config.h>
#include "cmsis_os.h"

#ifndef FLUID_SENSOR_H_
#define FLUID_SENSOR_H_

HAL_StatusTypeDef Fluid_Sensor_Init();
void Fluid_Sensor_ClearWatchDog();
void Fluid_Sensor_SetWatchDog(uint8_t sensNum, uint32_t LowTh, uint32_t HighTh, osThreadId TaskToNotify);
uint32_t Fluid_Sensor_GetSampleCount();
uint16_t Fluid_Sensor_GetValue(uint8_t SensorNum);

#endif /* FLUID_SENSOR_H_ */
