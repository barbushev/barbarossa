/*
 * HEATER_CONTROL_TASK.h
 *
 *  Created on: Feb 14, 2017
 *      Author: Ted
 */

#ifndef RTOSTASKS_HEATER_CTRL_TASK_H_
#define RTOSTASKS_HEATER_CTRL_TASK_H_

#include "stdint.h"

void HEATER_CTRL_TaskStart(void);
void HEATER_CTRL_SetPID(uint8_t heatNum, float kP, float kI, float kD);
void HEATER_CTRL_HeaterON(uint8_t heatNum, float TargetTemperature);
void HEATER_CTRL_HeaterOFF(uint8_t heatNum);
uint8_t HEATER_CTRL_HeaterIsInRange(uint8_t heatNum, float lowVal, float highVal);
float HEATER_CTRL_GetCurrentTemp(uint8_t heatNum);
float HEATER_CTRL_GetCurrentDuty(uint8_t heatNum);



#define constrain(x, a, b)  ((x) < (a))?(a):(((x) > (b))? (b) : (x))

typedef struct
{
    float kP;
    float kI;
	float kD;
    float IntegralTerm;
    float TargetTemperature;
    float TemperatureActual;
	float TemperatureLast;
	float ControlVariable;
}heater_ctrl_struct_t;

#endif /* RTOSTASKS_HEATER_CTRL_TASK_H_ */
