/*
 * Fan_Control.h
 *
 *  Created on: Feb 21, 2017
 *      Author: Ted
 */

#ifndef HEATCONTROL_FAN_CONTROL_H_
#define HEATCONTROL_FAN_CONTROL_H_
#include "stm32f4xx_hal.h"


void Fan_Control_Init();

void Fan_Control_On();
void Fan_Control_Off();
uint16_t Fan_Control_GetRPM(uint8_t FanNum);


#endif /* HEATCONTROL_FAN_CONTROL_H_ */
