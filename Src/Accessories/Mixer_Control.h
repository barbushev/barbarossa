/*
 * Mixer_Control.h
 *
 *  Created on: Mar 7, 2017
 *      Author: Ted
 */

#ifndef ACCESSORIES_MIXER_CONTROL_H_
#define ACCESSORIES_MIXER_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "barbarossa_config.h"



HAL_StatusTypeDef Mixer_Control_Init(void);
void Mixer_Control_MixerOff();
void Mixer_Control_MixerOn(uint16_t newDuty);
void Mixer_Control_SenseIT(void);
uint32_t Mixer_Control_GetRevs();
void Mixer_Control_ClearRevs();


#endif /* ACCESSORIES_MIXER_CONTROL_H_ */
