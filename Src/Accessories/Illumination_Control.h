/*
 * Illumination_Control.h
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#ifndef ACCESSORIES_ILLUMINATION_CONTROL_H_
#define ACCESSORIES_ILLUMINATION_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "barbarossa_config.h"

void Illumination_Control_Init(void);
void Illumination_Control_Enable(void);
void Illumination_Control_Disable(void);

#endif /* ACCESSORIES_ILLUMINATION_CONTROL_H_ */
