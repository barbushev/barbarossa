/*
 * Door_Latch.h
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#ifndef ACCESSORIES_DOOR_LATCH_H_
#define ACCESSORIES_DOOR_LATCH_H_

#include "stm32f4xx_hal.h"
#include "barbarossa_config.h"

void Door_Latch_Init(void);
void Door_Latch_Lock(void);
void Door_Latch_Unlock(void);


#endif /* ACCESSORIES_DOOR_LATCH_H_ */
