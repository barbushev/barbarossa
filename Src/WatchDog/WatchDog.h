/*
 * WatchDog.h
 *
 *  Created on: Mar 29, 2017
 *      Author: Ted
 */

#ifndef WATCHDOG_WATCHDOG_H_
#define WATCHDOG_WATCHDOG_H_


#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"
#include "barbarossa_config.h"


void WatchDog_Init(void);
void WatchDog_ResetCounter();


#endif /* WATCHDOG_WATCHDOG_H_ */
