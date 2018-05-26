/*
 * RTC.h
 *
 *  Created on: Feb 16, 2017
 *      Author: Ted
 */

#ifndef RTC_RTC_H_
#define RTC_RTC_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "integer.h"
#include "barbarossa_config.h"

void RTC_Init(void);
DWORD RTC_GetFATtime(void);
HAL_StatusTypeDef RTC_GetTime(RTC_TimeTypeDef *time);
HAL_StatusTypeDef RTC_GetDate(RTC_DateTypeDef *date);


#endif /* RTC_RTC_H_ */
