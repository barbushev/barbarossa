/*
 * WatchDog.c
 *
 *  Created on: Mar 29, 2017
 *      Author: Ted
 */
#include "WatchDog.h"

//private variables
static IWDG_HandleTypeDef hiwdg;


/* IWDG init function */
void WatchDog_Init(void)
{
  //1/32768 (LSI)ms  x PRESCALER x Reload Value
  //0.000030517578125 x 8 x 4095 = 0.999 seconds
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
	  BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }
}

// call this to reset timer
void WatchDog_ResetCounter()
{
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
