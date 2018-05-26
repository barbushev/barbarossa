/*
 * RTC.c
 *
 *  Created on: Feb 16, 2017
 *      Author: Ted
 */
#include "RTC.h"
#include "barbarossa_config.h"

RTC_HandleTypeDef hrtc;

/* RTC init function */
void RTC_Init(void)
{
	//enable access to Power Controller PWR APB1 interface clock
	//__HAL_RCC_PWR_CLK_ENABLE();  //already enabled by  BKPSRAM_Init
	//HAL_PWR_EnableBkUpAccess();  //already enabled by BKPSRAM_Init

	__HAL_RCC_LSE_CONFIG(RCC_LSE_ON);  //enable external low speed oscillator LSE

	uint16_t timeout = 0;
	while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)  //wait for the LSE to start with a timeout
		{
			timeout++;
			if (timeout > 100000)
			{
				BA_ERROR_HANLDER(BA_ERROR_LSE_FAIL, 0);
				break;
			}
		}

	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
	__HAL_RCC_RTC_ENABLE();

    /**Initialize RTC Only*/
	//RTC_InitTypeDef   RTC_InitStructure;

	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
	}

	HAL_RTC_WaitForSynchro(&hrtc);  //wait for calender to syncronize
}


/**************************************************************************
 * Provides time for FAT timestamp
 *************************************************************************/
DWORD RTC_GetFATtime()
{
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, FORMAT_BIN);
	return ((2000 + date.Year - 1980) << 25) | ((date.Month) << 21) | ((date.Date) << 16) | ((time.Hours) << 11) | ((time.Minutes) << 5) | (time.Seconds / 2);
}

HAL_StatusTypeDef RTC_GetTime(RTC_TimeTypeDef *time)
{
	HAL_StatusTypeDef status = HAL_RTC_GetTime(&hrtc, time, FORMAT_BIN);

	RTC_DateTypeDef date;
	status |= HAL_RTC_GetDate(&hrtc, &date, FORMAT_BIN);

	return status;
}

HAL_StatusTypeDef RTC_GetDate(RTC_DateTypeDef *date)
{
	RTC_TimeTypeDef time;
	HAL_StatusTypeDef status = HAL_RTC_GetTime(&hrtc, &time, FORMAT_BIN);
	status |= HAL_RTC_GetDate(&hrtc, date, FORMAT_BIN);

	return status;
}
