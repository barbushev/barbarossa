/*
 * fluid_sensor.c
 *
 *  Created on: Dec 28, 2016
 *      Author: Ted
 */
#include "stm32f4xx.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "fluid_sensor.h"
#include "main.h"

#include "USB_Task.h"   //remove later


static ADC_HandleTypeDef adcHandle;
static DMA_HandleTypeDef dmaHandle;
static volatile uint32_t convCount = 0;
static volatile uint32_t ADCBuffer[BA_ADC_SENSOR_COUNT];
static osThreadId handleOfTaskToNotify;

HAL_StatusTypeDef Fluid_Sensor_Init()
{
	__HAL_RCC_ADC3_CLK_ENABLE();								//CORRECT ADC# GOES HERE

	GPIO_InitTypeDef gpioInit;

    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;

    for (uint8_t i = 0; i < BA_ADC_SENSOR_COUNT; i++)
    {
    	gpioInit.Pin = BA_SENSORS[i].IO.IO_PIN;
    	HAL_GPIO_Init(BA_SENSORS[i].IO.IO_PORT, &gpioInit);
    }

    HAL_NVIC_SetPriority(ADC_IRQn, 5, 15);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

	adcHandle.Instance = ADC3;									//CORRECT ADC# GOES HERE     //sampling of DT box on Portrait at best is 1sample per 10ms.
	adcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;	//gets approx 1 sample of each sensor per 515uS -  ADC_CLOCK_SYNC_PCLK_DIV8
	adcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	adcHandle.Init.ScanConvMode = ENABLE;
	adcHandle.Init.ContinuousConvMode = ENABLE;
	adcHandle.Init.DiscontinuousConvMode = DISABLE;
	adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* Software start to trig the 1st conversion manually, without external event */
	adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.NbrOfConversion = BA_ADC_SENSOR_COUNT;
	adcHandle.Init.NbrOfDiscConversion = 0;
	adcHandle.Init.DMAContinuousRequests = ENABLE; /* ADC-DMA continuous requests to match with DMA configured in circular mode */
	adcHandle.Init.EOCSelection = DISABLE;

	HAL_StatusTypeDef status =  HAL_ADC_Init(&adcHandle);


	ADC_ChannelConfTypeDef adcChannel;

    for (uint8_t i = 0; i < BA_ADC_SENSOR_COUNT; i++)
    {
        adcChannel.Channel = BA_SENSORS[i].adcChannel;
        adcChannel.Rank = i + 1;								//This parameter must be a number between Min_Data = 1 and Max_Data = 16 */
        adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        status |= HAL_ADC_ConfigChannel(&adcHandle, &adcChannel);
    }

    __HAL_RCC_DMA2_CLK_ENABLE(); // DMA controller clock enable
    dmaHandle.Instance = DMA2_Stream0;
    dmaHandle.Init.Channel = DMA_CHANNEL_2;
    dmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;// Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits
    dmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD; // Transfer to memory by half-word to match with buffer variable type: half-word
    dmaHandle.Init.Mode = DMA_CIRCULAR; // DMA in circular mode to match with ADC configuration: DMA continuous requests
    dmaHandle.Init.Priority = DMA_PRIORITY_LOW;
    dmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    status |= HAL_DMA_Init(&dmaHandle);

        // DMA interrupt init
        // DMA2_Stream0_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);


   // Associate the initialized DMA handle to the ADC handle
   __HAL_LINKDMA(&adcHandle,DMA_Handle,dmaHandle);

   // -- Enables ADC DMA request - Casting as uint32_t since that is data type used by DMA
   status |= HAL_ADC_Start_DMA(&adcHandle, (uint32_t *)ADCBuffer, BA_ADC_SENSOR_COUNT);

   return status;
}



/*
 * Set up an analog watch dog.
 * sensNum is zero based
 * LowTh and HighTh are 0 to 4095
 * Values outside of the watchdog window (LowTh to HighTh) will generate an HAL_ADC_LevelOutOfWindowCallback
 * TaskToNotify is the handle to the task to be notified from HAL_ADC_LevelOutOfWindowCallback
 */
void Fluid_Sensor_SetWatchDog(uint8_t sensNum, uint32_t LowTh, uint32_t HighTh, osThreadId TaskToNotify)
{
	handleOfTaskToNotify = TaskToNotify;

	ADC_AnalogWDGConfTypeDef  WatchDog =
	{
			  .WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG,
			  .HighThreshold = HighTh,     //max value
			  .LowThreshold = LowTh,
			  .Channel =  BA_SENSORS[sensNum].adcChannel ,
			  .ITMode = ENABLE,
			  .WatchdogNumber = 0
	};

	if (HAL_ADC_AnalogWDGConfig (&adcHandle, &WatchDog) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_AWDCFG, 0);
}



/*
 * Used to clear the watch dog flag and disable the watch dog
 */
void Fluid_Sensor_ClearWatchDog()
{
	__HAL_ADC_CLEAR_FLAG(&adcHandle, ADC_FLAG_AWD);

	ADC_AnalogWDGConfTypeDef  WatchDog =
	{
	  .WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG,
	  .HighThreshold = 4095,     //max value
	  .LowThreshold = 0,
	  .Channel =  BA_SENSORS[0].adcChannel ,
	  .ITMode = DISABLE,
	  .WatchdogNumber = 0
	};

	if (HAL_ADC_AnalogWDGConfig (&adcHandle, &WatchDog) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_AWDCFG, 0);
}

inline uint16_t Fluid_Sensor_GetValue(uint8_t SensorNum)
{
	return (uint16_t)ADCBuffer[SensorNum];
}

uint32_t Fluid_Sensor_GetSampleCount()
{
	return convCount;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
   convCount++;
}

void DMA2_Stream0_IRQHandler()
    {
        HAL_DMA_IRQHandler(&dmaHandle);
    }


void ADC_IRQHandler()
    {
        HAL_ADC_IRQHandler(&adcHandle);  //it does get here when the watch dog is triggered
    }


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
// call the error handler

}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	//if the value falls out of the watchdog window
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_AWD) == SET)
	{
		Fluid_Sensor_ClearWatchDog();
		//signal the Task

		/* An interrupt handler that unblocks a high priority task in which the event that
		generated the interrupt is processed. If the priority of the task is high enough
		then the interrupt will return directly to the task (so it will interrupt one task
		then return to a different task), so the processing will occur contiguously in time -
		just as if all the processing had been done in the interrupt handler itself. */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		 /* Unblock the handling task so the task can perform any processing necessitated
		 by the interrupt. xHandlingTask is the task's handle, which was obtained
		 when the task was created. */

		if (handleOfTaskToNotify != NULL)
		{
		vTaskNotifyGiveFromISR( handleOfTaskToNotify, &xHigherPriorityTaskWoken );

		//Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
		//The macro used to do this is dependent on the port and may be called portEND_SWITCHING_ISR().

		handleOfTaskToNotify = NULL;
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}

	}
}



/*
uint16_t FLUID_SNS_POLL(uint8_t channel_num)
{
	ADC_ChannelConfTypeDef sConfig;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES ;
	sConfig.Rank      = 1;

	switch (channel_num)
	{
	case 0: { sConfig.Channel = ADC_CHANNEL_0; break; }
	case 1: { sConfig.Channel = ADC_CHANNEL_3; break; }
	case 2: { sConfig.Channel = ADC_CHANNEL_4; break; }
	case 3: { sConfig.Channel = ADC_CHANNEL_5; break; }

	default: return 0;
	}

	HAL_ADC_ConfigChannel(&adcHandle, &sConfig);
	HAL_ADC_Start(&adcHandle);

	HAL_ADC_PollForConversion(&adcHandle,10);
	//ADValue[channel_num]=HAL_ADC_GetValue(&adcHandle);
	return (uint16_t )HAL_ADC_GetValue(&adcHandle);
}
*/
