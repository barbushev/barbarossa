/*
 * BA_tasks.c
 *
 *  Created on: Feb 7, 2017
 *      Author: Ted
 */
#include <HEATER_CTRL_Task.h>
#include "BA_tasks.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "barbarossa_config.h"


#include "USB_Task.h"
#include "FlowRunner_Task.h"
#include "FileSystem_Task.h"


osThreadId MainTaskHandle;
osThreadId StepperDriver_THandle;
osThreadId FileSystem_TaskHandle;
osThreadId FlowRunner_TaskHandle;
osThreadId Idle_TaskHandle;

void MainT(void const * argument);
void StepperDriver(void const * argument);
void Idle(void const * argument);

void RTOS_Start()
{
	  FileSystem_TaskStart();

	  USB_TaskStart();

	  HEATER_CTRL_TaskStart();

	  FlowRunner_TaskStart();

	  /* Create the thread(s) */
	  /* definition and creation of MainTask */
	  osThreadDef(MainTask, MainT, osPriorityNormal, 0, 256);
	  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

	  /* definition and creation of StepperDriver_T */
	  osThreadDef(StepperDriver_T, StepperDriver, osPriorityAboveNormal, 0, 128);
	  StepperDriver_THandle = osThreadCreate(osThread(StepperDriver_T), NULL);

	  /* definition and creation of Idle_Task */
	  osThreadDef(Idle_Task, Idle, osPriorityIdle, 0, 64);
	  Idle_TaskHandle = osThreadCreate(osThread(Idle_Task), NULL);


	  /* Start scheduler */
	  osKernelStart();
}


void MainT(void const * argument)
{
  for(;;)
  {
//	  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
	  osDelay(1000);
  }
}

/* StepperDriver function */
void StepperDriver(void const * argument)
{
  /* USER CODE BEGIN StepperDriver */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
  }
  /* USER CODE END StepperDriver */
}

/* Idle function */
void Idle(void const * argument)
{
  /* USER CODE BEGIN Idle */
  /* Infinite loop */
  for(;;)
  {



  }
  /* USER CODE END Idle */
}





/*
__IO uint32_t DebugTick;
TIM_HandleTypeDef handleDebugTick;

void BA_ConfigureTimerForDebugTick()
{
//configGENERATE_RUN_TIME_STATS
	__TIM14_CLK_ENABLE();

	//84000000 Hz / 10 / 84 = 100 000 Hz or 1uS

	handleDebugTick.Instance = TIM14;								//CHECK CORRECT TIM #
	handleDebugTick.Init.Prescaler =  10; //84 -1;
	handleDebugTick.Init.CounterMode = TIM_COUNTERMODE_UP;
	handleDebugTick.Init.Period = 84;    //period;
	handleDebugTick.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_Base_Init(&handleDebugTick);

	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&handleDebugTick, &sClockSourceConfig);

	HAL_TIM_Base_Start_IT(&handleDebugTick);

	HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 14, 0);
	HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

}

void BA_IncDebugTick()
{
	DebugTick++;
}

uint32_t BA_GetDebugTickCount()
{
	return DebugTick;
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&handleDebugTick);
}
*/

/*
void PrintFloats(float num, uint8_t *txt)
{
	 int d1 = num;            // Get the integer part (678).
	 float f2 = (num - d1);     // Get fractional part (678.0123 - 678 = 0.0123).
	 int d2 = abs(trunc(f2 * 10000));   // Turn into integer (123).

	 // Print as parts, note that you need 0-padding for fractional bit.
	 // Since d1 is 678 and d2 is 123, you get "678.0123".
	 //USB_Send(100, "%s %d.%04d\r\n", txt, d1, d2);
}
*/

static TaskHandle_t xTaskToNotify = NULL;

/* The peripheral driver's transmit function. */
void StartTransmission( uint8_t *pcData, uint16_t xDataLength )
{
    /* At this point xTaskToNotify should be NULL as no transmission
    is in progress.  A mutex can be used to guard access to the
    peripheral if necessary. */
    configASSERT( xTaskToNotify == NULL );

    /* Store the handle of the calling task. */
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    /* Start the transmission - an interrupt is generated when the
    transmission is complete. */
    CDC_Transmit_FS( pcData, xDataLength );
}
/*-----------------------------------------------------------*/

/* The transmit end interrupt. */
void vTransmitEndISR( void )
{
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* At this point xTaskToNotify should not be NULL as a transmission was
    in progress. */
    configASSERT( xTaskToNotify != NULL );

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );

    /* There are no transmissions in progress, so no tasks to notify. */
    xTaskToNotify = NULL;

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

/* The task that initiates the transmission, then enters the Blocked state (so
not consuming any CPU time) to wait for it to complete. */
void vAFunctionCalledFromATask( uint8_t *ucDataToTransmit, uint16_t xDataLength )
{
uint32_t ulNotificationValue;
const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );

    /* Start the transmission by calling the function shown above. */
    StartTransmission( ucDataToTransmit, xDataLength );

    /* Wait to be notified that the transmission is complete.  Note the first
    parameter is pdTRUE, which has the effect of clearing the task's notification
    value back to 0, making the notification value act like a binary (rather than
    a counting) semaphore.  */
    ulNotificationValue = ulTaskNotifyTake( pdTRUE,
                                            xMaxBlockTime );

    if( ulNotificationValue == 1 )
    {
        /* The transmission ended as expected. */
    }
    else
    {
        /* The call to ulTaskNotifyTake() timed out. */
    }
}



