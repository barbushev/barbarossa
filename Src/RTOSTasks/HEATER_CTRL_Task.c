/*
 * HEATER_CONTROL_TASK.c
 *
 *  Created on: Feb 14, 2017
 *      Author: Ted
 */

#include <HEATER_CTRL_Task.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MAX31856.h"
#include "USB_Task.h"
#include "Fan_Control.h"
#include "WatchDog.h"


//the absolute amount of time between each loop of the task (in ms)
#define Heat_Control_Task_Delay	(100)

//private variables
static TIM_HandleTypeDef handleHTRPWM;
static osThreadId HeatControl_TaskHandle;
static QueueHandle_t xMAX31856_MAILBOX; //used for sharing latest temperatures from MAX31856 chips with the rest of the program
static SemaphoreHandle_t xHEATER_CONTROL_Mutex; //used for sharing access to HEATER_CONTROL struct

static heater_ctrl_struct_t HEATER_CONTROL[HEATER_COUNT] =   //intialize to default PID values
{
		[0].kP = 20.0f,
		[0].kI = 0.1f,
		[0].kD = 0.0f,

		[1].kP = 20.0f,
		[1].kI = 0.1f,
		[1].kD = 0.0f,

		[2].kP = 20.0f,
		[2].kI = 0.1f,
		[2].kD = 0.0f,

		[3].kP = 20.0f,
		[3].kI = 0.1f,
		[3].kD = 0.0f,
};

//private functions
static void HeatControl(void const * argument);
static void HEATER_CTRL_Clamp(float *variableToClamp);
static void HEATER_CTRL_SetPWMDutyCycle(uint8_t heatNum, uint16_t newDuty);
static uint16_t HEATER_CTRL_CalculateDrivePower(uint8_t heaterNum, uint8_t ResetIntegral);
static void HEATER_CTRL_Init(void);

void HEATER_CTRL_TaskStart()
{
	MAX31856_InitSPI();  //initialize the MAX31856 temperature feedback

	HEATER_CTRL_Init();  //initialize the PWM for heater control

	Fan_Control_Init();

    xMAX31856_MAILBOX = xQueueCreate( 1, sizeof( max31856_temperatures_t ) );  //create mailbox for sharing temperatures
    xHEATER_CONTROL_Mutex = xSemaphoreCreateMutex();

    /* definition and creation of TempRead_Task */
    osThreadDef(TempRead_Task, HeatControl, osPriorityRealtime, 0, 512);
	HeatControl_TaskHandle = osThreadCreate(osThread(TempRead_Task), NULL);
}


/* Heater Control Task - this is called by the RTOS scheduler */
void HeatControl(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		max31856_temperatures_t xData;

		//Get the latest temperature
		if (MAX31856_GetTCTemp(0, &(xData.Temperature0)) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_READ, 0);
		if (MAX31856_GetTCTemp(1, &(xData.Temperature1)) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_READ, 1);
		if (MAX31856_GetTCTemp(2, &(xData.Temperature2)) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_READ, 2);
		if (MAX31856_GetTCTemp(3, &(xData.Temperature3)) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_READ, 3);
		//NOTE if there is an error, maybe set duty cycle to 0?


		//Check reading against min and max temp. Read fault pin as well?
		if (xData.Temperature0 > HEATER_MAX_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMAX, 0);   //(int32_t)(xData.Temperature0 * 100000)
		else if (xData.Temperature0 < HEATER_MIN_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMIN, 0);

		if (xData.Temperature1 > HEATER_MAX_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMAX, 1);
		else if (xData.Temperature1 < HEATER_MIN_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMIN, 1);

		if (xData.Temperature2 > HEATER_MAX_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMAX, 2);
		else if (xData.Temperature2 < HEATER_MIN_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMIN, 2);

		if (xData.Temperature3 > HEATER_MAX_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMAX, 3);
		else if (xData.Temperature3 < HEATER_MIN_TEMP) BA_ERROR_HANLDER(BA_ERROR_MAX31856_TMIN, 3);

		if (xSemaphoreTake( xHEATER_CONTROL_Mutex, pdMS_TO_TICKS( 80 ) ) == pdPASS)  //max delay is 80ms
			{
			HEATER_CONTROL[0].TemperatureActual = xData.Temperature0;  //put temperatures in mailbox
			HEATER_CONTROL[1].TemperatureActual = xData.Temperature1;
			HEATER_CONTROL[2].TemperatureActual = xData.Temperature2;
			HEATER_CONTROL[3].TemperatureActual = xData.Temperature3;

			//if there is a set target, recalculate and set the duty cycle
			if (HEATER_CONTROL[0].TargetTemperature > 0) HEATER_CTRL_SetPWMDutyCycle(0, HEATER_CTRL_CalculateDrivePower(0, 0));
			if (HEATER_CONTROL[1].TargetTemperature > 0) HEATER_CTRL_SetPWMDutyCycle(1, HEATER_CTRL_CalculateDrivePower(1, 0));
			if (HEATER_CONTROL[2].TargetTemperature > 0) HEATER_CTRL_SetPWMDutyCycle(2, HEATER_CTRL_CalculateDrivePower(2, 0));
			if (HEATER_CONTROL[3].TargetTemperature > 0) HEATER_CTRL_SetPWMDutyCycle(3, HEATER_CTRL_CalculateDrivePower(3, 0));

			xData.Duty0 = HEATER_CONTROL[0].ControlVariable;
			xData.Duty1 = HEATER_CONTROL[1].ControlVariable;
			xData.Duty2 = HEATER_CONTROL[2].ControlVariable;
			xData.Duty3 = HEATER_CONTROL[3].ControlVariable;

			xSemaphoreGive(xHEATER_CONTROL_Mutex);
			}
		else BA_ERROR_HANLDER(BA_ERROR_MAX31856_TOUT, 0);		//timeout

		/* Use the RTOS tick count as the time stamp. */
		xData.TimeStamp = xTaskGetTickCount();
		/* Send the structure to the mailbox - overwriting any data that is already in the mailbox. */
		xQueueOverwrite( xMAX31856_MAILBOX, &xData );

		WatchDog_ResetCounter();

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );  //100ms absolute delay
	}
  /* USER CODE END TempRead */
}


/********************************************************
 * <Info> Initializes GPIOs and TIM for Heater Control
 * <return> None
 * <note> 0,1,2,3 - total of 4 heaters set to low
 **********************************************************/
static void HEATER_CTRL_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();                     //CORRECT TIM #!

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;		//CORRECT TIM #!
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  for (uint8_t i = 0; i < HEATER_COUNT; i++)
  {
	  GPIO_InitStruct.Pin = BA_HEATER_IO[i].IO.IO_PIN;
	  HAL_GPIO_Init(BA_HEATER_IO[i].IO.IO_PORT, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(BA_HEATER_IO[i].IO.IO_PORT, BA_HEATER_IO[i].IO.IO_PIN, GPIO_PIN_RESET);
  }

  //uint32_t period  = (HAL_RCC_GetSysClockFreq() / (HEAT_TIMER_PRESCALER * HEAT_TIMER_FREQUENCY)) - 1;

  //TIM4 is connected to APB1 bus, which has on F429 device 42MHz clock. But, timer has internal PLL, which double this frequency for timer, up to 84MHz

  handleHTRPWM.Instance = TIM4;								//CHECK CORRECT TIM #
  handleHTRPWM.Init.Prescaler =  HEAT_TIMER_PRESCALER -1;
  handleHTRPWM.Init.CounterMode = TIM_COUNTERMODE_UP;
  handleHTRPWM.Init.Period = HEATER_MAX_DUTY_CYCLE;    //period;
  handleHTRPWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //handleHTRPWM.Init.RepetitionCounter = 99;  //it will take 99+1 repetitions before an interrupt is generated

  if (HAL_TIM_Base_Init(&handleHTRPWM) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&handleHTRPWM, &sClockSourceConfig) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  if (HAL_TIM_PWM_Init(&handleHTRPWM) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  /*
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&handleHTRPWM, &sBreakDeadTimeConfig);
   */

  TIM_SlaveConfigTypeDef sSlaveConfig;
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  HAL_TIM_SlaveConfigSynchronization(&handleHTRPWM, &sSlaveConfig);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&handleHTRPWM, &sMasterConfig);


  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  for (uint8_t i = 0; i < HEATER_COUNT; i++)
  {
	  if (HAL_TIM_PWM_ConfigChannel(&handleHTRPWM, &sConfigOC, BA_HEATER_IO[i].TIM_CHANNEL) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }

  HAL_TIM_PWM_Start(&handleHTRPWM, BA_HEATER_IO[0].TIM_CHANNEL);
  HAL_TIM_PWM_Start(&handleHTRPWM, BA_HEATER_IO[1].TIM_CHANNEL);
  HAL_TIM_PWM_Start(&handleHTRPWM, BA_HEATER_IO[2].TIM_CHANNEL);
  HAL_TIM_PWM_Start(&handleHTRPWM, BA_HEATER_IO[3].TIM_CHANNEL);
}



/********************************************************
 * <Info> Sets the Duty Cycle for a specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <newDuty> The new Duty Cycle value
 * <note> 0,1,2,3 - total of 4 heaters set to low
 **********************************************************/
static void HEATER_CTRL_SetPWMDutyCycle(uint8_t heatNum, uint16_t newDuty)
{
  __HAL_TIM_SET_COMPARE(&handleHTRPWM, BA_HEATER_IO[heatNum].TIM_CHANNEL, newDuty);
}


/********************************************************
 * <Info> Clamps the output (duty cycle) to the mix and max values
 * <variableToClamp> Pointer to the variable to clamp
 * <note> 0,1,2,3 - total of 4 heaters set to low
 **********************************************************/
static void HEATER_CTRL_Clamp(float *variableToClamp)
{
	if (isnan(*variableToClamp)) { *variableToClamp =  HEATER_MIN_DUTY_CYCLE; }
    else if (*variableToClamp <= HEATER_MIN_DUTY_CYCLE) { *variableToClamp = HEATER_MIN_DUTY_CYCLE; }
    else if (*variableToClamp >= HEATER_MAX_DUTY_CYCLE) { *variableToClamp = HEATER_MAX_DUTY_CYCLE; }
}

/********************************************************
 * <Info> Calculate the Duty Cycle for specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <ResetIntegral> 0 - false; 1 - true clears Integral error.
 * <return> New duty cycle
 * <note> 0,1,2,3 - total of 4 heaters set to low
 **********************************************************/
static uint16_t HEATER_CTRL_CalculateDrivePower(uint8_t heatNum, uint8_t ResetIntegral)
{
    //TemperatureActual and TemperatureLast may change while calculating so make a copy of these in advance
	float TemperatureActual = HEATER_CONTROL[heatNum].TemperatureActual;
	float TemperatureLast   = HEATER_CONTROL[heatNum].TemperatureLast;
	if (ResetIntegral) HEATER_CONTROL[heatNum].IntegralTerm = 0.0f;

	 //float dTime = 0.1f; // time since last update in seconds - THIS SHOULD BE CLOSE TO CONSTANT so it is not needed
	 float error = HEATER_CONTROL[heatNum].TargetTemperature - TemperatureActual;  //calculate error
     float proportionalTerm = HEATER_CONTROL[heatNum].kP * error;  // proportional term calculation
     HEATER_CONTROL[heatNum].IntegralTerm += (HEATER_CONTROL[heatNum].kI * error); // * dTime);

     // this only seems to want to go negative when cooling down.
     if (HEATER_CONTROL[heatNum].IntegralTerm < 0) HEATER_CONTROL[heatNum].IntegralTerm = 0;

     // derivative term calculation
     float dInput = TemperatureLast - TemperatureActual;

	 float derivativeTerm = 0;
	 if (dInput != 0) derivativeTerm = HEATER_CONTROL[heatNum].kD * (dInput); //   / dTime);
     float output = proportionalTerm + HEATER_CONTROL[heatNum].IntegralTerm + derivativeTerm;

     HEATER_CTRL_Clamp(&output);

     HEATER_CONTROL[heatNum].ControlVariable = output;

     return (uint16_t)output;
}



/***********************************PUBLIC FUNCTIONS *******************************************/


/********************************************************
 * <Info> Get the most recent temperature reading for a specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <return> Returns the temperature value
 * <note> 0,1,2,3 - total of 4 heaters (0 based)
 **********************************************************/
float HEATER_CTRL_GetCurrentDuty(uint8_t heatNum)
{
	max31856_temperatures_t xData;

	 /* Update the Example_t structure pointed to by pxData with the data contained in
		 the mailbox. If xQueueReceive() was used here then the mailbox would be left
		 empty, and the data could not then be read by any other tasks. Using
		 xQueuePeek() instead of xQueueReceive() ensures the data remains in the mailbox.
		 A block time of 0ms specified, so the calling task will be not placed in the Blocked
		 state to wait for the mailbox to contain data should the mailbox be empty. */
	if (xQueuePeek( xMAX31856_MAILBOX, &xData, 0 ) != pdPASS) return 0;  //if there is nothing in the queue

	switch (heatNum)
		{
		case 0: return xData.Duty0;
			break;
		case 1: return xData.Duty1;
			break;
		case 2: return xData.Duty2;
			break;
		case 3: return xData.Duty3;
			break;
		}

	//shouldn't get here
	return 0;
}



/********************************************************
 * <Info> Get the most recent temperature reading for a specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <return> Returns the temperature value
 * <note> 0,1,2,3 - total of 4 heaters (0 based)
 **********************************************************/
float HEATER_CTRL_GetCurrentTemp(uint8_t heatNum)
{
	max31856_temperatures_t xData;

	 /* Update the Example_t structure pointed to by pxData with the data contained in
		 the mailbox. If xQueueReceive() was used here then the mailbox would be left
		 empty, and the data could not then be read by any other tasks. Using
		 xQueuePeek() instead of xQueueReceive() ensures the data remains in the mailbox.
		 A block time of 0ms specified, so the calling task will be not placed in the Blocked
		 state to wait for the mailbox to contain data should the mailbox be empty. */
	if (xQueuePeek( xMAX31856_MAILBOX, &xData, 0 ) != pdPASS) return 0;  //if there is nothing in the queue

	switch (heatNum)
		{
		case 0: return xData.Temperature0;
			break;
		case 1: return xData.Temperature1;
			break;
		case 2: return xData.Temperature2;
			break;
		case 3: return xData.Temperature3;
			break;
		}

	//shouldn't get here
	return 0;
}


/********************************************************
 * <Info> Check if a certain heater temperature is within a range
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <lowVal> Low end of range
 * <highVal> High end of range
 * <return> Returns 1 if true, else 0.
 * <note> 0,1,2,3 - total of 4 heaters (0 based)
 **********************************************************/
uint8_t HEATER_CTRL_HeaterIsInRange(uint8_t heatNum, float lowVal, float highVal)
{
	float currentTemp = HEATER_CTRL_GetCurrentTemp(heatNum);

	if ((currentTemp >= lowVal) && (currentTemp <= highVal)) return 1;
	else return 0;
}

/********************************************************
 * <Info> Set PID for a specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <kP> Proportional coefficient
 * <kI> Integral coefficient
 * <kD> Derivative coefficient
 * <note> 0,1,2,3 - total of 4 heaters (0 based)
 **********************************************************/
void HEATER_CTRL_SetPID(uint8_t heatNum, float kP, float kI, float kD)
{
	if (xSemaphoreTake( xHEATER_CONTROL_Mutex, pdMS_TO_TICKS( 500 ) ) == pdPASS)  //max delay is 500ms
		{
		HEATER_CONTROL[heatNum].kP = kP;
		HEATER_CONTROL[heatNum].kI = kI;
		HEATER_CONTROL[heatNum].kD = kD;
		xSemaphoreGive(xHEATER_CONTROL_Mutex);
		}
	else BA_ERROR_HANLDER(BA_ERROR_MAX31856_TOUT, 0);  //timeout
}

/********************************************************
 * <Info> Turn on a specific heater to a target temperature
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <TargetTemperature> Target Temperature
 * <note> 0,1,2,3 - total of 4 heaters (0 based)
 **********************************************************/
void HEATER_CTRL_HeaterON(uint8_t heatNum, float TargetTemperature)
{
	if (xSemaphoreTake( xHEATER_CONTROL_Mutex, pdMS_TO_TICKS( 500 ) ) == pdPASS)  //max delay is 500ms
		{
		uint8_t resetIntegral = 1; //if the new target temp is higher than the current one, do not reset integral
		if (TargetTemperature > HEATER_CONTROL[heatNum].TargetTemperature) resetIntegral = 0;
		HEATER_CONTROL[heatNum].TargetTemperature = TargetTemperature;
		HAL_TIM_PWM_Start(&handleHTRPWM, BA_HEATER_IO[heatNum].TIM_CHANNEL);
		HEATER_CTRL_SetPWMDutyCycle(heatNum, HEATER_CTRL_CalculateDrivePower(heatNum, resetIntegral));
		xSemaphoreGive(xHEATER_CONTROL_Mutex);
		}
	else BA_ERROR_HANLDER(BA_ERROR_MAX31856_TOUT, 0);  //timeout
}

/********************************************************
 * <Info> Turn off a specific heater
 * <heatNum> Number of the heater 0, 1, 2, 3
 * <note> 0,1,2,3 - total of 4 heaters set to low
 **********************************************************/
void HEATER_CTRL_HeaterOFF(uint8_t heatNum)
{
	if (xSemaphoreTake( xHEATER_CONTROL_Mutex, pdMS_TO_TICKS( 500 ) ) == pdPASS)  //max delay is 500ms
		{
		HEATER_CONTROL[heatNum].TargetTemperature = 0.0f;
		HEATER_CONTROL[heatNum].ControlVariable = HEATER_MIN_DUTY_CYCLE; //aka OFF
		HEATER_CTRL_SetPWMDutyCycle(heatNum, HEATER_MIN_DUTY_CYCLE);
		HAL_TIM_PWM_Stop(&handleHTRPWM, BA_HEATER_IO[heatNum].TIM_CHANNEL);
		xSemaphoreGive(xHEATER_CONTROL_Mutex);
		}
	else BA_ERROR_HANLDER(BA_ERROR_MAX31856_TOUT, 0);  //timeout
}
