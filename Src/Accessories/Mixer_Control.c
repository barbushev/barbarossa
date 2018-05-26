/*
 * Mixer_Control.c
 *
 *  Created on: Mar 7, 2017
 *      Author: Ted
 */

#include "Mixer_Control.h"
#include "USB_Task.h"

static TIM_HandleTypeDef handleMixerPWM;

static void Mixer_Control_SetPWMDutyCycle(uint16_t newDuty);



static uint16_t MixerCurrentDuty = 0;				//used for keeping track of the current duty cycle
static volatile uint32_t MixerRevCounter = 0;		//used for keeping track of the number of revolutions

/*******************************************************************************
 * Description    : Initializes IO for Mixer Motor PWM Control.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
HAL_StatusTypeDef Mixer_Control_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();                     //CORRECT TIM #!

  GPIO_InitTypeDef GPIO_InitStruct;

  //Initialize Mixer Sense
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = BA_Mixer_Sense_IO.IO_PIN;
  HAL_GPIO_Init(BA_Mixer_Sense_IO.IO_PORT, &GPIO_InitStruct);

  /* Set Priority of External Line Interrupt used for the Mixer Sense*/
  HAL_NVIC_SetPriority(1, 15, 0);											//REMINDER - use correct priority

  /* Enable the External Line Interrupt used for the Mixer Sense*/
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);											//REMINDER - user correct EXTI

  //Initialize Mixer Control
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;		//CORRECT TIM #!
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = BA_Mixer_Control_IO.IO.IO_PIN;
  HAL_GPIO_Init(BA_Mixer_Control_IO.IO.IO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BA_Mixer_Control_IO.IO.IO_PORT, BA_Mixer_Control_IO.IO.IO_PIN, GPIO_PIN_RESET);

  handleMixerPWM.Instance = TIM2;								//CHECK CORRECT TIM #
  handleMixerPWM.Init.Prescaler = 3; //Set to 21 000kHz outside of the audible frequency.
  handleMixerPWM.Init.CounterMode = TIM_COUNTERMODE_UP;
  handleMixerPWM.Init.Period = 999;    //MAX duty cycle
  handleMixerPWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  HAL_StatusTypeDef status = HAL_TIM_Base_Init(&handleMixerPWM);

  TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  status |= HAL_TIM_ConfigClockSource(&handleMixerPWM, &sClockSourceConfig);

  status |= HAL_TIM_PWM_Init(&handleMixerPWM);

  TIM_SlaveConfigTypeDef sSlaveConfig;
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  status |= HAL_TIM_SlaveConfigSynchronization(&handleMixerPWM, &sSlaveConfig);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  status |= HAL_TIMEx_MasterConfigSynchronization(&handleMixerPWM, &sMasterConfig);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  status |= HAL_TIM_PWM_ConfigChannel(&handleMixerPWM, &sConfigOC, BA_Mixer_Control_IO.TIM_CHANNEL);

  return status;
}


/*******************************************************************************
 * Description    : Sets PWM duty cycle for the mixing motor. This function is private.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
static void Mixer_Control_SetPWMDutyCycle(uint16_t newDuty)
{
  __HAL_TIM_SET_COMPARE(&handleMixerPWM, BA_Mixer_Control_IO.TIM_CHANNEL, newDuty);
  MixerCurrentDuty = newDuty;
}


/*******************************************************************************
 * Description    : Turns the Mixing Motor Off.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Mixer_Control_MixerOff()
{
	Mixer_Control_SetPWMDutyCycle(0);
	HAL_TIM_PWM_Stop(&handleMixerPWM, BA_Mixer_Control_IO.TIM_CHANNEL);
}


/*******************************************************************************
 * Description    : Turns the Mixing Motor ON.
 * Input          : Duty Cycle - 0 to 999.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Mixer_Control_MixerOn(uint16_t newDuty)
{
	HAL_TIM_PWM_Start(&handleMixerPWM, BA_Mixer_Control_IO.TIM_CHANNEL);
	Mixer_Control_SetPWMDutyCycle(newDuty);
	Mixer_Control_ClearRevs();
}


/*******************************************************************************
 * Description    : Increments Mixer Motor Rev Counter. Called by HAL_GPIO_EXTI_Callback
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t Mixer_Control_GetRevs()
{
	return MixerRevCounter;
}

/*******************************************************************************
 * Description    : Increments Mixer Motor Rev Counter. Called by HAL_GPIO_EXTI_Callback
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Mixer_Control_ClearRevs()
{
	MixerRevCounter = 0;
}

/*******************************************************************************
 * Description    : Increments Mixer Motor Rev Counter. Called by HAL_GPIO_EXTI_Callback
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Mixer_Control_SenseIT(void)
{
	MixerRevCounter++;
}

//setup handler
void EXTI3_IRQHandler(void)						//REMINDER - use correct handler function
{
	HAL_GPIO_EXTI_IRQHandler(BA_Mixer_Sense_IO.IO_PIN);
}

