/*
 * Fan_Control.c
 *
 *  Created on: Feb 21, 2017
 *      Author: Ted
 */
#include "Fan_Control.h"


#define Fans_Enable_Pin 	(GPIO_PIN_2)
#define Fans_Enable_Port 	(GPIOE)

/*******************************************************************************
 * Description    : Initialize the IO for cooling Fan
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Fan_Control_Init()
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /*Configure GPIO pin : PF6 */
	  GPIO_InitStruct.Pin = Fans_Enable_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(Fans_Enable_Port, &GPIO_InitStruct);

	  //set the pin to low
	  HAL_GPIO_WritePin(Fans_Enable_Port, Fans_Enable_Pin, GPIO_PIN_RESET);

	  //setup Timer for Input Capture Counter

}


/*******************************************************************************
 * Description    : Read the RPM from one of the fans.
 * Input          : Fan Number - 0 or 1
 * Output         : None.
 * Return         : The number of RPM
 *******************************************************************************/
uint16_t Fan_Control_GetRPM(uint8_t FanNum)
{
	return 0;
}


/*******************************************************************************
 * Description    : Turns the Fans on.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Fan_Control_On()
{
	 HAL_GPIO_WritePin(Fans_Enable_Port, Fans_Enable_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
 * Description    : Turns the Fans off.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Fan_Control_Off()
{
	 HAL_GPIO_WritePin(Fans_Enable_Port, Fans_Enable_Pin, GPIO_PIN_RESET);
}

