/*
 * Door_Sensor.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#include "Door_Sensor.h"

/*******************************************************************************
 * Description    : Initializes IO for Door Sensor.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Door_Sensor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;				//why because Tom said so - it's being driven
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = BA_Door_Sensor_IO.IO_PIN;

  HAL_GPIO_Init(BA_Door_Sensor_IO.IO_PORT, &GPIO_InitStruct);
}


/*******************************************************************************
 * Description    : Reads the status of the door sensor.
 * Input          : none.
 * Output         : None.
 * Return         : 0 = open, 1 = closed.
 *******************************************************************************/
uint8_t Door_Sensor_GetState()
{
	return HAL_GPIO_ReadPin(BA_Door_Sensor_IO.IO_PORT, BA_Door_Sensor_IO.IO_PIN);
}
