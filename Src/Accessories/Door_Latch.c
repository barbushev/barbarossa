/*
 * Door_Latch.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#include "Door_Latch.h"

/*******************************************************************************
 * Description    : Initializes IO for Door Latch.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Door_Latch_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;				//why because Tom said so - it's being driven
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = BA_Door_Latch_IO.IO_PIN;

  HAL_GPIO_Init(BA_Door_Latch_IO.IO_PORT, &GPIO_InitStruct);
  Door_Latch_Unlock();
}

/*******************************************************************************
 * Description    : Locks the door.
 * Input          : none.
 * Output         : None.
 * Return         : none.
 *******************************************************************************/
void Door_Latch_Lock(void)
{
	HAL_GPIO_WritePin(BA_Door_Latch_IO.IO_PORT, BA_Door_Latch_IO.IO_PIN, GPIO_PIN_SET);
}

/*******************************************************************************
 * Description    : Unlocks the door.
 * Input          : none.
 * Output         : None.
 * Return         : none.
 *******************************************************************************/
void Door_Latch_Unlock(void)
{
	HAL_GPIO_WritePin(BA_Door_Latch_IO.IO_PORT, BA_Door_Latch_IO.IO_PIN, GPIO_PIN_RESET);
}
