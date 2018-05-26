/*
 * Illumination_Control.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Ted
 */

#include "Illumination_Control.h"

/*******************************************************************************
 * Description    : Initializes IO for Illumination Contrl.
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Illumination_Control_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = BA_Illumination_Control_IO.IO_PIN;

  HAL_GPIO_Init(BA_Illumination_Control_IO.IO_PORT, &GPIO_InitStruct);
  Illumination_Control_Disable();
}

/*******************************************************************************
 * Description    : Turns Illumination On.
 * Input          : none.
 * Output         : None.
 * Return         : none.
 *******************************************************************************/
void Illumination_Control_Enable(void)
{
	HAL_GPIO_WritePin(BA_Illumination_Control_IO.IO_PORT, BA_Illumination_Control_IO.IO_PIN, GPIO_PIN_RESET);
}

/*******************************************************************************
 * Description    : Turns Illumination Off.
 * Input          : none.
 * Output         : None.
 * Return         : none.
 *******************************************************************************/
void Illumination_Control_Disable(void)
{
	HAL_GPIO_WritePin(BA_Illumination_Control_IO.IO_PORT, BA_Illumination_Control_IO.IO_PIN, GPIO_PIN_SET);
}
