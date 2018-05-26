/*
 * actuator_io.c
 *
 *  Created on: Dec 19, 2016
 *      Author: Ted
 */

#include <Actuator_Control.h>

//private functions
static void control_actuators(int32_t *, uint8_t, uint8_t);


/*
 * Initialize the H-Bridge and Feedback IO
 */
void ACTUATOR_INIT()
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	 for (int i = 0; i < BA_HBRIDGE_COUNT; i++)
 	 {
		 //initialize H-Bridge IO
 		 GPIO_InitStruct.Pin = BA_ACTUATOR[i].H_BRIDGE_PIN_A;
 		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 		 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
 		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 		 HAL_GPIO_Init(BA_ACTUATOR[i].H_BRIDGE_PORT_A, &GPIO_InitStruct);

 		 GPIO_InitStruct.Pin = BA_ACTUATOR[i].H_BRIDGE_PIN_B;
 		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 		 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
 		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 		 HAL_GPIO_Init(BA_ACTUATOR[i].H_BRIDGE_PORT_B, &GPIO_InitStruct);

 		 //initialize Feedback IO
 		 GPIO_InitStruct.Pin = BA_Actuator_FB_IO[i].LIMA.IO_PIN;
 		 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 		 GPIO_InitStruct.Pull = GPIO_NOPULL;
 		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 		 HAL_GPIO_Init(BA_Actuator_FB_IO[i].LIMA.IO_PORT, &GPIO_InitStruct);

 		 GPIO_InitStruct.Pin = BA_Actuator_FB_IO[i].LIMB.IO_PIN;
 		 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 		 GPIO_InitStruct.Pull = GPIO_NOPULL;
 		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 		 HAL_GPIO_Init(BA_Actuator_FB_IO[i].LIMB.IO_PORT, &GPIO_InitStruct);
 	 }

	  //Set output levels
	 int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
	 ACTUATOR_RETRACT(list_of_actuators, 18);
}

void ACTUATOR_EXTEND(int32_t *list_of_actuators, uint8_t count)
{
	control_actuators(list_of_actuators, count, 1);
}

void ACTUATOR_RETRACT(int32_t *list_of_actuators, uint8_t count)
{
	control_actuators(list_of_actuators, count, 0);
}

static void control_actuators(int32_t *list_of_actuators, uint8_t count, uint8_t Direction)
{
	uint8_t state1 = 0;
	uint8_t state2 = 1;

	if (Direction)
	{
		state1 = 1;
		state2 = 0;
	}

	for (uint8_t i = 0; i < count; i++)
	{
		uint8_t hbridge_num = list_of_actuators[i] - 1; // -1 because struct is zero based

		HAL_GPIO_WritePin(BA_ACTUATOR[hbridge_num].H_BRIDGE_PORT_A, BA_ACTUATOR[hbridge_num].H_BRIDGE_PIN_A, state1);
		HAL_GPIO_WritePin(BA_ACTUATOR[hbridge_num].H_BRIDGE_PORT_B, BA_ACTUATOR[hbridge_num].H_BRIDGE_PIN_B, state2);
	}
}





uint8_t Actuator_AreExtended(int32_t *list_of_actuators, uint8_t count)
{
	uint8_t LIMA = 0, LIMB = 0;

	for (uint8_t i = 0; i < count; i++)
		{
			uint8_t actuatorNum = list_of_actuators[i] - 1; // -1 because struct is zero based

			//Extended - LIMA - high, LIMB - low
			LIMA += (HAL_GPIO_ReadPin(BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PORT, BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PIN) == GPIO_PIN_SET);
			LIMB += (HAL_GPIO_ReadPin(BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PORT, BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PIN) == GPIO_PIN_RESET);
		}

	return ((LIMA == count)  && (LIMB == count));
}

uint8_t Actuator_AreRetracted(int32_t *list_of_actuators, uint8_t count)
{
	uint8_t LIMA = 0, LIMB = 0;

	for (uint8_t i = 0; i < count; i++)
		{
			uint8_t actuatorNum = list_of_actuators[i] - 1; // -1 because struct is zero based

			//Retracted - LIMA - low, LIMB - high
			LIMA += (HAL_GPIO_ReadPin(BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PORT, BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PIN) == GPIO_PIN_RESET);
			LIMB += (HAL_GPIO_ReadPin(BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PORT, BA_Actuator_FB_IO[actuatorNum].LIMA.IO_PIN) == GPIO_PIN_SET);
		}

	return ((LIMA == count)  && (LIMB == count));
}


/*
LIMA - default is high
LIMB - default is high

if both are low, PCA failure
if both are high, motor failure

configure as no pull.
*/
