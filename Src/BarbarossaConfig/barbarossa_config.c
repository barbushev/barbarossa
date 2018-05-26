/*
 * barbarossa_config.c
 *
 *  Created on: Jan 1, 2017
 *      Author: Ted
 */

#include <barbarossa_config.h>
#include "FileSystem_Task.h"


#include "USB_Task.h"  //used for sending error over usb;


int32_t BA_StepperMotorOffset[8] = {0,0,0,0,0,0,0,0};


//***********************************************I2C config for Digipots*****************************************//
const ba_i2c_struct_t BA_DIGIPOT =
{
		.I2C_SDA.IO_PIN = GPIO_PIN_0,
		.I2C_SDA.IO_PORT = GPIOF,

		.I2C_SCL.IO_PIN  = GPIO_PIN_1,
		.I2C_SCL.IO_PORT = GPIOF,
};
//***********************************************I2C config for Digipots*****************************************//


//***********************************************IO for Door Sensor*****************************************//
const ba_io_struct_t BA_Illumination_Control_IO =
{
		.IO_PIN = GPIO_PIN_7,
		.IO_PORT = GPIOE,
};
//***********************************************IO for Door Sensor*****************************************//


//***********************************************IO for Door Sensor*****************************************//
const ba_io_struct_t BA_Door_Latch_IO =
{
		.IO_PIN = GPIO_PIN_9,
		.IO_PORT = GPIOE,
};
//***********************************************IO for Door Sensor*****************************************//


//***********************************************IO for Door Sensor*****************************************//
const ba_io_struct_t BA_Door_Sensor_IO =
{
		.IO_PIN = GPIO_PIN_10,
		.IO_PORT = GPIOE,
};
//***********************************************IO for Door Sensor*****************************************//



//***********************************************IO for Mixer Control with PWM*****************************************//
const ba_io_with_pwm_t BA_Mixer_Control_IO =
{
		.IO.IO_PIN = GPIO_PIN_11,
		.IO.IO_PORT = GPIOB,
		.TIM_CHANNEL = TIM_CHANNEL_4,
};

const ba_io_struct_t BA_Mixer_Sense_IO =
{
		.IO_PIN = GPIO_PIN_3,
		.IO_PORT = GPIOE,
};
//***********************************************IO for Mixer Control with PWM*****************************************//





//*******************************************************HEATER CONFIG*******************************************//
//USING TIM4 for HEATER PWM: PD12 - htr0, PD13 - htr1, PD14 - htr2, PD15 - htr3
const ba_io_with_pwm_t BA_HEATER_IO[HEATER_COUNT] =
{
		[0].IO.IO_PIN = GPIO_PIN_12,
		[0].IO.IO_PORT = GPIOD,
		[0].TIM_CHANNEL = TIM_CHANNEL_1,

		[1].IO.IO_PIN = GPIO_PIN_13,
		[1].IO.IO_PORT = GPIOD,
		[1].TIM_CHANNEL = TIM_CHANNEL_2,

		[2].IO.IO_PIN = GPIO_PIN_14,
		[2].IO.IO_PORT = GPIOD,
		[2].TIM_CHANNEL = TIM_CHANNEL_3,

		[3].IO.IO_PIN = GPIO_PIN_15,
		[3].IO.IO_PORT = GPIOD,
		[3].TIM_CHANNEL = TIM_CHANNEL_4,
};

//***************************************************END HEATER CONFIG*******************************************//



//***************************************************MAX31856 IO CONFIG*******************************************//

const ba_io_struct_t MAX31856_ChipSelect_IO[HEATER_COUNT] =
{
		[0].IO_PIN = GPIO_PIN_15,
		[0].IO_PORT = GPIOA,

		[1].IO_PIN = GPIO_PIN_8,
		[1].IO_PORT = GPIOB,

		[2].IO_PIN = GPIO_PIN_9,
		[2].IO_PORT = GPIOB,

		[3].IO_PIN = GPIO_PIN_10,
		[3].IO_PORT = GPIOB,
};
//***************************************************END MAX31856 CONFIG*******************************************//







//*****************************************************ACTUATOR CONFIG*******************************************//
const struct BA_HBRIDGE_STRUCT BA_ACTUATOR[BA_HBRIDGE_COUNT] =
{
	[0].H_BRIDGE_PIN_A  = GPIO_PIN_3,  	//PK3	ACTUATOR0_F
	[0].H_BRIDGE_PORT_A = GPIOK,
	[0].H_BRIDGE_PIN_B  = GPIO_PIN_4,  	//PK4	ACTUATOR0_R
	[0].H_BRIDGE_PORT_B = GPIOK,

	[1].H_BRIDGE_PIN_A  = GPIO_PIN_5,	//PK5 	ACTUATOR1_F
	[1].H_BRIDGE_PORT_A = GPIOK,
	[1].H_BRIDGE_PIN_B  = GPIO_PIN_6,   //PK6 	ACTUATOR1_R
	[1].H_BRIDGE_PORT_B = GPIOK,

	[2].H_BRIDGE_PIN_A  = GPIO_PIN_0,	//PG0	ACTUATOR2_F
	[2].H_BRIDGE_PORT_A = GPIOG,
	[2].H_BRIDGE_PIN_B  = GPIO_PIN_1,	//PG1	ACTUATOR2_R
	[2].H_BRIDGE_PORT_B = GPIOG,

	[3].H_BRIDGE_PIN_A  = GPIO_PIN_2,	//PG2	ACTUATOR3_F
	[3].H_BRIDGE_PORT_A = GPIOG,
	[3].H_BRIDGE_PIN_B  = GPIO_PIN_3,	//PG3	ACTUATOR3_R
	[3].H_BRIDGE_PORT_B = GPIOG,

	[4].H_BRIDGE_PIN_A  = GPIO_PIN_4,	//PG4	ACTUATOR4_F
	[4].H_BRIDGE_PORT_A = GPIOG,
	[4].H_BRIDGE_PIN_B  = GPIO_PIN_5,	//PG5	ACTUATOR4_R
	[4].H_BRIDGE_PORT_B = GPIOG,

	[5].H_BRIDGE_PIN_A  = GPIO_PIN_6,	//PG6	ACTUATOR5_F
	[5].H_BRIDGE_PORT_A = GPIOG,
	[5].H_BRIDGE_PIN_B  = GPIO_PIN_7,	//PG7	ACTUATOR5_R
	[5].H_BRIDGE_PORT_B = GPIOG,

	[6].H_BRIDGE_PIN_A  = GPIO_PIN_8,	//PG8	ACTUATOR6_F
	[6].H_BRIDGE_PORT_A = GPIOG,
	[6].H_BRIDGE_PIN_B  = GPIO_PIN_9,	//PG9	ACTUATOR6_R
	[6].H_BRIDGE_PORT_B = GPIOG,

	[7].H_BRIDGE_PIN_A  = GPIO_PIN_10,	//PG10	ACTUATOR7_F
	[7].H_BRIDGE_PORT_A = GPIOG,
	[7].H_BRIDGE_PIN_B  = GPIO_PIN_11,	//PG11	ACTUATOR7_R
	[7].H_BRIDGE_PORT_B = GPIOG,

	[8].H_BRIDGE_PIN_A  = GPIO_PIN_12,	//PG12	ACTUATOR8_F
	[8].H_BRIDGE_PORT_A = GPIOG,
	[8].H_BRIDGE_PIN_B  = GPIO_PIN_13,	//PG13	ACTUATOR8_R
	[8].H_BRIDGE_PORT_B = GPIOG,

	[9].H_BRIDGE_PIN_A  = GPIO_PIN_14,	//PG14	ACTUATOR9_F
	[9].H_BRIDGE_PORT_A = GPIOG,
	[9].H_BRIDGE_PIN_B  = GPIO_PIN_15,	//PG15	ACTUATOR9_R
	[9].H_BRIDGE_PORT_B = GPIOG,

	[10].H_BRIDGE_PIN_A  = GPIO_PIN_0,	//PD0	ACTUATOR10_F
	[10].H_BRIDGE_PORT_A = GPIOD,
	[10].H_BRIDGE_PIN_B  = GPIO_PIN_1,	//PD1	ACTUATOR10_R
	[10].H_BRIDGE_PORT_B = GPIOD,

	[11].H_BRIDGE_PIN_A  = GPIO_PIN_4,	//PD4	ACTUATOR11_F
	[11].H_BRIDGE_PORT_A = GPIOD,
	[11].H_BRIDGE_PIN_B  = GPIO_PIN_5,	//PD5	ACTUATOR11_R
	[11].H_BRIDGE_PORT_B = GPIOD,

	[12].H_BRIDGE_PIN_A  = GPIO_PIN_6,	//PD6	ACTUATOR12_F
	[12].H_BRIDGE_PORT_A = GPIOD,
	[12].H_BRIDGE_PIN_B  = GPIO_PIN_7,	//PD7	ACTUATOR12_R
	[12].H_BRIDGE_PORT_B = GPIOD,

	[13].H_BRIDGE_PIN_A  = GPIO_PIN_8,	//PD8	ACTUATOR13_F
	[13].H_BRIDGE_PORT_A = GPIOD,
	[13].H_BRIDGE_PIN_B  = GPIO_PIN_9,	//PD9	ACTUATOR13_R
	[13].H_BRIDGE_PORT_B = GPIOD,

	[14].H_BRIDGE_PIN_A  = GPIO_PIN_10,	//PD10	ACTUATOR14_F
	[14].H_BRIDGE_PORT_A = GPIOD,
	[14].H_BRIDGE_PIN_B  = GPIO_PIN_11,	//PD11	ACTUATOR14_R
	[14].H_BRIDGE_PORT_B = GPIOD,

	[15].H_BRIDGE_PIN_A  = GPIO_PIN_12,	//PF12	ACTUATOR15_F
	[15].H_BRIDGE_PORT_A = GPIOF,
	[15].H_BRIDGE_PIN_B  = GPIO_PIN_13,	//PF13	ACTUATOR15_R
	[15].H_BRIDGE_PORT_B = GPIOF,

	[16].H_BRIDGE_PIN_A  = GPIO_PIN_14,	//PF14	ACTUATOR16_F
	[16].H_BRIDGE_PORT_A = GPIOF,
	[16].H_BRIDGE_PIN_B  = GPIO_PIN_15, //PF15	ACTUATOR16_R
	[16].H_BRIDGE_PORT_B = GPIOF,

	[17].H_BRIDGE_PIN_A  = GPIO_PIN_6,	//PC6	ACTUATOR17_F
	[17].H_BRIDGE_PORT_A = GPIOC,
	[17].H_BRIDGE_PIN_B  = GPIO_PIN_7,	//PC7	ACTUATOR17_R
	[17].H_BRIDGE_PORT_B = GPIOC,
};


///ACTUATOR FEEDBACK CONFIG
const ba_io_actuatorfb_struct_t BA_Actuator_FB_IO[BA_HBRIDGE_COUNT] =
{
	[0].LIMA.IO_PIN	 = GPIO_PIN_0,  //PJ0 ACTUATOR1_LIMA
	[0].LIMA.IO_PORT = GPIOJ,
	[0].LIMB.IO_PIN	 = GPIO_PIN_1,	//PJ1 ACTUATOR1_LIMB
	[0].LIMB.IO_PORT = GPIOJ,

	[1].LIMA.IO_PIN	 = GPIO_PIN_2,  //PJ2 ACTUATOR2_LIMA
	[1].LIMA.IO_PORT = GPIOJ,
	[1].LIMB.IO_PIN	 = GPIO_PIN_3,	//PJ3 ACTUATOR2_LIMB
	[1].LIMB.IO_PORT = GPIOJ,

	[2].LIMA.IO_PIN	 = GPIO_PIN_4,  //PJ4 ACTUATOR3_LIMA
	[2].LIMA.IO_PORT = GPIOJ,
	[2].LIMB.IO_PIN	 = GPIO_PIN_5,	//PJ5 ACTUATOR3_LIMB
	[2].LIMB.IO_PORT = GPIOJ,

	[3].LIMA.IO_PIN	 = GPIO_PIN_6,  //PJ6 ACTUATOR4_LIMA
	[3].LIMA.IO_PORT = GPIOJ,
	[3].LIMB.IO_PIN	 = GPIO_PIN_7,	//PJ7 ACTUATOR4_LIMB
	[3].LIMB.IO_PORT = GPIOJ,

	[4].LIMA.IO_PIN	 = GPIO_PIN_8,  //PJ8 ACTUATOR5_LIMA
	[4].LIMA.IO_PORT = GPIOJ,
	[4].LIMB.IO_PIN	 = GPIO_PIN_9,	//PJ9 ACTUATOR5_LIMB
	[4].LIMB.IO_PORT = GPIOJ,

	[5].LIMA.IO_PIN	 = GPIO_PIN_10,  //PJ10 ACTUATOR6_LIMA
	[5].LIMA.IO_PORT = GPIOJ,
	[5].LIMB.IO_PIN	 = GPIO_PIN_11,	 //PJ11 ACTUATOR6_LIMB
	[5].LIMB.IO_PORT = GPIOJ,

	[6].LIMA.IO_PIN	 = GPIO_PIN_12,  //PJ12 ACTUATOR7_LIMA
	[6].LIMA.IO_PORT = GPIOJ,
	[6].LIMB.IO_PIN	 = GPIO_PIN_13,	 //PJ13 ACTUATOR7_LIMB
	[6].LIMB.IO_PORT = GPIOJ,

	[7].LIMA.IO_PIN	 = GPIO_PIN_14,  //PJ14 ACTUATOR8_LIMA
	[7].LIMA.IO_PORT = GPIOJ,
	[7].LIMB.IO_PIN	 = GPIO_PIN_15,  //PJ15 ACTUATOR8_LIMB
	[7].LIMB.IO_PORT = GPIOJ,

	[8].LIMA.IO_PIN	 = GPIO_PIN_0,  //PI0 ACTUATOR9_LIMA
	[8].LIMA.IO_PORT = GPIOI,
	[8].LIMB.IO_PIN	 = GPIO_PIN_1,	//PI1 ACTUATOR9_LIMB
	[8].LIMB.IO_PORT = GPIOI,

	[9].LIMA.IO_PIN	 = GPIO_PIN_2,  //PI2 ACTUATOR10_LIMA
	[9].LIMA.IO_PORT = GPIOI,
	[9].LIMB.IO_PIN	 = GPIO_PIN_3,	//PI3 ACTUATOR10_LIMB
	[9].LIMB.IO_PORT = GPIOI,

	[10].LIMA.IO_PIN  = GPIO_PIN_4,  //PI4 ACTUATOR11_LIMA
	[10].LIMA.IO_PORT = GPIOI,
	[10].LIMB.IO_PIN  = GPIO_PIN_5,	 //PI5 ACTUATOR11_LIMB
	[10].LIMB.IO_PORT = GPIOI,

	[11].LIMA.IO_PIN  = GPIO_PIN_6,  //PI6 ACTUATOR12_LIMA
	[11].LIMA.IO_PORT = GPIOI,
	[11].LIMB.IO_PIN  = GPIO_PIN_7,	 //PI7 ACTUATOR12_LIMB
	[11].LIMB.IO_PORT = GPIOI,

	[12].LIMA.IO_PIN  = GPIO_PIN_8,  //PI8 ACTUATOR13_LIMA
	[12].LIMA.IO_PORT = GPIOI,
	[12].LIMB.IO_PIN  = GPIO_PIN_9,	 //PI9 ACTUATOR13_LIMB
	[12].LIMB.IO_PORT = GPIOI,

	[13].LIMA.IO_PIN  = GPIO_PIN_10,  //PI10 ACTUATOR14_LIMA
	[13].LIMA.IO_PORT = GPIOI,
	[13].LIMB.IO_PIN  = GPIO_PIN_11,  //PI11 ACTUATOR14_LIMB
	[13].LIMB.IO_PORT = GPIOI,

	[14].LIMA.IO_PIN  = GPIO_PIN_12,  //PI12 ACTUATOR15_LIMA
	[14].LIMA.IO_PORT = GPIOI,
	[14].LIMB.IO_PIN  = GPIO_PIN_13,  //PI13 ACTUATOR15_LIMB
	[14].LIMB.IO_PORT = GPIOI,

	[15].LIMA.IO_PIN  = GPIO_PIN_14,  //PI14 ACTUATOR16_LIMA
	[15].LIMA.IO_PORT = GPIOI,
	[15].LIMB.IO_PIN  = GPIO_PIN_15,  //PI15 ACTUATOR16_LIMB
	[15].LIMB.IO_PORT = GPIOI,

	[16].LIMA.IO_PIN  = GPIO_PIN_0,  //PK0 ACTUATOR17_LIMA
	[16].LIMA.IO_PORT = GPIOK,
	[16].LIMB.IO_PIN  = GPIO_PIN_1,  //PK1 ACTUATOR17_LIMB
	[16].LIMB.IO_PORT = GPIOK,

	[17].LIMA.IO_PIN  = GPIO_PIN_2,  //PK2 ACTUATOR18_LIMA
	[17].LIMA.IO_PORT = GPIOK,
	[17].LIMB.IO_PIN  = GPIO_PIN_3,  //PK3 ACTUATOR18_LIMB
	[17].LIMB.IO_PORT = GPIOK,
};

























//***************************************************END ACTUATOR CONFIG*******************************************//


const ba_adc_struct_t BA_SENSORS[BA_ADC_SENSOR_COUNT] =
{
	[0].IO.IO_PIN  = GPIO_PIN_0,	//PC0 ADC3_IN10
	[0].IO.IO_PORT = GPIOC,
	[0].adcChannel = ADC_CHANNEL_10,

	[1].IO.IO_PIN = GPIO_PIN_1,		//PC1 ADC3_IN11
	[1].IO.IO_PORT = GPIOC,
	[1].adcChannel = ADC_CHANNEL_11,

	[2].IO.IO_PIN = GPIO_PIN_2,		//PC2 ADC3_IN12
	[2].IO.IO_PORT = GPIOC,
	[2].adcChannel = ADC_CHANNEL_12,

	[3].IO.IO_PIN = GPIO_PIN_3,		//PF3 ADC3_IN9
	[3].IO.IO_PORT = GPIOF,
	[3].adcChannel = ADC_CHANNEL_3,

	[4].IO.IO_PIN = GPIO_PIN_4,		//PF4 ADC3_IN14
	[4].IO.IO_PORT = GPIOF,
	[4].adcChannel = ADC_CHANNEL_14,

	[5].IO.IO_PIN = GPIO_PIN_5,		//PF5 ADC3_IN15
	[5].IO.IO_PORT = GPIOF,
	[5].adcChannel = ADC_CHANNEL_15,

	[6].IO.IO_PIN = GPIO_PIN_6,		//PF6 ADC3_IN4
	[6].IO.IO_PORT = GPIOF,
	[6].adcChannel = ADC_CHANNEL_4,

	[7].IO.IO_PIN = GPIO_PIN_7,		//PF7 ADC3_IN5
	[7].IO.IO_PORT = GPIOF,
	[7].adcChannel = ADC_CHANNEL_5,

	[8].IO.IO_PIN = GPIO_PIN_8,		//PF8 ADC3_IN6
	[8].IO.IO_PORT = GPIOF,
	[8].adcChannel = ADC_CHANNEL_6,

	[9].IO.IO_PIN = GPIO_PIN_9,		//PF9 ADC3_IN7
	[9].IO.IO_PORT = GPIOF,
	[9].adcChannel = ADC_CHANNEL_7,

	[10].IO.IO_PIN = GPIO_PIN_10,	//PF10 ADC3_IN8
	[10].IO.IO_PORT = GPIOF,
	[10].adcChannel = ADC_CHANNEL_8,
};







const ba_io_struct_t BA_Heart_Beat_LED_IO = { GPIO_PIN_14, GPIOB };
const ba_io_struct_t BA_Fault_LED_IO  	  = { GPIO_PIN_15, GPIOB };


/*******************************************************************************
 * Description    : Initializes IO for Heart Beat and Fault LEDs
 * Input          : none.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void BA_LEDs_Init()
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	  GPIO_InitStruct.Pin = BA_Heart_Beat_LED_IO.IO_PIN;
	  HAL_GPIO_Init( BA_Heart_Beat_LED_IO.IO_PORT, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = BA_Fault_LED_IO.IO_PIN;
	  HAL_GPIO_Init( BA_Fault_LED_IO.IO_PORT, &GPIO_InitStruct);


	  //Set default states
	  HAL_GPIO_WritePin(BA_Heart_Beat_LED_IO.IO_PORT, BA_Heart_Beat_LED_IO.IO_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(BA_Fault_LED_IO.IO_PORT, BA_Fault_LED_IO.IO_PIN, GPIO_PIN_RESET);
}


/*******************************************************************************
 * Description    : Heart Beat LED Set State
 * Input          : State - GPIO_PIN_SET or GPIO_PIN_RESET.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void BA_LEDs_Heart_Beat_Set(GPIO_PinState state)
{
	HAL_GPIO_WritePin(BA_Heart_Beat_LED_IO.IO_PORT, BA_Heart_Beat_LED_IO.IO_PIN, state);
}

/*******************************************************************************
 * Description    : Fault LED Set State
 * Input          : State - GPIO_PIN_SET or GPIO_PIN_RESET.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void BA_LEDs_Fault_Set(GPIO_PinState state)
{
	HAL_GPIO_WritePin(BA_Fault_LED_IO.IO_PORT, BA_Fault_LED_IO.IO_PIN, state);
}




/*******************************************************************************
 * Description    : Error Handler
 * Input          : ErrorCode and ErrorParam
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void BA_ERROR_HANLDER(ba_error_types ErrorCode, int32_t ErrorParam)
{
	// int32 - may have up to 10 digits - 2,147,483,647
	char buff[14]; //needs to fit Error_types enum + 10 digits + null char. Error_types shouldn't be more than 3 digits. 3 + 10 + 1= 14
	snprintf(buff, sizeof(buff), "%u|%ld", ErrorCode, ErrorParam);

	//send the error code and errorParam separated by a |
	USB_SendFromTask(0, BA_msgID_Error, buff);

	switch (ErrorCode)
	{
	case BA_ERROR_HARDWARE_INIT:	//generic init error - can only occur during initialization
		{
			//NOTE it is possible that USB and SD card have not yet been initialized!
			//This should probably just light an LED
			//USB_SendFromTask(0, "BA_ERROR_HARDWARE_INIT\r\n");

			break;
		}


	case BA_ERROR_MAX31856_INIT:
		{
			//ErrorParam relates to the number of heater (0-3)
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MAX31856_INIT\r\n");
			break;
		}

	case BA_ERROR_MAX31856_READ:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MAX31856_READ\r\n");
			break;
		}

	case BA_ERROR_MAX31856_TMAX:
		{


			//USB_SendFromTask(0, BA_msgID_Error, buff);
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MAX31856_TMAX\r\n");
			break;
		}

	case BA_ERROR_MAX31856_TMIN:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MAX31856_TMIN\r\n");
			break;
		}

	case BA_ERROR_MAX31856_TOUT:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MAX31856_TOUT\r\n");
			break;
		}

	case BA_ERROR_MCP4661_READ: //reading from digipot failed. Parameter = digipot number.
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MCP4661_READ\r\n");
			break;
		}

	case BA_ERROR_MCP4661_WRITE: //writing to digipot failed. Parameter = digipot number.
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MCP4661_WRITE\r\n");
			break;
		}

	case BA_ERROR_MCP4661_RANGE: //sensor can't be calibrated within the digipot range. Parameter = digipot number.
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_MCP4661_RANGE\r\n");

			break;
		}

	case BA_ERROR_AWDCFG:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_AWDCFG\r\n");
			break;
		}

	case BA_ERROR_L6470_SPI: //error while communicating to L6470 chips via SPI
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_L6470_SPI\r\n");
			break;
		}

	case BA_ERROR_RESET_IWDG:
		{

			break;
		}

	case BA_ERROR_RESET_LOWP:
		{

			break;
		}

	case BA_ERROR_LSE_FAIL:		//low speed oscillator failure
		{

			break;
		}

	case BA_ERROR_FLASH_ERASE:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_FLASH_ERASE");
			break;
		}

	case BA_ERROR_FLASH_PROGRAM:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_FLASH_PROGRAM");
			break;
		}

	case BA_ERROR_FLASH_SIZE:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_FLASH_SIZE");
			break;
		}

	case BA_ERROR_FLASH_NOFILE:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_FLASH_NOFILE");
			break;
		}

	case BA_ERROR_SDCARD_INIT:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_SDCARD_INIT\r\n");
			break;
		}

	case BA_ERROR_SDCARD_FLOW:
		{
			//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_SDCARD_FLOW\r\n");
			break;
		}

	case BA_ERROR_SDCARD_DLOG:
		{
			//USB_SendFromTask(0, BA_msgID_Error,"BA_ERROR_SDCARD_DLOG\r\n");
			break;
		}

	case BA_ERROR_SDCARD_ELOG:
		{
			//USB_SendFromTask(0, BA_msgID_Error,"BA_ERROR_SDCARD_ELOG\r\n");
			break;
		}

	case BA_ERROR_SDCARD_DREAD:
			{
				//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_SDCARD_DREAD\r\n");
				break;
			}

	case BA_ERROR_SDCARD_FREAD:
			{
				//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_SDCARD_FREAD\r\n");
				break;
			}

	case BA_ERROR_SDCARD_FWRITE:
			{
				//USB_SendFromTask(0, BA_msgID_Error, "BA_ERROR_SDCARD_FWRITE\r\n");
				break;
			}

	default:	//generic error?
		{

			break;
		}
	}


//Log it to file unless it was a BA_ERROR_SDCARD related - might want to do this per case instead

if (ErrorCode < BA_ERROR_SDCARD_INIT)
	{
	fs_errorlog_struct_t ErrorToLog;
	ErrorToLog.TimeStamp = xTaskGetTickCount();
	ErrorToLog.ErrorNum = ErrorCode;
	ErrorToLog.ErrorParam = ErrorParam;
	FileSystem_LogError(&ErrorToLog);
	}
}
