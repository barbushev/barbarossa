/*
 * barbarossa_config.h
 *
 *  Created on: Jan 1, 2017
 *      Author: Ted
 */

#ifndef BARBAROSSA_CONFIG_H_
#define BARBAROSSA_CONFIG_H_

#include <Actuator_Control.h>
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "fluid_sensor_cal.h"
#include "fluid_sensor.h"
#include "usbd_cdc_if.h"
#include "MAX31856.h"

#define BA_FIRMWARE_VERSION ((char *)"0001")			//the firmware version is 4 characters
#define BA_FIRMWARE_UPDATE_FLAG	(0xAA)					//this flag can be set at the Backup Register Address below in order to trigger DFU mode on boot.
#define BA_FIRMWARE_UPDATE_ADDR	(0x00)					//used in conjunction with BA_FIRMWARE_UPDATE_FLAG during boot to check if system needs to load DFU.

/*
Determine the desired PWM resolution (for example: 100 steps, 200 steps, 1000…)
Determine the desired PWM frequency (for example: 1kHz, 10kHz, 40kHz, …)
Set the Counter Period at the same value as the resolution
Set the Prescaler = 48000000/(PWM frequency*PWM resolution) – 1
Set the Pulse equal to the desired Pulse width where the value varies from 0 to Counter Period.
 */

#define HEATER_MIN_DUTY_CYCLE 0
#define HEATER_MAX_DUTY_CYCLE 999 //using uint16_t and generic timers are 16 bit
#define HEATER_COUNT 4
#define HEATER_MIN_TEMP (5)
#define HEATER_MAX_TEMP (200)

#define HEAT_TIMER_PRESCALER  (84)
#define HEAT_TIMER_FREQUENCY  (1000)

typedef struct
{
	uint16_t IO_PIN;
    GPIO_TypeDef* IO_PORT;
}ba_io_struct_t;

typedef struct
{
	ba_io_struct_t IO;
    uint32_t adcChannel;
}ba_adc_struct_t;

typedef struct
{
	ba_io_struct_t IO;
	uint32_t TIM_CHANNEL;
}ba_io_with_pwm_t;

typedef struct
{
	ba_io_struct_t I2C_SDA;
	ba_io_struct_t I2C_SCL;
}ba_i2c_struct_t;

typedef struct
{
	ba_io_struct_t LIMA;
	ba_io_struct_t LIMB;
}ba_io_actuatorfb_struct_t;



const ba_io_with_pwm_t BA_Mixer_Control_IO;
const ba_io_struct_t BA_Mixer_Sense_IO;

const ba_io_with_pwm_t BA_HEATER_IO[HEATER_COUNT];
const ba_io_struct_t MAX31856_ChipSelect_IO[HEATER_COUNT];
const ba_io_struct_t BA_Door_Sensor_IO;
const ba_io_struct_t BA_Door_Latch_IO;
const ba_io_struct_t BA_Illumination_Control_IO;



//***************************** IO Configuration for Actuators*************************************
#define BA_HBRIDGE_COUNT 18


const ba_io_actuatorfb_struct_t BA_Actuator_FB_IO[BA_HBRIDGE_COUNT];


#define BA_ADC_SENSOR_COUNT 11

struct BA_HBRIDGE_STRUCT
{
	uint16_t H_BRIDGE_PIN_A;
	GPIO_TypeDef* H_BRIDGE_PORT_A;
	uint16_t H_BRIDGE_PIN_B;
	GPIO_TypeDef* H_BRIDGE_PORT_B;
};

//DECLARATION OF THE PINS USED FOR EACH module. ZERO BASED. PINS ARE DEFINED IN c file
const struct BA_HBRIDGE_STRUCT BA_ACTUATOR[BA_HBRIDGE_COUNT];

const ba_adc_struct_t BA_SENSORS[BA_ADC_SENSOR_COUNT];
const ba_i2c_struct_t BA_DIGIPOT;
const ba_io_struct_t BA_STATUS_LED;
const ba_io_struct_t BA_ERROR_LED;

void BA_LEDs_Init();
void BA_LEDs_Heart_Beat_Set(GPIO_PinState state);
void BA_LEDs_Fault_Set(GPIO_PinState state);


//ERROR enums
typedef enum {
	BA_ERROR_HARDWARE_INIT,		//if any module errors out during initialization - may occur only during bootup
	BA_ERROR_MAX31856_INIT,		//Configuring Chip failed.
	BA_ERROR_MAX31856_READ,		//Reading data from Chip failed.
	BA_ERROR_MAX31856_TMAX,		//Above Max temperature
	BA_ERROR_MAX31856_TMIN,     //Below Min temperature
	BA_ERROR_MAX31856_TOUT,		//Timeout

	BA_ERROR_MCP4661_READ, 			//reading from digipot failed. Parameter = digipot number.
	BA_ERROR_MCP4661_WRITE,			//writing to digipot failed. Parameter = digipot number.
	BA_ERROR_MCP4661_RANGE,			//sensor can't be calibrated within the digipot range. Parameter = digipot number.

	BA_ERROR_AWDCFG,				//analog watch dog config error. Unlikely to occur

	BA_ERROR_L6470_SPI,				//error while communicating to L6470 chips via SPI
	BA_ERROR_RESET_IWDG,			//independent watch dog caused a reset
	BA_ERROR_RESET_LOWP,			//reset caused by low power
	BA_ERROR_LSE_FAIL,				//Low speed oscillator failure

	BA_ERROR_FLASH_ERASE,           //Flash erase error
	BA_ERROR_FLASH_PROGRAM,			//Flash programming error
	BA_ERROR_FLASH_SIZE,			//No available Flash memory size for the binary file
	BA_ERROR_FLASH_NOFILE,			//The binary file is not available

	BA_ERROR_SDCARD_INIT,			//error while initializing SD card
	BA_ERROR_SDCARD_DLOG,			//error while Data logging
	BA_ERROR_SDCARD_ELOG,			//error while Error logging
	BA_ERROR_SDCARD_FLOW,			//error while loading a Flow (sequence)
	BA_ERROR_SDCARD_DREAD,			//error while reading a directory
	BA_ERROR_SDCARD_FREAD,			//error while reading a file
	BA_ERROR_SDCARD_FWRITE			//error while writing a file
} ba_error_types;

typedef struct
{
	uint32_t TimeStamp;
	uint32_t ErrorNum;
	int32_t	 ErrorParam;
}fs_errorlog_struct_t;

int32_t BA_StepperMotorOffset[8];	//used for stepper motor calibration. The first move away from the home sensor is executed with this offset.



void BA_ERROR_HANLDER(ba_error_types ErrorCode, int32_t ErrorParam);


#endif /* BARBAROSSA_CONFIG_H_ */
