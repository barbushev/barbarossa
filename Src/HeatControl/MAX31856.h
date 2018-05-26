/*
 * MAX31856.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Ted Barbushev
 */

#ifndef APPLICATION_USER_MAX31856_MAX31856_H_
#define APPLICATION_USER_MAX31856_MAX31856_H_

#include <barbarossa_config.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_spi.h"
#include "math.h" //used for trunc and abs - may be removed after debugging

void MAX31856_InitSPI();
HAL_StatusTypeDef MAX31856_GetTCTemp(uint8_t ChipID, float *ReturnValue);
HAL_StatusTypeDef MAX31856_GetFaultMask(uint8_t ChipID, uint8_t *ReturnValue);
HAL_StatusTypeDef MAX31856_SetFaultMask(uint8_t ChipID, uint8_t *data);
HAL_StatusTypeDef MAX31856_GetFaultStatus(uint8_t ChipID, uint8_t *ReturnValue);

HAL_StatusTypeDef MAX31856_GetConfigReg0(uint8_t ChipID, uint8_t *ReturnValue);
HAL_StatusTypeDef MAX31856_SetConfigReg0(uint8_t ChipID, uint8_t *data);
HAL_StatusTypeDef MAX31856_GetConfigReg1(uint8_t ChipID, uint8_t *ReturnValue);
HAL_StatusTypeDef MAX31856_SetConfigReg1(uint8_t ChipID, uint8_t *data);

HAL_StatusTypeDef MAX31856_SetColdJunctionHFT(uint8_t ChipID, int8_t *Temperature);	//HFT - High Fault Threshold
HAL_StatusTypeDef MAX31856_GetColdJunctionHFT(uint8_t ChipID, int8_t *ReturnValue);
HAL_StatusTypeDef MAX31856_SetColdJunctionLFT(uint8_t ChipID, int8_t *Temperature); //LFT - Low Fault Threshold
HAL_StatusTypeDef MAX31856_GetColdJunctionLFT(uint8_t ChipID, int8_t *ReturnValue);

HAL_StatusTypeDef MAX31856_GetColdJunctionTempOffset(uint8_t ChipID, float *ReturnValue);
HAL_StatusTypeDef MAX31856_SetColdJunctionTempOffset(uint8_t ChipID, float *Temperature);
HAL_StatusTypeDef MAX31856_GetColdJunctionTemperature(uint8_t ChipID, float *ReturnValue);
HAL_StatusTypeDef MAX31856_SetColdJunctionTemperature(uint8_t ChipID, float *Temperature);

HAL_StatusTypeDef MAX31856_GetLinearizedTemperatureHFT(uint8_t ChipID, float *ReturnValue);
HAL_StatusTypeDef MAX31856_SetLinearizedTemperatureHFT(uint8_t ChipID, float *Temperature);
HAL_StatusTypeDef MAX31856_SetLinearizedTemperatureLFT(uint8_t ChipID, float *Temperature);
HAL_StatusTypeDef MAX31856_GetLinearizedTemperatureLFT(uint8_t ChipID, float *ReturnValue);

#define MAX31856_TIMEOUT     10						//REMINDER call correct __HAL_RCC_SPI1_CLK_ENABLE(); SPI# instance and Prescaler
#define MAX31856_SCK_PIN	 GPIO_PIN_5				//PA5     ------> SPI1_SCK - was PB3
#define MAX31856_SCK_PORT	 GPIOA
#define MAX31856_MOSI_PIN	 GPIO_PIN_5				//PB5     ------> SPI1_MOSI
#define MAX31856_MOSI_PORT	 GPIOB
#define MAX31856_MISO_PIN	 GPIO_PIN_4				//PB4     ------> SPI1_MISO
#define MAX31856_MISO_PORT	 GPIOB

/*
 * MAX31856 Register Memory Map - Table 6, Page 18 MAX31856.pdf
 */										//-----------------------------------------------------------------------------------------
typedef enum {			   				//| ADDRESS | READ/WRITE | NAME   |DEFV | FUNCTION                                        |
	MAX31856_REG_CR0    = 0x00,			//| 00h/80h | Read/Write | CR0    | 00h | Configuration 0 Register                        |
	MAX31856_REG_CR1    = 0x01,			//| 01h/81h | Read/Write | CR1    | 03h | Configuration 1 Register                        |
	MAX31856_REG_MASK   = 0x02,			//| 02h/82h | Read/Write | MASK   | FFh | Fault Mask Register                             |
	MAX31856_REG_CJHF   = 0x03,			//| 03h/83h | Read/Write | CJHF   | 7Fh | Cold-Junction High Fault Threshold              |
	MAX31856_REG_CJLF   = 0x04,			//| 04h/84h | Read/Write | CJLF   | C0h | Cold-Junction Low Fault Threshold               |
	MAX31856_REG_LTHFTH = 0x05,			//| 05h/85h | Read/Write | LTHFTH | 7Fh | Linearized Temperature High Fault Threshold MSB |
	MAX31856_REG_LTHFTL = 0x06,     	//| 06h/86h | Read/Write | LTHFTL | FFh | Linearized Temperature High Fault Threshold LSB |
	MAX31856_REG_LTLFTH = 0x07,     	//| 07h/87h | Read/Write | LTLFTH | 80h | Linearized Temperature Low Fault Threshold MSB  |
	MAX31856_REG_LTLFTL = 0x08,			//| 08h/88h | Read/Write | LTLFTL | 00h | Linearized Temperature Low Fault Threshold LSB  |
	MAX31856_REG_CJTO   = 0x09,			//| 09h/89h | Read/Write | CJTO   | 00h | Cold-Junction Temperature Offset Register       |
	MAX31856_REG_CJTH   = 0x0A,			//| 0Ah/8Ah | Read/Write | CJTH   | 00h | Cold-Junction Temperature Register, MSB         |
	MAX31856_REG_CJTL   = 0x0B,			//| 0Bh/8Bh | Read/Write | CJTL   | 00h | Cold-Junction Temperature Register, LSB  		  |
	MAX31856_REG_LTCBH  = 0x0C,			//| 0Ch     | Read Only  | LTCBH  | 00h | Linearized TC Temperature, Byte 2               |
	MAX31856_REG_LTCBM  = 0x0D,			//| 0Dh 	| Read Only  | LTCBM  | 00h | Linearized TC Temperature, Byte 1               |
	MAX31856_REG_LTCBL  = 0x0E,			//| 0Eh     | Read Only  | LTCBL  | 00h | Linearized TC Temperature, Byte 0               |
	MAX31856_REG_SR     = 0x0F,			//| 0Fh     | Read Only  | SR     | 00h | Fault Status Register                           |
}max31856_registers_t;					//-----------------------------------------------------------------------------------------

#define MAX31856_REG_WRITE(x) (0x80 | x)       //to be ORed with the Write register


/* Register 00h/80h: Configuration 0 Register (CR0) - Default Value 0x00
 * The Configuration 0 register selects the conversion mode (automatic or triggered by the 1-shot command), selects opencircuit
 * fault detection timing, enables the cold-junction sensor, clears the fault status register, and selects the filter notch
 * frequencies. The effects of the configuration bits are described below.
 */
typedef enum{
	MAX31856_CR0_50Hz   = 0x01,	//50Hz/60Hz Noise Rejection Filter Selection
									//0= Selects rejection of 60Hz and its harmonics (default)
									//1= Selects rejection of 50Hz and its harmonics Note: Change the notch frequency only while in the “Normally Off” mode – not in the Automatic Conversion mode.
	MAX31856_CR0_FAULTCLR  = 0x02,	//Fault Status Clear
								//0 = Default
								//1 = When in interrupt mode, returns all Fault Status bits [7:0] in the Fault Status Register (0Fh) to 0 and deasserts the FAULT output. This bit has no effect in comparator mode. Note that the FAULT output and the fault bit may reassert immediately if the fault persists. To prevent the FAULT output from reasserting, first set the Fault Mask bits. The fault status clear bit self-clears to 0.
	MAX31856_CR0_FAULT_INTERRUPT = 0x04,	//Fault Mode
								//0 = Comparator Mode. The FAULT output and respective fault bit reflects the state of any nonmasked faults by asserting when the fault condition is true, and deasserting when the fault condition is no longer true. There is a 2°C hysteresis when in comparator mode for threshold fault conditions. (default)
								//1 = Interrupt Mode. The FAULT output and respective fault bit asserts when a non-masked fault condition is true and remain asserted until a 1 is written to the Fault Status Clear bit. This deasserts FAULT and respective fault bit until a new fault is detected (note that this may occur immediately if the fault condition is still in place).
	MAX31856_CR0_CJ_DISABLE	   = 0x08,	//Cold-Junction Sensor Disable
								//0 = Cold-junction temperature sensor enabled (default)
								//1 = Cold-junction temperature sensor disabled. Data from an external temperature sensor may be written to the cold-junction temperature register. When this bit changes from 0 to 1, the most recent cold-junction temperature value will remain in the cold-junction temperature register until the internal sensor is enabled or until a new value is written to the register. The overall temperature conversion time is reduced by 25ms (typ) when this bit is set to 1.

	MAX31856_CR0_OCFAULT01  = 0x10,  //TC Fault Test -- See Table 4 in datasheet for more info
	MAX31856_CR0_OCFAULT10  = 0x20,  //00 - Disabled (Default
	MAX31856_CR0_OCFAULT11  = 0x30,	 //01 - Enabled (Once every 16 conversions)


	MAX31856_CR0_1SHOT     = 0x40,	//One-Shot Mode
									//0 = No conversions requested (default)
									//1 = This causes a single cold-junction and thermocouple conversion to take place when Conversion Mode bit =0 (normally off mode). The conversion is triggered when CS goes high after writing a 1 to this bit. Note that if a multi-byte write is performed, the conversion is triggered when CS goes high at the end of the transaction. A single conversion requires approximately 143ms in 60Hz filter mode or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
	MAX31856_CR0_CMODE_AUTO= 0x80,	//Conversion Mode
									//0 = Normally Off mode (default)
									//1 = Automatic Conversion mode. Conversions occur continuously every 100ms (nominal).
}max31856_cr0_bits_t;


/* Thermocouple Voltage Conversion Averaging Mode
 * The Thermocouple Voltage Conversion Averaging Mode settings should not be changed while
 * conversions are taking place.
 * Adding samples increases the conversion time and reduces noise.
 * Typical conversion times:
 * 1-shot or first conversion in Auto mode:
 * = tCONV + (samples -1) x 33.33mS (60Hz rejection)
 * = tCONV + (samples -1) x 40mS (50Hz rejection)
 * 2 thru n conversions in Auto mode
 * = tCONV + (samples -1) x 16.67mS (60Hz rejection)
 * = tCONV + (samples -1) x 20mS (50Hz rejection)
 */
typedef enum
{
	MAX31856_CONVAVG_1 = 0b000,  //000 = 1 sample (default)
	MAX31856_CONVAVG_2 = 0b001,  //001 = 2 samples averaged
	MAX31856_CONVAVG_4 = 0b010,  //010 = 4 samples averaged
	MAX31856_CONVAVG_8 = 0b011,	 //011 = 8 samples averaged
	MAX31856_CONVAVG_16= 0b100,  //1xx = 16 samples averaged
}max31856_convavg_t;


/*
 * Thermocouple Type
 */
typedef enum
{
  MAX31856_TCTYPE_B  = 0b0000, //0000 = B Type
  MAX31856_TCTYPE_E  = 0b0001, //0001 = E Type
  MAX31856_TCTYPE_J  = 0b0010, //0010 = J Type
  MAX31856_TCTYPE_K  = 0b0011, //0011 = K Type (default)
  MAX31856_TCTYPE_N  = 0b0100, //0100 = N Type
  MAX31856_TCTYPE_R  = 0b0101, //0101 = R Type
  MAX31856_TCTYPE_S  = 0b0110, //0110 = S Type
  MAX31856_TCTYPE_T  = 0b0111, //0111 = T Type
  MAX31856_VMODE_G8  = 0b1000, //10xx = Voltage Mode, Gain = 8. Code = 8 x 1.6 x 217 x VIN    Where Code is 19 bit signed number from TC registers and VIN is thermocouple input voltage
  MAX31856_VMODE_G32 = 0b1100, //11xx = Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN  Where Code is 19 bit signed number from TC registers and VIN is thermocouple input voltage
} max31856_thermocoupletype_t;


/*
 * Fault Register
 */
typedef enum
{
	MAX31856_FLT_TC_OPEN = 0x01,//Thermocouple Open-Circuit Fault
	                                // 0 = No open circuit or broken thermocouple wires are detected (default)
	                                // 1 = An open circuit such as broken thermocouple wires has been detected. The FAULT output is asserted unless masked.
	MAX31856_FLT_OVUV	 = 0x02, //Overvoltage or Undervoltage Input Fault
									// 0 = The input voltage is positive and less than VDD (default).
									// 1 = The input voltage is negative or greater than VDD. The FAULT output is asserted unless masked.
									// Note: The presence of the OVUV fault will suspend conversions and the ability of the MAX31856 to
									// detect other faults (or clear faults when in comparator mode) until the fault is no longer present.
	MAX31856_FLT_TC_LOW  = 0x04,//Thermocouple Temperature Low Fault
									//0 = Thermocouple temperature is at or higher than the thermocouple temperature low threshold (default).
									//1 = Thermocouple temperature is lower than the thermocouple temperature low threshold. The FAULT output is asserted unless masked.
	MAX31856_FLT_TC_HIGH = 0x08,//Thermocouple Temperature High Fault
									//0 = The Thermocouple Temperature is at or lower than the thermocouple temperature high threshold (default).
									//1 = The Thermocouple Temperature is higher than the thermocouple temperature high threshold. The FAULT output is asserted unless masked.
	MAX31856_FLT_CJ_LOW  = 0x10,//Cold-Junction Low Fault
									//0 = The Cold-Junction temperature is at or higher than the cold-junction temperature low threshold (default).
									//1 = The Cold-Junction temperature is lower than the cold-junction temperature low threshold. The FAULT output is asserted unless masked.
	MAX31856_FLT_CJ_HIGH = 0x20,//Cold-Junction High Fault
									//0 = The Cold-Junction temperature is at or lower than the cold-junction temperature high threshold (default).
									//1 = The Cold-Junction temperature is higher than the cold-junction temperature high threshold. The FAULT output is asserted unless masked.
	MAX31856_FLT_TC_RANGE =0x40,//Thermocouple Out-of-Range
									//0 = The Thermocouple Hot Junction temperature is within the normal operating range (see Table 1).
									//1 = The Thermocouple Hot Junction temperature is outside of the normal operating range. Note: The TC Range bit should be ignored in voltage mode.
	MAX31856_FLT_CJ_RANGE =0x80,//Cold Junction Out-of-Range
									//0 = The Cold-Junction temperature is within the normal operating range (-55°C to +125°C for types E, J, K, N, and T; -50°C to +125°C for types R and S; 0 to 125°C for type B).
									//1 = The Cold-Junction temperature is outside of the normal operating range.
}max31856_faultreg_t;


/*
 * Fault MASK
 */
typedef enum
{
	MAX31856_FLTMASK_TC_OPEN = 0x01,//Thermocouple Open-Circuit Fault Mask
										//0 = FAULT output asserted when a thermocouple open condition is detected
										//1 = FAULT output masked (default)
	MAX31856_FLTMASK_UVOV	 = 0x02, //Over-voltage or Undervoltage Input Fault Mask
										//0 = FAULT output asserted when an over- or undervoltage condition is detected
										//1 = FAULT output masked (default)
	MAX31856_FLTMASK_TC_LOW  = 0x04, //Thermocouple Temperature Low Fault Threshold Mask
										//0 = FAULT output asserted when the Thermocouple Temperature falls below the Thermocouple Temperature low threshold limit value
										//1 = FAULT output masked (default)
	MAX31856_FLTMASK_TC_HIGH = 0x08, //Thermocouple Temperature High Fault Threshold Mask
										//0 = FAULT output asserted when the Thermocouple Temperature rises above the Thermocouple Temperature high threshold limit value
										//1 = FAULT output masked (default)
	MAX31856_FLTMASK_CJ_LOW  = 0x10, //Cold-Junction Low Fault Threshold Mask
										//0 = FAULT output asserted when the Cold-Junction Temperature falls below the Cold-Junction Temperature low threshold limit value
										//1 = FAULT output masked (default)
	MAX31856_FLTMASK_CJ_HIGH = 0x20, //Cold-Junction High Fault Threshold Mask
										//0 = FAULT output asserted when the Cold-Junction Temperature rises above the Cold-Junction Temperature high threshold limit value
										//1 = FAULT output masked (default)
	//7:6 Reserved Reserved.
}max31856_faultmask_t;


typedef struct
{
	uint8_t ConfigReg0;
	uint8_t FaultMask;
	max31856_thermocoupletype_t TCType;
	max31856_convavg_t			ConvAveraging;
	int8_t	CJHighFaultThreshold;
	int8_t  CJLowFaultThreshold;
	float   TCHighFaultThreshold;
	float   TCLowFaultThreshold;
}max31856_config_t;

typedef struct
{
	float Temperature0;
	float Temperature1;
	float Temperature2;
	float Temperature3;
	float Duty0;
	float Duty1;
	float Duty2;
	float Duty3;
	uint32_t TimeStamp;
}max31856_temperatures_t;



#endif /* APPLICATION_USER_MAX31856_MAX31856_H_ */
