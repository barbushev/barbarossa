/*
 * MAX31856.c
 *
 *  Created on: Jan 20, 2017
 *      Author: Ted Barbushev
 */

#include <barbarossa_config.h>
#include <MAX31856.h>

static HAL_StatusTypeDef MAX31856_ReadRegister(uint8_t ChipID, max31856_registers_t RegisterToRead, uint8_t numBytes, uint8_t *ReturnValue);
static HAL_StatusTypeDef MAX31856_WriteRegister(uint8_t ChipID, max31856_registers_t RegisterToWrite, uint8_t numBytes, uint8_t *ValueToWrite);
static HAL_StatusTypeDef MAX31856_ConfigureSensor(uint8_t ChipID, max31856_config_t *configData);  //call only after initSPI
static SPI_HandleTypeDef handleSPI;

void MAX31856_InitSPI()
{
	  GPIO_InitTypeDef GPIO_InitStruct;


	  /*Configure GPIO pin : PD2 */          //External Interrupt
	  /*
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	  */

	  //configure and set chip select pins
	  for (uint8_t ChipID = 0; ChipID < HEATER_COUNT; ChipID++)
	  {
		  GPIO_InitStruct.Pin = MAX31856_ChipSelect_IO[ChipID].IO_PIN;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(MAX31856_ChipSelect_IO[ChipID].IO_PORT, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(MAX31856_ChipSelect_IO[ChipID].IO_PORT, MAX31856_ChipSelect_IO[ChipID].IO_PIN, GPIO_PIN_SET); //chip select high
	  }

	  __HAL_RCC_SPI1_CLK_ENABLE();    //Correct SPI# GOES HERE

      GPIO_InitStruct.Pin = MAX31856_SCK_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
      GPIO_InitStruct.Alternate =  GPIO_AF5_SPI1;    //Correct SPI# GOES HERE
      HAL_GPIO_Init(MAX31856_SCK_PORT, &GPIO_InitStruct);

      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Pin = MAX31856_MOSI_PIN;
      HAL_GPIO_Init(MAX31856_MOSI_PORT, &GPIO_InitStruct);

      GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Pin = MAX31856_MISO_PIN;
	  HAL_GPIO_Init(MAX31856_MISO_PORT, &GPIO_InitStruct);


	  handleSPI.Instance = SPI1; //Correct SPI# GOES HERE
	  handleSPI.Init.Mode = SPI_MODE_MASTER;
	  handleSPI.Init.Direction = SPI_DIRECTION_2LINES;
	  handleSPI.Init.DataSize = SPI_DATASIZE_8BIT;
	  handleSPI.Init.CLKPolarity = SPI_POLARITY_HIGH;
	  handleSPI.Init.CLKPhase = SPI_PHASE_2EDGE;
	  handleSPI.Init.NSS = SPI_NSS_SOFT;
	  handleSPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  handleSPI.Init.TIMode = SPI_TIMODE_DISABLED;
	  handleSPI.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	  handleSPI.Init.NSS = SPI_NSS_SOFT;

	  //CPU Freq / prescaler = spi clock		168Mhz / 32 = 5.25 Mhz

      handleSPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  HAL_SPI_Init(&handleSPI);

	  max31856_config_t TempSensor;
	  TempSensor.TCType = MAX31856_TCTYPE_T;
	  TempSensor.ConvAveraging = MAX31856_CONVAVG_1; //MAX31856_CONVAVG_8;
	  TempSensor.FaultMask = MAX31856_FLTMASK_TC_OPEN | MAX31856_FLTMASK_UVOV | MAX31856_FLTMASK_TC_LOW | MAX31856_FLTMASK_TC_HIGH;
	  TempSensor.ConfigReg0 = MAX31856_CR0_CMODE_AUTO;
	  TempSensor.CJHighFaultThreshold = 75;
	  TempSensor.CJLowFaultThreshold  = 5;
	  TempSensor.TCHighFaultThreshold = HEATER_MAX_TEMP;
	  TempSensor.TCLowFaultThreshold  = HEATER_MIN_TEMP;

	  if (MAX31856_ConfigureSensor(0, &TempSensor) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_INIT, 0);
	  if (MAX31856_ConfigureSensor(1, &TempSensor) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_INIT, 1);
	  if (MAX31856_ConfigureSensor(2, &TempSensor) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_INIT, 2);
	  if (MAX31856_ConfigureSensor(3, &TempSensor) != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_MAX31856_INIT, 3);
}

/*
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		HEATER_CONTROL[0].TemperatureActual = MAX31856_Test1();
		DRDY = HAL_GetTick() - DRSTART;// - DRSTART;
		DRSTART = HAL_GetTick();

	}
}
*/

HAL_StatusTypeDef MAX31856_ConfigureSensor(uint8_t ChipID, max31856_config_t *configData)
{
	HAL_StatusTypeDef status;
	status = MAX31856_SetConfigReg0(ChipID, &configData->ConfigReg0);
	uint8_t cr1 = (configData->ConvAveraging << 4) | configData->TCType;

	status |= MAX31856_SetConfigReg1(ChipID, &cr1); //low 4 bits is TC, upper 3 bits is ConvAvg, bit 7 reserved

	status |= MAX31856_SetColdJunctionHFT(ChipID, &configData->CJHighFaultThreshold);
	status |= MAX31856_SetColdJunctionLFT(ChipID, &configData->CJLowFaultThreshold);

	status |= MAX31856_SetLinearizedTemperatureHFT(ChipID, &configData->TCHighFaultThreshold);
	status |= MAX31856_SetLinearizedTemperatureLFT(ChipID, &configData->TCLowFaultThreshold);

	status |= MAX31856_SetFaultMask(ChipID, &configData->FaultMask);

	return status;
}

/*  Read N bytes from register. Usually 1, 2 or 3. ChipID - 0,1,2,3
 */
HAL_StatusTypeDef MAX31856_ReadRegister(uint8_t ChipID, max31856_registers_t RegisterToRead, uint8_t numBytes, uint8_t *ReturnValue)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(MAX31856_ChipSelect_IO[ChipID].IO_PORT, MAX31856_ChipSelect_IO[ChipID].IO_PIN, GPIO_PIN_RESET);  //chip select low

	status = HAL_SPI_Transmit(&handleSPI, &RegisterToRead, 1, MAX31856_TIMEOUT);					        	//select the register
	status |= HAL_SPI_TransmitReceive(&handleSPI, &RegisterToRead, ReturnValue, numBytes, MAX31856_TIMEOUT);	//clock the data out

	HAL_GPIO_WritePin(MAX31856_ChipSelect_IO[ChipID].IO_PORT, MAX31856_ChipSelect_IO[ChipID].IO_PIN, GPIO_PIN_SET);		//chip select high
	return status;
}

/*  Write N bytes to register. Usually 1, 2 or 3. ChipID - 0,1,2,3
 */
HAL_StatusTypeDef MAX31856_WriteRegister(uint8_t ChipID, max31856_registers_t RegisterToWrite, uint8_t numBytes, uint8_t *ValueToWrite)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(MAX31856_ChipSelect_IO[ChipID].IO_PORT, MAX31856_ChipSelect_IO[ChipID].IO_PIN, GPIO_PIN_RESET);  //chip select low
	uint8_t rxdata[1] = {MAX31856_REG_WRITE(RegisterToWrite)};    //convert a read register to write register via MAX31856_REG_WRITE()

	status  = HAL_SPI_Transmit(&handleSPI, rxdata, 1, MAX31856_TIMEOUT);		//select the register
	status |= HAL_SPI_Transmit(&handleSPI, ValueToWrite, numBytes, MAX31856_TIMEOUT);

	HAL_GPIO_WritePin(MAX31856_ChipSelect_IO[ChipID].IO_PORT, MAX31856_ChipSelect_IO[ChipID].IO_PIN, GPIO_PIN_SET);		//chip select high
	return status;
}

/* Cold-Junction High Fault Threshold Register (CJHF)
 * Write a temperature limit value to this register. When the measured cold-junction temperature is greater than this value,
 * the CJ High fault status bit will be set and (if not masked) the FAULT output will assert.
 * The maximum cold-junction temperature is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_SetColdJunctionHFT(uint8_t ChipID, int8_t *Temperature)
{
	return MAX31856_WriteRegister(ChipID, MAX31856_REG_CJHF, 1, (uint8_t *)Temperature); //Two's-complement integers - int8_t to uint8_t
}

/* Cold-Junction High Fault Threshold Register (CJHF)
 * Read a temperature limit value from this register. When the measured cold-junction temperature is greater than this value,
 * the CJ High fault status bit will be set and (if not masked) the FAULT output will assert.
 * The maximum cold-junction temperature is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_GetColdJunctionHFT(uint8_t ChipID, int8_t *ReturnValue)
{
	return MAX31856_ReadRegister(ChipID, MAX31856_REG_CJHF, 1, (uint8_t *)ReturnValue); //Two's-complement integers - int8_t to uint8_t
}

/* Cold-Junction Low Fault Threshold Register (CJLF)
 * Write a temperature limit value to this register. When the measured cold-junction temperature is less than this value, the
 * CJ Low fault status bit will be set and (if not masked) the FAULT output will assert
 * The maximum cold-junction temperature is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_SetColdJunctionLFT(uint8_t ChipID, int8_t *Temperature)
{
	return MAX31856_WriteRegister(ChipID, MAX31856_REG_CJLF, 1, (uint8_t *)Temperature); //Two's-complement integers - int8_t to uint8_t
}

/* Cold-Junction Low Fault Threshold Register (CJLF)
 * Write a temperature limit value to this register. When the measured cold-junction temperature is less than this value, the
 * CJ Low fault status bit will be set and (if not masked) the FAULT output will assert
 * The maximum cold-junction temperature is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_GetColdJunctionLFT(uint8_t ChipID, int8_t *ReturnValue)
{
	return MAX31856_ReadRegister(ChipID, MAX31856_REG_CJLF, 1, (uint8_t *)ReturnValue); //Two's-complement integers - int8_t to uint8_t
}

/* Read Linearized Temperature High Fault Threshold Register, MSB (LTHFTH)
 * When the linearized thermocouple temperature is greater than the two-byte (05h and 06h) limit value,
 * the TC High fault status bit will be set and (if not masked) the FAULT output will assert.
 */
HAL_StatusTypeDef MAX31856_GetLinearizedTemperatureHFT(uint8_t ChipID, float *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[2];

	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_LTHFTH, 2, buffer); //start at High Byte and device will auto increment to next address - low byte

	uint16_t ret = (buffer[0] << 8);
	ret |= buffer[1];

	int16_t temp = ret;
	*ReturnValue = (temp * 0.0625f);  //0.0625 - for 15 bits

	return status;
}

/* Write Linearized Temperature High Fault Threshold Register, MSB (LTHFTH)
 * When the linearized thermocouple temperature is greater than the two-byte (05h and 06h) limit value,
 * the TC High fault status bit will be set and (if not masked) the FAULT output will assert.
 * Upto 4 digits after decimal point
 */
HAL_StatusTypeDef MAX31856_SetLinearizedTemperatureHFT(uint8_t ChipID, float *Temperature)
{
	HAL_StatusTypeDef status;

	int16_t temp = (*Temperature / 0.0625f);  //don't do this directly under data - have to pass through a signed int to take advantage of two's complement
	uint16_t data = temp;

	uint8_t buffer[2];
	buffer[0]  = (data >> 8);
	buffer[1]  = (data);

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_LTHFTH, 2,  buffer); //start at High Byte and device will auto increment to next address - low byte

	return status;
}

/* Write Linearized Temperature Low Fault Threshold Register
 * When the linearized thermocouple temperature is less than the two-byte (07h and 08h) limit value,
 * the TC Low fault status bit will be set and (if not masked) the FAULT output will assert.
 */
HAL_StatusTypeDef MAX31856_SetLinearizedTemperatureLFT(uint8_t ChipID, float *Temperature)
{
	HAL_StatusTypeDef status;

	int16_t temp = (*Temperature / 0.0625f);  //don't do this directly under data - have to pass through a signed int to take advantage of two's complement
	uint16_t data = temp;

	uint8_t buffer[2];
	buffer[0]  = (data >> 8);
	buffer[1]  = (data);

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_LTLFTH, 2,  buffer); //start at High Byte and device will auto increment to next address - low byte

	return status;
}

/* Read Linearized Temperature Low Fault Threshold Register
 * When the linearized thermocouple temperature is less than the two-byte (07h and 08h) limit value,
 * the TC Low fault status bit will be set and (if not masked) the FAULT output will assert.
 */
HAL_StatusTypeDef MAX31856_GetLinearizedTemperatureLFT(uint8_t ChipID, float *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[2];

	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_LTLFTH, 2, buffer); //start at High Byte and device will auto increment to next address - low byte

	uint16_t ret = (buffer[0] << 8);
	ret |= buffer[1];

	int16_t temp = ret;
	*ReturnValue = (temp * 0.0625f);  //0.0625 - for 15 bits

	return status;
}

/* Read Cold-Junction Temperature Offset Register (CJTO)
 * When the cold-junction temperature sensor is enabled, this register allows an offset temperature to be applied to the
 * measured value. See the Cold-Junction Temperature Sensing section of this data sheet for additional information.
 * The MSB of the offset register is 4°C and the LSB is 0.0625°C. The resulting range of the offset value applied to the
 * measured CJ temperature is -8°C to +7.9375°C. The default offset value is 0°C.
 */
HAL_StatusTypeDef MAX31856_GetColdJunctionTempOffset(uint8_t ChipID, float *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[1];

	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_CJTO, 1, buffer);

	int8_t temp = buffer[0];
	*ReturnValue = (temp * 0.0625f);  //0.0625 - for 15 bits

	return status;
}

/* Write Cold-Junction Temperature Offset Register (CJTO)
 * When the cold-junction temperature sensor is enabled, this register allows an offset temperature to be applied to the
 * measured value. See the Cold-Junction Temperature Sensing section of this data sheet for additional information.
 * The MSB of the offset register is 4°C and the LSB is 0.0625°C. The resulting range of the offset value applied to the
 * measured CJ temperature is -8°C to +7.9375°C. The default offset value is 0°C.
 */
HAL_StatusTypeDef MAX31856_SetColdJunctionTempOffset(uint8_t ChipID, float *Temperature)
{
	HAL_StatusTypeDef status;

	int8_t temp = (*Temperature / 0.0625f);  //don't do this directly under data - have to pass through a signed int to take advantage of two's complement
	uint8_t buffer[1] = {temp};

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_CJTO, 1,  buffer); //start at High Byte and device will auto increment to next address - low byte

	return status;
}


/* Read Cold-Junction Temperature Register (CJTH)
 * This register is used for cold-junction compensation of the thermocouple measurement. When the cold-junction temperature sensor is enabled,
 * this register is read-only and contains the measured cold-junction temperature plus the value in the Cold-Junction Offset register.
 * Also when the cold-junction temperature sensor is enabled, a read of this register will reset the DRDY pin high. When the cold-junction
 * temperature sensor is disabled, this register becomes a read-write register that contains the most recent cold-junction conversion result
 * until a new value is written into it. This allows writing the results from an external temperature sensor, if desired.
 * The maximum contained in the two cold-junction temperature bytes is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_GetColdJunctionTemperature(uint8_t ChipID, float *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[2];

	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_CJTH, 2, buffer);

	uint16_t ret = (buffer[0] << 8);
	ret         |= (buffer[1]);

	int16_t temp = ret;
	temp >>= 2; //the lower 2 bits are always 0 according to manual
	*ReturnValue = (temp * 0.015625f);  //0.015625 - per bit  +127.984375

	return status;
}

/* Write Cold-Junction Temperature Register (CJTH)
 * This register is used for cold-junction compensation of the thermocouple measurement. When the cold-junction temperature sensor is enabled,
 * this register is read-only and contains the measured cold-junction temperature plus the value in the Cold-Junction Offset register.
 * Also when the cold-junction temperature sensor is enabled, a read of this register will reset the DRDY pin high. When the cold-junction
 * temperature sensor is disabled, this register becomes a read-write register that contains the most recent cold-junction conversion result
 * until a new value is written into it. This allows writing the results from an external temperature sensor, if desired.
 * The maximum contained in the two cold-junction temperature bytes is clamped at 128°C and the minimum is clamped at -64°C.
 */
HAL_StatusTypeDef MAX31856_SetColdJunctionTemperature(uint8_t ChipID, float *Temperature)
{
	HAL_StatusTypeDef status;

	int16_t temp = (*Temperature / 0.015625f);  //don't do this directly under data - have to pass through a signed int to take advantage of two's complement
	temp <<= 2;  //lower 2 bits should always be 0
	uint16_t data = temp;


	uint8_t buffer[2];
	buffer[0]  = (data >> 8);
	buffer[1]  = (data);
	//buffer[1] >>= 2;

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_CJTH, 2,  buffer); //start at High Byte and device will auto increment to next address - low byte

	return status;
}

/* Linearized TC Temperature
 * This is a 19-bit register that contains the linearized and cold-junction-compensated thermocouple temperature value.
 */
HAL_StatusTypeDef MAX31856_GetTCTemp(uint8_t ChipID, float *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[3];
	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_LTCBH, 3,  buffer); //start at High Byte and device will auto increment to next address - low byte

	uint32_t ret = (buffer[0] << 24);		//shift all the way left because of sign
	ret         |= (buffer[1] << 16);
	ret         |= (buffer[2] << 8);

	int32_t temp = ret;  //two's complement
	temp >>= 13;  //the lower 5 bits are not used and the data was shifted 8 bits to far to the left to begin with so >> 13

	*ReturnValue = temp * 0.0078125;  //0.0078125 - for 18 bits

	return status;
}


/* Read Fault Status Register
 * The Fault Status Register contains eight bits that indicate the fault conditions (Thermocouple Out-of-Range, Cold
 * Junction Out-of-Range, Cold Junction High, Cold Junction Low, Thermocouple High Temperature, Thermocouple Low
 * Temperature, Over-Under Voltage, or Open Thermocouple) that have been detected.
 * Note: When the MAX31856 is set to operate in “comparator” fault mode (set with bit 2 of Configuration 0 register (00h)), the fault
 * status bits simply reflect the state of any faults by asserting when the fault condition is true, and deasserting when the fault condition
 * is no longer true.
 * When in “interrupt” fault mode, the fault status bits assert when a fault condition is true. The bits remain asserted until a 1 is written
 * to the Fault Status Clear bit. This deasserts the fault bits until a new fault is detected (note that this may occur immediately if the
 * fault condition is still in place).
 */
HAL_StatusTypeDef MAX31856_GetFaultStatus(uint8_t ChipID, uint8_t *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[1];
	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_SR, 1,  buffer);

	return status;
}


/* Read Fault Mask Register (MASK)
 * The Fault Mask Register allows the user to mask faults from causing the FAULT output from asserting. Masked faults
 * will still result in fault bits being set in the Fault Status register (0Fh). Note that the FAULT output is never asserted by
 * thermocouple and cold-junction out-of-range status.
 */
HAL_StatusTypeDef MAX31856_GetFaultMask(uint8_t ChipID, uint8_t *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[1];
	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_MASK, 1,  buffer);

	return status;
}

/* Write Fault Mask Register (MASK)
 * The Fault Mask Register allows the user to mask faults from causing the FAULT output from asserting. Masked faults
 * will still result in fault bits being set in the Fault Status register (0Fh). Note that the FAULT output is never asserted by
 * thermocouple and cold-junction out-of-range status.
 */
HAL_StatusTypeDef MAX31856_SetFaultMask(uint8_t ChipID, uint8_t *data)
{
	HAL_StatusTypeDef status;

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_MASK, 1, data);

	return status;
}


/* Read Configuration 0 Register (CR0)
 * The Configuration 0 register selects the conversion mode (automatic or triggered by the 1-shot command), selects opencircuit
 * fault detection timing, enables the cold-junction sensor, clears the fault status register, and selects the filter notch frequencies.
 */
HAL_StatusTypeDef MAX31856_GetConfigReg0(uint8_t ChipID, uint8_t *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[1];
	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_CR0, 1,  buffer);

	return status;
}

/* Write Configuration 0 Register (CR0)
 * The Configuration 0 register selects the conversion mode (automatic or triggered by the 1-shot command), selects opencircuit
 * fault detection timing, enables the cold-junction sensor, clears the fault status register, and selects the filter notch frequencies.
 */
HAL_StatusTypeDef MAX31856_SetConfigReg0(uint8_t ChipID, uint8_t *data)
{
	HAL_StatusTypeDef status;

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_CR0, 1, data);

	return status;
}

/* Read Configuration 1 Register (CR1)
 * The Configuration 1 register selects the averaging time for the thermocouple voltage conversion averaging mode and
 * also selects the thermocouple type being monitored.
 */
HAL_StatusTypeDef MAX31856_GetConfigReg1(uint8_t ChipID, uint8_t *ReturnValue)
{
	HAL_StatusTypeDef status;

	uint8_t buffer[1];
	status  = MAX31856_ReadRegister(ChipID, MAX31856_REG_CR1, 1,  buffer);

	return status;
}

/* Write Configuration 1 Register (CR1)
 * The Configuration 1 register selects the averaging time for the thermocouple voltage conversion averaging mode and
 * also selects the thermocouple type being monitored.
*/
HAL_StatusTypeDef MAX31856_SetConfigReg1(uint8_t ChipID, uint8_t *data)
{
	HAL_StatusTypeDef status;

	status  = MAX31856_WriteRegister(ChipID, MAX31856_REG_CR1, 1, data);

	return status;
}
