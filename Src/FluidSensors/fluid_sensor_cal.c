/*
 * fluid_sensor_cal.c
 *
 *  Created on: Dec 21, 2016
 *      Author: Ted
 */

#include "fluid_sensor_cal.h"
#include "barbarossa_config.h"


I2C_HandleTypeDef i2cHandle;

//Private functions
static uint16_t DigiRead(uint8_t , uint8_t );
static HAL_StatusTypeDef DigiWrite(uint8_t, uint8_t, uint8_t *);
static HAL_StatusTypeDef SelectChannel(uint8_t bI2CChannel, uint8_t isSelected);
static uint8_t GetRegisterAddress(enum POT_MEM *potmem, enum POT_SIDE *potside, uint8_t *eeprom_reg_num);
static void GetPotAddress(uint8_t potnum, uint8_t *potADDR, uint8_t *muxCHAN);

HAL_StatusTypeDef FLUID_CAL_INIT()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate =  GPIO_AF4_I2C2;						//CORRECT I2C number goes here

	GPIO_InitStruct.Pin = BA_DIGIPOT.I2C_SCL.IO_PIN;
    HAL_GPIO_Init(BA_DIGIPOT.I2C_SCL.IO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BA_DIGIPOT.I2C_SDA.IO_PIN;
    HAL_GPIO_Init(BA_DIGIPOT.I2C_SDA.IO_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __I2C2_CLK_ENABLE();											//CORRECT I2C number goes here

	i2cHandle.Instance = I2C2;										//CORRECT I2C number goes here
	i2cHandle.Init.ClockSpeed = 400000;
	i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2cHandle.Init.OwnAddress1 = 0;
	i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2cHandle.Init.OwnAddress2 = 0;
	i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	return (HAL_I2C_Init(&i2cHandle));
}

uint8_t FLUID_CAL_IS_READY()
{
	if (HAL_I2C_IsDeviceReady(&i2cHandle,0,1, 100) == HAL_OK) return 1;
	else return 0;
}

/// Gets the reading for a certain digipot. Returns (256) if unsuccessful.
/// <"potnum">Number of pot to be read. Valid numbers are 1 to 11 included.
/// <"potmem">VOLATILE, NON_VOLATILE or EEPROM
/// <"potside">SENSOR or COMPARATOR
/// <"eeprom_reg_num">EEPROM Register number. Valid numbers are 0 to 9 included. Only used when potmem = EEPROM.
/// <returns>Returns pot setting as uint16_t. If unsuccessful, returns (256)
uint16_t FLUID_CAL_READPOT(uint8_t potnum, enum POT_MEM potmem, enum POT_SIDE potside, uint8_t eeprom_reg_num)
{
	 uint8_t regADDR = GetRegisterAddress(&potmem, &potside, &eeprom_reg_num);
	 uint8_t muxCHAN, potADDR;
	 GetPotAddress(potnum, &potADDR, &muxCHAN);

	 //SelectChannel if going through mux
	 SelectChannel(muxCHAN, 1);
	 uint16_t data = DigiRead(potADDR, regADDR);
	 SelectChannel(muxCHAN, 0);
	 return data;
}


/// Writes to a certain digipot. Returns HAL_OK if successful.
/// <"potnum">Number of pot to write to. Valid numbers are 1 to 11 included.
/// <"potmem">VOLATILE, NON_VOLATILE or EEPROM
/// <"potside">SENSOR or COMPARATOR
/// <"eeprom_reg_num">EEPROM Register number. Valid numbers are 0 to 9 included.
/// <"potvalue">Pot value - valid numbers are 0 to 255 included
/// <returns>HAL_OK if successful
HAL_StatusTypeDef FLUID_CAL_WRITEPOT(uint8_t potnum, enum POT_MEM potmem, enum POT_SIDE potside, uint8_t eeprom_reg_num, uint8_t potvalue)
    {
		uint8_t regADDR = GetRegisterAddress(&potmem, &potside, &eeprom_reg_num);
		uint8_t muxCHAN, potADDR;
		GetPotAddress(potnum, &potADDR, &muxCHAN);

	    //SelectChannel if going through mux
	    SelectChannel(muxCHAN, 1);
	    HAL_StatusTypeDef data = (DigiWrite(potADDR, regADDR, &potvalue));
	    SelectChannel(muxCHAN, 0);
	    return data;
    }

/*
 * PCA9545A Channel Select
 * Set isSelected = 0 to deselect a specific channel and isSelected = 1 to select it
 */
static HAL_StatusTypeDef SelectChannel(uint8_t bI2CChannel, uint8_t isSelected)
    {
	uint8_t buffer = (isSelected << bI2CChannel);

    return HAL_I2C_Master_Transmit(&i2cHandle, ADDR_I2CSWITCH, &buffer, 1, I2C_TIMEOUT);
    }

/*
 * MCP4661 I2C random write
 */
HAL_StatusTypeDef DigiWrite(uint8_t slaveAddress, uint8_t registerAddress, uint8_t *pData)
{
    uint8_t buffer[2];
    slaveAddress &= ~1;  //set LSB to 0 (write bit)

    registerAddress <<= 4;
    buffer[0] = (registerAddress | WRITE_COMMAND_LO_NIB);
    buffer[1] = *pData;

	return HAL_I2C_Master_Transmit(&i2cHandle, slaveAddress, buffer, 2, I2C_TIMEOUT );
}

/*
 * MCP4661 I2C random read
 */
uint16_t DigiRead(uint8_t slaveAddress, uint8_t registerAddress)
{
    uint8_t buffer[2];
    HAL_StatusTypeDef status;

    slaveAddress &= ~1;  //set LSB to 0 (write bit)

    registerAddress <<= 4;
    buffer[0] = (registerAddress | READ_COMMAND_LO_NIB);  //device memory address + command

    status = HAL_I2C_Master_Transmit(&i2cHandle, slaveAddress, buffer, 1, I2C_TIMEOUT);

    slaveAddress |= 1;  //set LSB to 1 (read bit)
    status |= HAL_I2C_Master_Receive(&i2cHandle, slaveAddress, buffer, 2, I2C_TIMEOUT);

    if (status != HAL_OK) return 256;

    return buffer[1];
}


//helper function to get the pot address
void GetPotAddress(uint8_t potnum, uint8_t *potADDR, uint8_t *muxCHAN)
{
	 switch (potnum)
		         {
		             case 0:{*potADDR = DPOT_POS02; *muxCHAN = ADDR_FEEBLE_A;  break;}
		             case 1:{*potADDR = DPOT_POS03; *muxCHAN = ADDR_FLAIL;     break;}
		             case 2:{*potADDR = DPOT_POS04; *muxCHAN = ADDR_FEEBLE_A;  break;}
		             case 3:{*potADDR = DPOT_POS05; *muxCHAN = ADDR_FEEBLE_A;  break;}
		             case 4:{*potADDR = DPOT_POS06; *muxCHAN = ADDR_FEEBLE_A;  break;}
		             case 5:{*potADDR = DPOT_POS07; *muxCHAN = ADDR_FEEBLE_B;  break;}
		             case 6:{*potADDR = DPOT_POS08; *muxCHAN = ADDR_FLAIL;     break;}
		             case 7:{*potADDR = DPOT_POS09; *muxCHAN = ADDR_FEEBLE_B;  break;}
		             case 8:{*potADDR = DPOT_POS10; *muxCHAN = ADDR_FEEBLE_B;  break;}
		             case 9:{*potADDR = DPOT_POS11; *muxCHAN = ADDR_FEEBLE_B; break;}
		             case 10:{*potADDR = DPOT_POS12; *muxCHAN = ADDR_FEEBLE_B; break;}
		         }
}


//helper function to get the register address
uint8_t GetRegisterAddress(enum POT_MEM *potmem, enum POT_SIDE *potside, uint8_t *eeprom_reg_num)
{
	 uint8_t reg = 0;

	 switch (*potmem)
	 {
	 case VOLATILE:
	 	 {
		 	 if (*potside == SENSOR)     reg = DPOT_REG_VWIP1;
		 	 if (*potside == COMPARATOR) reg = DPOT_REG_VWIP0;
	 		 break;
	 	 }
	 case NON_VOLATILE:
	 	 {
	 		 if (*potside == SENSOR) 	reg = DPOT_REG_NWIP1;
	 		 if (*potside == COMPARATOR) reg = DPOT_REG_NWIP0;
	 		 break;
	 	 }
	 case EEPROM:
	 	 {
	 		 reg = EEPROM_REG_NUM(*eeprom_reg_num);
	 		 break;
	 	 }
	 }

	 return reg;
}

/*
 * n & ~1 replaces the least significant bit of n with zero; n | 1, with one.

To replace the LSB with b, where b can be either 0 or 1, you can use (n & ~1) | b.

To replace the k-th bit with b (where k=0 stands for the LSB): (n & ~(1 << k)) | (b << k).
 */
