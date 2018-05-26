/*
 * fluid_sensor_cal.h
 *
 *  Created on: Dec 21, 2016
 *      Author: Ted
 */

#ifndef FLUID_SENSOR_CAL_H_
#define FLUID_SENSOR_CAL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

//timeout for I2C commands (ms)
#define I2C_TIMEOUT 10

//addresses
//#define ADDR_PCA9545A   0x00
//#define ADDR_FEEBLE_A   0x01
//#define ADDR_FEEBLE_B   0x02
//#define ADDR_FLAIL 	    0x04

#define ADDR_FEEBLE_A   0x00
#define ADDR_FEEBLE_B   0x01
#define ADDR_FLAIL 	    0x02
#define ADDR_I2CSWITCH  0xE0              //originally 0x70, but after left shift 1bit 0xE0

//mcp4661 digipot specific
#define WRITE_COMMAND_LO_NIB  0x02
#define READ_COMMAND_LO_NIB   0x0F
#define INCR_COMMAND_LO_NIB   0x04
#define DECR_COMMAND_LO_NIB   0x08

//...MCP4661 i2c addressing...All are left shifted 1 bit. Original value is commented out
#define DPOT_CARD   0x50 //0X28
#define DPOT_POS02  0x52 //0x29 //...i2c channel 0...MCP4661 b'0101 w/ HW address b'001...
#define DPOT_POS03  0x54 //0x2A //HW address 001...
#define DPOT_POS04  0x54 //0x2A //HW address 010...
#define DPOT_POS05  0x56 //0x2B //HW address 011...
#define DPOT_POS06  0x58 //0x2C //HW address 100...
#define DPOT_POS07  0x50 //0x28 //HW address 101...
#define DPOT_POS08  0x52 //0x29 //HW address 110...
#define DPOT_POS09  0x52 //0x29 //HW address 111...
#define DPOT_POS10  0x54 //0x2A //...i2c channel 1...MCP4661 b'0101 w/ HW address b'000...
#define DPOT_POS11  0x56 //0x2B //HW address 001...
#define DPOT_POS12  0x58 //0x2C //HW address 010...

//...MCP4661 i2c control registers...
#define DPOT_REG_VWIP0   0x0
#define DPOT_REG_VWIP1   0x1
#define DPOT_REG_NWIP0   0x2
#define DPOT_REG_NWIP1   0x3
#define DPOT_REG_VTCON   0x4
#define DPOT_REG_STATUS  0x5

//...MCP4661 i2c eeprom registers...
//#define DPOT_REG_EEPR0  0x6
//#define DPOT_REG_EEPR1  0x7
//#define DPOT_REG_EEPR2  0x8
//#define DPOT_REG_EEPR3  0x9
//#define DPOT_REG_EEPR4  0xA
//#define DPOT_REG_EEPR5  0xB
//#define DPOT_REG_EEPR6  0xC
//#define DPOT_REG_EEPR7  0xD
//#define DPOT_REG_EEPR8  0xE
//#define DPOT_REG_EEPR9  0xF
#define EEPROM_REG_NUM(x) ((x) += 0x06)

enum POT_MEM
    {
        VOLATILE,
        NON_VOLATILE,
		EEPROM
    };

enum POT_SIDE
    {
        SENSOR,
        COMPARATOR
    };

HAL_StatusTypeDef FLUID_CAL_INIT();
uint8_t FLUID_CAL_IS_READY();
uint16_t FLUID_CAL_READPOT(uint8_t, enum POT_MEM, enum POT_SIDE, uint8_t);
HAL_StatusTypeDef FLUID_CAL_WRITEPOT(uint8_t, enum POT_MEM, enum POT_SIDE, uint8_t , uint8_t );

#endif /* FLUID_SENSOR_CAL_H_ */
