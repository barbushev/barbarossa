/*
 * actuator_io.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Ted
 */

#ifndef ACTUATOR_IO_H_
#define ACTUATOR_IO_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "barbarossa_config.h"

void ACTUATOR_INIT();
void ACTUATOR_EXTEND(int32_t *, uint8_t);
void ACTUATOR_RETRACT(int32_t *, uint8_t);
uint8_t Actuator_AreExtended(int32_t *, uint8_t);
uint8_t Actuator_AreRetracted(int32_t *, uint8_t);

#endif /* ACTUATOR_IO_H_ */
