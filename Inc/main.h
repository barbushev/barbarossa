#ifndef __MAIN_H
#define __MAIN_H

void JumpToBootLoader(void);

/**
 * Write 8-bit value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 1 is valid, if more, HardFault error can happen.
 * - uint8_t value:
 *         8-bit value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_Write8(address, value)    (*(__IO uint8_t *) (BKPSRAM_BASE + (address)) = (value))

/*
 * Read 8-bit value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 1 is valid, if more, HardFault error can happen.
 *
 * 8-bit value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_Read8(address)            (*(__IO uint8_t *) (BKPSRAM_BASE + address))


#endif /* __MAIN_H */
