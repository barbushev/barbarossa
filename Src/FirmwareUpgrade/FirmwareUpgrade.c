/*
 * FirmwareUpgrade.c
 *
 *  Created on: Apr 20, 2017
 *      Author: Ted
 */

#include "FirmwareUpgrade.h"
#include "USB_Task.h"

// Private variables ---------------------------------------------------------
static uint32_t TmpProgramCounter = 0x00;
static uint32_t TmpReadSize = 0x00;
static uint32_t RamAddress = 0x00;
static uint32_t FirstSector = 0;
static uint32_t NbOfSectors = 0;
static uint32_t SectorError = 0;
static uint32_t OB_RDP_LEVEL;
static __IO uint32_t LastPGAddress = APPLICATION_ADDRESS;
static uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};
static FIL FlashFile;  /* File object for download operation */

/* Private function prototypes -----------------------------------------------*/
static uint8_t FW_ProgramFlash();
static FlagStatus FLASH_If_ReadOutProtectionStatus(void);
static uint8_t FLASH_If_EraseSectors(uint32_t Address);
static uint8_t FLASH_If_Write(uint32_t Address, uint32_t Data);
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address);
static FLASH_OBProgramInitTypeDef FLASH_OBProgramInitStruct;
static FLASH_EraseInitTypeDef FLASH_EraseInitStruct;


/**
  * @brief  IAP Write Flash memory.
  * @param  FileToFlash is the absolute path + file name of the binary file to be flashed
  * @retval Returns 0 = Fail, or 1 = Success
  */
uint8_t FW_FlashFileToMemory(const char *FileToFlash)
{
  uint8_t result = 1;
  /* Open the binary file to be downloaded */
  if(f_open(&FlashFile, FileToFlash, FA_READ) == FR_OK)
  {
    if(FlashFile.fsize > USER_FLASH_SIZE)
    {
      // No available Flash memory size for the binary file
      BA_ERROR_HANLDER(BA_ERROR_FLASH_SIZE, 0);
      return 0;
    }
    else
    {
      // Download is On Going. Erase FLASH sectors to download image

      if(FLASH_If_EraseSectors(APPLICATION_ADDRESS) != 0x00)
      {
        // Flash erase error
        BA_ERROR_HANLDER(BA_ERROR_FLASH_ERASE, 0);
        return 0;
      }

      // Program flash memory
      result = FW_ProgramFlash();

      // Download Done. Close file
      f_close(&FlashFile);
    }
  }
  else
  {
    // The binary file is not available
    BA_ERROR_HANLDER(BA_ERROR_FLASH_NOFILE, 0);
    return 0;
  }

  return result;
}

/*
 * @brief  Write Flash to Memory
 * @param  None
 * @retval None
 */
static uint8_t FW_ProgramFlash()
{
  uint8_t result = 1;
  uint32_t programcounter = 0x00;
  uint8_t readflag = SET;
  uint16_t bytesread;

  HAL_FLASH_Unlock();

  /* RAM Address Initialization */
  RamAddress = (uint32_t) &RAM_Buf;

  /* Erase address init */
  LastPGAddress = APPLICATION_ADDRESS;

  /* While file still contain data */
  while ((readflag == SET))
  {
    /* Read maximum 512 Kbyte from the selected file */
    f_read (&FlashFile, RAM_Buf, BUFFER_SIZE, (void *)&bytesread);

    /* Temp variable */
    TmpReadSize = bytesread;

    /* The read data < "BUFFER_SIZE" Kbyte */
    if(TmpReadSize < BUFFER_SIZE)
    {
      readflag = RESET;
    }

    /* Program flash memory */
    for(programcounter = TmpReadSize; programcounter != 0; programcounter -= 4)
    {
      TmpProgramCounter = programcounter;
      /* Write word into flash memory */
      if( FLASH_If_Write((LastPGAddress- TmpProgramCounter + TmpReadSize), *(uint32_t *)(RamAddress - programcounter + TmpReadSize )) != 0x00)
      {
        // Flash programming error
        BA_ERROR_HANLDER(BA_ERROR_FLASH_PROGRAM, 0);
        return 0;
      }
    }
    /* Update last programmed address value */
    LastPGAddress = LastPGAddress + TmpReadSize;
  }

  HAL_FLASH_Lock();
  return result;
}


/**
  * @brief  Gets Flash readout protection status.
  * @param  None
  * @retval ReadOut protection status
  */
FlagStatus FLASH_If_ReadOutProtectionStatus(void)
{
  FlagStatus readoutstatus = RESET;

  FLASH_OBProgramInitStruct.RDPLevel = OB_RDP_LEVEL;

  HAL_FLASHEx_OBGetConfig(&FLASH_OBProgramInitStruct);

  if(OB_RDP_LEVEL == SET)
  {
    readoutstatus = SET;
  }
  else
  {
    readoutstatus = RESET;
  }

  return readoutstatus;
}

/**
  * @brief  Erases the required FLASH Sectors.
  * @param  Address: Start address for erasing data
  * @retval 0: Erase sectors done with success
  *         1: Erase error
  */
uint8_t FLASH_If_EraseSectors(uint32_t Address)
{
  /* Erase the user Flash area
    (area defined by APPLICATION_ADDRESS and USER_FLASH_LAST_PAGE_ADDRESS) ****/

  if(Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    /* Get the 1st sector to erase */
    FirstSector = FLASH_If_GetSectorNumber(Address);
    /* Get the number of sector to erase from 1st sector */
    NbOfSectors = FLASH_If_GetSectorNumber(USER_FLASH_LAST_PAGE_ADDRESS) - FirstSector + 1;

    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    FLASH_EraseInitStruct.Sector = FirstSector;
    FLASH_EraseInitStruct.NbSectors = NbOfSectors;
    FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if(HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &SectorError) != HAL_OK)
      return (1);
  }
  else
  {
    return (1);
  }

  return (0);
}

/**
  * @brief  Writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  Address: Start address for writing data buffer
  * @param  Data: Pointer on data buffer
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  */
uint8_t FLASH_If_Write(uint32_t Address, uint32_t Data)
{
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and APPLICATION_ADDRESS) ***********/

  if(Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data)!= HAL_OK)
      return (1);
  }
  else
  {
    return (1);
  }

  return (0);
}

/**
  * @brief  Returns the Flash sector Number of the address
  * @param  None
  * @retval The Flash sector Number of the address
  */
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address)
{
  uint32_t sector = 0;

  if(Address < ADDR_FLASH_SECTOR_1 && Address >= ADDR_FLASH_SECTOR_0)
  {
    sector = FLASH_SECTOR_0;
  }
  else if(Address < ADDR_FLASH_SECTOR_2 && Address >= ADDR_FLASH_SECTOR_1)
  {
    sector = FLASH_SECTOR_1;
  }
  else if(Address < ADDR_FLASH_SECTOR_3 && Address >= ADDR_FLASH_SECTOR_2)
  {
    sector = FLASH_SECTOR_2;
  }
  else if(Address < ADDR_FLASH_SECTOR_4 && Address >= ADDR_FLASH_SECTOR_3)
  {
    sector = FLASH_SECTOR_3;
  }
  else if(Address < ADDR_FLASH_SECTOR_5 && Address >= ADDR_FLASH_SECTOR_4)
  {
    sector = FLASH_SECTOR_4;
  }
  else if(Address < ADDR_FLASH_SECTOR_6 && Address >= ADDR_FLASH_SECTOR_5)
  {
    sector = FLASH_SECTOR_5;
  }
  else if(Address < ADDR_FLASH_SECTOR_7 && Address >= ADDR_FLASH_SECTOR_6)
  {
    sector = FLASH_SECTOR_6;
  }
  else if(Address < ADDR_FLASH_SECTOR_8 && Address >= ADDR_FLASH_SECTOR_7)
  {
    sector = FLASH_SECTOR_7;
  }
  else if(Address < ADDR_FLASH_SECTOR_9 && Address >= ADDR_FLASH_SECTOR_8)
  {
    sector = FLASH_SECTOR_8;
  }
  else if(Address < ADDR_FLASH_SECTOR_10 && Address >= ADDR_FLASH_SECTOR_9)
  {
    sector = FLASH_SECTOR_9;
  }
  else if(Address < ADDR_FLASH_SECTOR_11 && Address >= ADDR_FLASH_SECTOR_10)
  {
    sector = FLASH_SECTOR_10;
  }
  else if(Address < ADDR_FLASH_SECTOR_12 && Address >= ADDR_FLASH_SECTOR_11)
  {
    sector = FLASH_SECTOR_11;
  }
  else if(Address < ADDR_FLASH_SECTOR_13 && Address >= ADDR_FLASH_SECTOR_12)
  {
    sector = FLASH_SECTOR_12;
  }
  else if(Address < ADDR_FLASH_SECTOR_14 && Address >= ADDR_FLASH_SECTOR_13)
  {
    sector = FLASH_SECTOR_13;
  }
  else if(Address < ADDR_FLASH_SECTOR_15 && Address >= ADDR_FLASH_SECTOR_14)
  {
    sector = FLASH_SECTOR_14;
  }
  else if(Address < ADDR_FLASH_SECTOR_16 && Address >= ADDR_FLASH_SECTOR_15)
  {
    sector = FLASH_SECTOR_15;
  }
  else if(Address < ADDR_FLASH_SECTOR_17 && Address >= ADDR_FLASH_SECTOR_16)
  {
    sector = FLASH_SECTOR_16;
  }
  else if(Address < ADDR_FLASH_SECTOR_18 && Address >= ADDR_FLASH_SECTOR_17)
  {
    sector = FLASH_SECTOR_17;
  }
  else if(Address < ADDR_FLASH_SECTOR_19 && Address >= ADDR_FLASH_SECTOR_18)
  {
    sector = FLASH_SECTOR_18;
  }
  else if(Address < ADDR_FLASH_SECTOR_20 && Address >= ADDR_FLASH_SECTOR_19)
  {
    sector = FLASH_SECTOR_19;
  }
  else if(Address < ADDR_FLASH_SECTOR_21 && Address >= ADDR_FLASH_SECTOR_20)
  {
    sector = FLASH_SECTOR_20;
  }
  else if(Address < ADDR_FLASH_SECTOR_22 && Address >= ADDR_FLASH_SECTOR_21)
  {
    sector = FLASH_SECTOR_21;
  }
  else if(Address < ADDR_FLASH_SECTOR_23 && Address >= ADDR_FLASH_SECTOR_22)
  {
    sector = FLASH_SECTOR_22;
  }
  else if(Address >= ADDR_FLASH_SECTOR_23)
  {
      sector = FLASH_SECTOR_23;
  }

  return sector;
}
