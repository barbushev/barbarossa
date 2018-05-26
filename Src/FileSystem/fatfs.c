#include "fatfs.h"
#include "stm32f4xx_hal_rtc.h"
#include "RTC.h"
#include "file_operations.h"

	  //cardOK = false;
	  //cardReaderInitialized = false;
	  //rootIsOpened = false;
char SD_Path[4];  /* SD logical drive path */

uint8_t FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  uint8_t status = FATFS_LinkDriver(&SD_Driver, SD_Path);
  status |= FILEOP_MountCard();
  return status;
}

uint8_t FATFS_DeInit(void)
{
	  return FATFS_UnLinkDriver(SD_Path);
}


/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
	return RTC_GetFATtime();
}

