#include "bsp_driver_sd.h"
#include "fatfs.h"
  
static SD_HandleTypeDef hsd;
static HAL_SD_CardInfoTypedef SDCardInfo;


static void SD_MspInit(void);


/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

  /* Check if the SD card is plugged in the slot */
  if (BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR;
  }

  /* HAL SD initialization */
  SD_MspInit();

  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd, &SDCardInfo);

  /* Configure SD Bus width */
    if(sd_state == MSD_OK)
    {
      /* Enable wide operation */
      if(HAL_SD_WideBusOperation_Config(&hsd, SDIO_BUS_WIDE_4B) != SD_OK)
      {
    	  sd_state = MSD_ERROR;
      }
      else
      {
    	  sd_state = MSD_OK;
      }
    }

    //if (sd_state == MSD_OK) return FATFS_Init();

    return sd_state;
}

/******************************************************//**
 * @brief Initializes the SD MSP.
 * @param  None
 * @retval SD None
 **********************************************************/
static void SD_MspInit(void)
{
  static DMA_HandleTypeDef dmaRxHandle;
  static DMA_HandleTypeDef dmaTxHandle;
  GPIO_InitTypeDef GPIO_Init_Structure;

  /* Enable SDIO clock */
  __SDIO_CLK_ENABLE();

  /* Enable DMA2 clocks */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Enable GPIOs clock */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_SDIO;

  /* GPIOC configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;

  HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);


  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(SDIO_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);

  /* Configure DMA Rx parameters */
  dmaRxHandle.Init.Channel             = BSP_SD_DMAx_Rx_CHANNEL;
  dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaRxHandle.Init.Mode                = DMA_PFCTRL;
  dmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  dmaRxHandle.Instance = BSP_SD_DMAx_Rx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(&hsd, hdmarx, dmaRxHandle);

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaRxHandle);

  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaRxHandle);

  /* Configure DMA Tx parameters */
  dmaTxHandle.Init.Channel             = BSP_SD_DMAx_Tx_CHANNEL;
  dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaTxHandle.Init.Mode                = DMA_PFCTRL;
  dmaTxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  dmaTxHandle.Instance = BSP_SD_DMAx_Tx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(&hsd, hdmatx, dmaTxHandle);

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaTxHandle);

  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaTxHandle);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(BSP_SD_DMAx_Rx_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(BSP_SD_DMAx_Rx_IRQn);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(BSP_SD_DMAx_Tx_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(BSP_SD_DMAx_Tx_IRQn);

}


/******************************************************//**
 * @brief  SD card Hw de-initialisation
 * @param None
 * @retval  SD status
 **********************************************************/
uint8_t BSP_SD_DeInit(void)
{
	uint8_t SD_state = MSD_OK;

	if( HAL_SD_DeInit(&hsd) != HAL_OK)
	{
		SD_state = MSD_ERROR;
	}

	  /* Disable SDIO clock */
	  __SDIO_CLK_DISABLE();

	  /* Disable DMA2 clocks */
	  //__BSP_BSP_SD_DMAx_TxRx_CLK_ENABLE();

	return SD_state;

}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0 in success otherwise 1. 
  */
uint8_t BSP_SD_ITConfig(void)
{  
	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = BSP_SD_DETECT_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(BSP_SD_DETECT_PORT, &GPIO_InitStruct);

	  /* Set Priority of External Line Interrupt used for the SD detect interrupt*/
	  HAL_NVIC_SetPriority(BSP_SD_DETECT_IRQn, BSP_SD_DETECT_PRIORITY, 0);

	  /* Enable the External Line Interrupt used for the SD detect  interrupt*/
	  HAL_NVIC_EnableIRQ(BSP_SD_DETECT_IRQn);

  return (uint8_t)0;
}


void EXTI15_10_IRQHandler(void)
{
	SD_DetectIRQHandler();
}


/** @brief  SD detect IT treatment
  */
void BSP_SD_DetectIT()
{
		//HAL_Delay(100);  //IS A DELAY NECCESSARY?

		if( BSP_SD_IsDetected() == SD_PRESENT)  //when low, card is in
		{
			/* After sd disconnection, a SD Init is required + FAT link */

			//BSP_SD_Init();
			//BA_STATUS_LED_SET(GPIO_PIN_SET);

			//USB_SendFromIRQ("Card In");
		}
		else
		{
			//FATFS_DeInit();
			//BSP_SD_DeInit();
			//BA_STATUS_LED_SET(GPIO_PIN_RESET);
			//USB_SendFromIRQ("Card Out");
		}
}


/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t sd_state;
  if(HAL_SD_ReadBlocks(&hsd, pData, ReadAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    sd_state = MSD_ERROR;
  }
  else
  {
    sd_state = MSD_OK;
  }
  return sd_state;  
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode. 
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t sd_state;
  if(HAL_SD_WriteBlocks(&hsd, pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)  
  {
    sd_state = MSD_ERROR;
  }
  else
  {
    sd_state = MSD_OK;
  }
  return sd_state;  
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;
  
  /* Read block(s) in DMA transfer mode */
  if(HAL_SD_ReadBlocks_DMA(&hsd, pData, ReadAddr, BlockSize, NumOfBlocks) != SD_OK)  
  {
    sd_state = MSD_ERROR;
  }
  
  /* Wait until transfer is complete */
  if(sd_state == MSD_OK)
  {
    if(HAL_SD_CheckReadOperation(&hsd, (uint32_t)SD_DATATIMEOUT) != SD_OK)  
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      sd_state = MSD_OK;
    }
  }
  
  return sd_state; 
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;
  
  /* Write block(s) in DMA transfer mode */
  if(HAL_SD_WriteBlocks_DMA(&hsd, pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)  
  {
    sd_state = MSD_ERROR;
  }
  
  /* Wait until transfer is complete */
  if(sd_state == MSD_OK)
  {
    if(HAL_SD_CheckWriteOperation(&hsd, (uint32_t)SD_DATATIMEOUT) != SD_OK)  
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      sd_state = MSD_OK;
    }
  }
  
  return sd_state; 
}

/**
  * @brief  Erases the specified memory area of the given SD card. 
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t BSP_SD_Erase(uint64_t StartAddr, uint64_t EndAddr)
{
  uint8_t sd_state;
  if(HAL_SD_Erase(&hsd, StartAddr, EndAddr) != SD_OK)  
  {
    sd_state = MSD_ERROR;
  }
  else
  {
    sd_state = MSD_OK;
  }
  return sd_state;
}

/**
  * @brief  Handles SD card interrupt request.
  */
void SDIO_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd);
}

/**
  * @brief  Handles SD DMA Tx transfer interrupt request.
  */
void DMA2_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hsd.hdmatx); 
}

/**
  * @brief  Handles SD DMA Rx transfer interrupt request.
  */
void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hsd.hdmarx);
}


/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  *            @arg  SD_TRANSFER_ERROR: Data transfer error 
  */
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void)
{
  return(HAL_SD_GetStatus(&hsd));
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  */
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef* CardInfo)
{
  /* Get SD card Information */
  HAL_SD_Get_CardInfo(&hsd, CardInfo);
}
/* USER CODE END 0 */

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_IsDetected(void)
{
   if (HAL_GPIO_ReadPin(BSP_SD_DETECT_PORT,BSP_SD_DETECT_PIN)) return SD_NOT_PRESENT;  //it's high when card is out
  else return SD_PRESENT;
}


