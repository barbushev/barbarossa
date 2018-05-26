/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "USB_Task.h"

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[USB_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[USB_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
USBD_CDC_LineCodingTypeDef LineCoding =
{ 921600, /* baud rate*/
0x00, /* stop bits-1*/
0x00, /* parity - none*/
0x08 /* nb. of bits 8*/
};
/* USER CODE END PRIVATE_VARIABLES */


  extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
	  memcpy(&LineCoding, pbuf, sizeof(LineCoding));
	  /*
	  LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
	  LineCoding.format     = pbuf[4];
	  LineCoding.paritytype = pbuf[5];
	  LineCoding.datatype   = pbuf[6];
	    */
    break;

  case CDC_GET_LINE_CODING:     
	  memcpy(pbuf, &LineCoding, sizeof(LineCoding));
	  /*
	  pbuf[0] = (uint8_t)(LineCoding.bitrate);
	  pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
	  pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
	  pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
	  pbuf[4] = LineCoding.format;
	  pbuf[5] = LineCoding.paritytype;
	  pbuf[6] = LineCoding.datatype;
	    */
    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
	//if it queued ok, all is well
	if (USB_Receive(Buf) == pdPASS)
	{
		USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);   //reset the buffer position back to 0

		while (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_BUSY) __NOP();  //wait until not busy
	}
	else  //if queue was full, stop receiving. To continue receiving CDC_ContinueReceive() will be called from a task
	{
		USB_Pause_Receive(); //pause reception
	}

  return (USBD_OK);
  /* USER CODE END 6 */ 
}


//used to continue USB reception - this is paused when USB receive queue is full
void CDC_ResumeReceive(void)
{
	//no point of blocking or checking outcome.
	//At this point it is guaranteed that there is at least one position left in the queue
	USB_SendFromTask(0, BA_msgID_Error, (char *)UserRxBufferFS);  //send an error message

	//re-enable recv.
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[0]);
	while (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_BUSY) __NOP();  //wait until not busy
}

uint8_t CDC_USB_TransmitMessage(const char *msgID, const char * msgLen, const char *data)
{
	snprintf((char *)UserTxBufferFS, sizeof(UserTxBufferFS), "%s%s%s%s", msgID, msgLen, data, "\0");

	USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
	uint8_t result = USBD_OK;
	uint16_t totalLen =  strlen((char *)UserTxBufferFS) + 1; //the +1 is for the additional NULL character

	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, totalLen);

	result |= USBD_CDC_TransmitPacket(&hUsbDeviceFS);

	while(hcdc->TxState);   //wait for data to be transmitted

	if ((totalLen%64==0) && (result==USBD_OK))
	{
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
		result |= USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	}

	return result;
}


/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/*******************************************************************************
 * Function Name  : usbIsConfigured.
 * Description    : Determines if USB VCP is configured or not
 * Input          : None.
 * Output         : None.
 * Return         : True if configured.
 *******************************************************************************/
uint8_t usbIsConfigured(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

/*******************************************************************************
 * Function Name  : usbIsConnected.
 * Description    : Determines if USB VCP is connected or not
 * Input          : None.
 * Output         : None.
 * Return         : True if connected.
 *******************************************************************************/
uint8_t usbIsConnected(void)
{
    return (hUsbDeviceFS.dev_state != USBD_STATE_DEFAULT);
}

/*******************************************************************************
 * Function Name  : CDC_BaudRate.
 * Description    : Get the current baud rate
 * Input          : None.
 * Output         : None.
 * Return         : Baud rate in bps
 *******************************************************************************/
uint32_t CDC_BaudRate(void)
{
    return LineCoding.bitrate;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/*
void CDC_Reset_Receive(void)
{
    cdc_received_tot = 0;
    cdc_received = 0;
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}


void USB_SEND(const char *string, ...)
{
char buf[256];
va_list arglist;
va_start(arglist,string);
uint8_t length = vsprintf(buf,string,arglist);
va_end(arglist);

while (CDC_Transmit_FS((uint8_t *)buf, length));

}
*/
