/*
 * USB_Task.c
 *
 *  Created on: Feb 15, 2017
 *      Author: Ted
 */

#include "USB_Task.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"  //for MX_USB_DEVICE_Init()
#include "stdarg.h" // used for vargs
#include "ctype.h"    //used to "toupper"

#include "fatfs.h"
#include "file_operations.h"
#include "RTC.h"

#include "HEATER_CTRL_Task.h"

#include "Door_Sensor.h"
#include "Mixer_Control.h"
#include "L6470.h"

#include "FlowRunner_Task.h"
#include "FileSystem_Task.h"

#include "FirmwareUpgrade.h"


//private variables
static osThreadId USBControl_TaskHandle;
static QueueHandle_t xUSB_Transmit_Queue;
static QueueHandle_t xUSB_Receive_Queue;
static QueueSetHandle_t xUSB_Queue_Set;
static SemaphoreHandle_t xUSB_Receive_IsPaused;  //in case of a xUSB_Receive_Queue overflow, this will be set

//private functions
static void USBControl(void const * argument);
static void StringToUpper(uint8_t *pdata);

static void USB_SendPrivate(uint8_t msgID, const char *data);
static void USB_SendRaw(const char *string, ...);


static void USB_uint8_to_string(uint8_t value, char *retval);   //converts uint8_t to string and padds with zeros. Example 6 will be "006"
static void USB_uint16_to_string(uint16_t value, char *retval);	//converts uint16_t to string and padds with zeros. Example 23 will be "0023"

void USB_TaskStart()
{
	  /* init code for USB_DEVICE */
	  MX_USB_DEVICE_Init();

	  /* Create the semaphores(s) */
      //osSemaphoreDef(xUSB_Receive_Paused);
	  //xUSB_Receive_IsPaused = osSemaphoreCreate(osSemaphore(xUSB_Receive_Paused), 1);

	  xUSB_Receive_IsPaused = xSemaphoreCreateBinary();

	  /* definition and creation of USBControl_Task */
	  osThreadDef(USBControl_Task, USBControl, osPriorityNormal, 0, 2048);   //osPriorityBelowNormal
	  USBControl_TaskHandle = osThreadCreate(osThread(USBControl_Task), NULL);

	  //The USB transmit and receive queue will hold a max of 5 messages. */
	  xUSB_Transmit_Queue = xQueueCreate( 5, sizeof(ba_usbpacket_t));
	  xUSB_Receive_Queue  = xQueueCreate( 5, sizeof(ba_usbpacket_t));


	 // xUSB_Received_Semaphore = xSemaphoreCreateBinary();
	  xUSB_Queue_Set = xQueueCreateSet( 2 * 10 );   //a set of 2 queues times 10 elements each
	  xQueueAddToSet( xUSB_Transmit_Queue, xUSB_Queue_Set );
	  xQueueAddToSet( xUSB_Receive_Queue, xUSB_Queue_Set );

	  vQueueAddToRegistry(xUSB_Queue_Set, "USB Queue");
}


/* USBControl - USB Gatekeeper Task, any requests to send stuff via USB are queued.
 * Any received data is delegated to appropriate tasks. */
void USBControl(void const * argument)
{
  /* USER CODE BEGIN USBControl */

 uint8_t sendstatus = 0;

  /* Infinite loop */
  for(;;)
  {
	  QueueHandle_t xQueueThatContainsData;

	  /* Block on the queue set indefinitely to wait for one of the queues in the set to contain data.
	   Cast the QueueSetMemberHandle_t value returned from xQueueSelectFromSet() to a
	   QueueHandle_t, as it is known all the members of the set are queues (the queue set
	   does not contain any semaphores). */

	  xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet( xUSB_Queue_Set, pdMS_TO_TICKS(100) );  //portMAX_DELAY

	  if (xQueueThatContainsData == NULL)
	  {
		  if (sendstatus)
		  {
			  int32_t Temps[4], Dutys[4];
			  for (uint8_t i = 0; i < 4; i++)
			  	  {
				  Temps[i] = (int32_t)(HEATER_CTRL_GetCurrentTemp(i) * 100000);
				  Dutys[i] = (int32_t)HEATER_CTRL_GetCurrentDuty(i);
			  	  }


		  char dataStream[256];
		  snprintf(dataStream, sizeof(dataStream), "%ld|%ld|%ld|%ld|%ld|%ld|%ld|%ld|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%lu",
				  Temps[0], Dutys[0], Temps[1], Dutys[1], Temps[2], Dutys[2], Temps[3], Dutys[3],
				  Fluid_Sensor_GetValue(0), Fluid_Sensor_GetValue(1), Fluid_Sensor_GetValue(2), Fluid_Sensor_GetValue(3),
				  Fluid_Sensor_GetValue(4), Fluid_Sensor_GetValue(5), Fluid_Sensor_GetValue(6), Fluid_Sensor_GetValue(7),
				  Fluid_Sensor_GetValue(8), Fluid_Sensor_GetValue(9), Fluid_Sensor_GetValue(10),
				  Door_Sensor_GetState(), FlowRunner_IsRunning(), FlowRunner_IsAborting(),FlowRunner_GetCurrentStep(), Mixer_Control_GetRevs());

		  USB_SendPrivate(BA_msgID_StatusData, dataStream);

		  }
		  continue;
	  }


	  if (xQueueThatContainsData == ( QueueSetMemberHandle_t ) xUSB_Receive_Queue)
		  {
		  	  static char cFileName[MAXPATHLENGTH + MAXFILENAMELENGTH];			// local copy of filename extracted from command

		  	  ba_usbpacket_t pcMessageToReceive;
		      xQueueReceive(xUSB_Receive_Queue, &pcMessageToReceive, 0 );

			  //if the semaphore was available immediately, the usb has been paused
			  if (xSemaphoreTake( xUSB_Receive_IsPaused, 0 ) == pdTRUE)
			  	  {
				  	  //verify queue has available spaces before resuming USB Receive
				  	  if (uxQueueSpacesAvailable(xUSB_Receive_Queue) > 0) CDC_ResumeReceive();
			  	  }



			  //verify entire message was received
			  if ((strlen(pcMessageToReceive.data) + 2 + 4) != pcMessageToReceive.msgLen)				//2 chars for msgID and 4 chars for msgLen
			  {
				  USB_SendRaw("msg Len does not match");
			  }
			  else
			  switch (pcMessageToReceive.msgID)
			  {
			  	  case BA_msgID_Command:
			  	  {
			  		  flow_command_t processed_cmd;

      	  			  if (FlowRunner_ProcessCommand((uint8_t *)pcMessageToReceive.data, &processed_cmd))
			  		  {
      	  				  FlowRunner_Execute(&processed_cmd);
			  		  }
			  		  break;
			  	  }

			  	  case BA_msgID_AddFlow:
			  	  {
			  		  if (FlowRunner_BuildQueue((uint8_t *)pcMessageToReceive.data)) USB_SendRaw("ENQUEUED");
	      	  		  	  else USB_SendRaw("INVALID");
			  		  break;
			  	  }

			  	  case BA_msgID_StartFlow:
			  	  {
	  			  	  //this will be happening in a different task
			  		  if (FlowRunner_IsRunning() == 0)
	  			  	  {
			  			  FlowRunner_StartRun();
			  			  USB_SendRaw("Started...");
	  			  	  }
	  			  	  else
	  			  	  {
	  			  		  USB_SendRaw("Already running...");
	  			  	  }

			  		  break;
			  	  }

			  	  case BA_msgID_ClearFlow:
			  	  {
			  		  USB_SendRaw("%u STEPS CLEARED.",  FlowRunner_ResetQueue());
			  		  break;
			  	  }

			  	  case BA_msgID_AbortFlow:
			  	  {
			  	  	  FlowRunner_AbortRun();
			  	  	  USB_SendRaw("Aborting...");
			  	  	  break;
			  	  }

			  	  case BA_msgID_DoorState:
			  	  {
			  		  USB_SendRaw("Door Status %u",  Door_Sensor_GetState());
			  		  break;
			  	  }

			  	  case BA_msgID_StatusEnable:
			  	  {
			  		sendstatus = 1;
			  		break;
			  	  }

			  	  case BA_msgID_StatusDisable:
			  	  {
			  		sendstatus = 0;
			  		break;
			  	  }

			  	  case BA_msgID_GetMem:
			  	  {
			  		  USB_SendRaw("Available Heap Memory: %u", xPortGetFreeHeapSize());
			  		  USB_SendRaw("Baud rate: %u", CDC_BaudRate());
			  		  break;
			  	  }

			  	  case BA_msgID_LoadFlow:
			  	  {
			  		  if (!FileSystem_IsLoadingAFlow())	//check if a flow is already being loaded
			  		  {
	 	 	 	  		  FileSystem_LoadFlow(pcMessageToReceive.data);
	 	 	 	  		  USB_SendRaw("Loading...");
			  		  }
			  		  else
			  		  {
			  			  USB_SendRaw("busy..");
			  		  }

			  		  break;
			  	  }

			  	  case BA_msgID_LogEnable:
			  	  {
			  		 if (!FileSystem_IsLoggingDataEnabled())
			  		 {
			  			 FileSystem_StartLoggingData(pcMessageToReceive.data);
			  			 USB_SendRaw("Logging enabled.");
			  		 }
			  		 else
			  		 {
			  			 USB_SendRaw("Already logging.");
			  		 }

			  		 break;
			  	  }

			  	  case BA_msgID_LogDisable:
			  	  {
			  		  FileSystem_StopLoggingData();
			  		  USB_SendRaw("Logging disabled.");

			  		break;
			  	  }

			  	  case BA_msgID_ReadDir: 								// Command to read directory
			  	  {
			  		  FileSystem_ReadDir(pcMessageToReceive.data);		//path follows "DIR" cmnd, send command to FileSystem task to read directory from path
			  		  break;
			  	  }

			  	  case BA_msgID_ReadFile:								// Command to read a text file
			  	  {
			  		  FileSystem_ReadFile(pcMessageToReceive.data);		// Filename contained in data, send command to FileSystem task to read a text file
			  		  break;
			  	  }

			  	  case BA_msgID_WriteFile: 								// Command to write a text file
			  	  {
			  		  strcpy(cFileName,pcMessageToReceive.data);						// extract the file name

			  		  break;
			  	  }

			  	  case BA_msgID_StreamWriteFile:
			  	  {
			  		  if (cFileName != NULL) FileSystem_WriteFile((const char *)cFileName, (const char *)pcMessageToReceive.data);
			  		  break;
			  	  }

			  	  case BA_msgID_StreamWriteFileEnd:
			  	  {
			  		  cFileName[0] = '\0';
			  		  break;
			  	  }

			  	  case BA_msgID_CreateFolder:
			  	  {
			  		  FileSystem_CreateFolder(pcMessageToReceive.data);
			  		  break;
			  	  }

			  	  case BA_msgID_RenameFileOrFolder:
			  	  {
			  		  FileSystem_RenameFolderOrFile(pcMessageToReceive.data);
			  		  break;
			  	  }

			  	  case BA_msgID_DeleteFileOrFolder:
			  	  {
			  		  FileSystem_DeleteFolderOrFile(pcMessageToReceive.data);
			  		  break;
			  	  }

			  	  case BA_msgID_FirmwareVersion:
			  	  {
			  		  USB_SendPrivate(BA_msgID_FirmwareVersion, BA_FIRMWARE_VERSION);
			  		  break;
			  	  }

			  	  case BA_msgID_FirmwareUpgradeStart:
			  	  {
			  		  //taskENTER_CRITICAL();
			  		  if (FW_FlashFileToMemory(pcMessageToReceive.data)) USB_SendPrivate(BA_msgID_FirmwareUpgradeComplete, NULL);

			  		  //taskEXIT_CRITICAL();
			  		  break;
			  	  }

			  	  case BA_msgID_Reboot:
			  	  {
			  		  NVIC_SystemReset();
			  		  break;
			  	  }

			  	  case BA_msgID_RunFirmwareLoader:
			  	  {

			  		  BKPSRAM_Write8(BA_FIRMWARE_UPDATE_ADDR, BA_FIRMWARE_UPDATE_FLAG);  //set a flag in Backup SRAM which will be read during SystemInit
			  		  NVIC_SystemReset(); // Reset the processor
			  		  break;
			  	  }

			  	  default:
			  	  {
			  		  USB_SendRaw("Invalid ID");
			  	  }
			  }
		  }
	  else if (xQueueThatContainsData == ( QueueSetMemberHandle_t ) xUSB_Transmit_Queue)
	  	  {
		  	  if (usbIsConnected())
		  	  {
		  		  ba_usbpacket_t pcMessageToTransmit;
		  		  xQueuePeek(xUSB_Transmit_Queue, &pcMessageToTransmit, 0);  //peek at the message

		  		  USB_SendPrivate(pcMessageToTransmit.msgID, pcMessageToTransmit.data);

		  		  xQueueReceive( xUSB_Transmit_Queue, &pcMessageToTransmit, 0 );
		  	  }
	  	  }
	  }
}

/********************************************************
 * <Info> Send data over USB. Not Task safe - internal use only.
 * <msgID> Message ID of ba_usb_id_t
 * <data> String with variable arguments (data to be sent)
 **********************************************************/
static void USB_SendPrivate(uint8_t msgID, const char *data)
{
	//msg ID is 2 chars, msgLen is 4 chars.
	char msgIDchar[3], msgLen[5];

	uint16_t DataLen = strlen(data);
	USB_uint8_to_string(msgID, msgIDchar);							//convert the msgID to a string
	USB_uint16_to_string(DataLen + 2 , msgLen);		//convert the length to a string. The 10 is for 2 msgID chars, 4 length chars, 3 checksum chars

	//construct the full frame
	CDC_USB_TransmitMessage(msgIDchar, msgLen, data);
}


/********************************************************
 * <Info> Send data over USB. Not Task safe - internal use only.
 * <string> String with variable arguments (data to be sent)
 **********************************************************/
static void USB_SendRaw(const char *string, ...)
{
	char buf[256];
	va_list arglist;
	va_start(arglist,string);
	uint8_t length = vsprintf(buf,string,arglist);
	va_end(arglist);

	while (CDC_Transmit_FS((uint8_t *)buf, length));
	while (CDC_Transmit_FS((uint8_t *)"\0",    1));
}



/********************************************************
 * <Info> Queue data to be transmitted over USB. Call from Tasks only.
 * <maxBlock> Maximum time in ms to block if the queue is full
 * <string> String with variable arguments (data to be sent)
 * <returns> pdPASS (1) or errQUEUE_FULL (0)
 **********************************************************/
BaseType_t USB_SendFromIRQ(char *data)
{
	return xQueueSendToBackFromISR(xUSB_Transmit_Queue, &data, NULL);
}


/********************************************************
 * <Info> Queue data to be transmitted over USB. Call from Tasks only.
 * <maxBlock> Maximum time in ms to block if the queue is full
 * <msgID> The ID number of the message. Use ba_usb_id_t enums.
 * <string> String with variable arguments (data to be sent)
 * <returns> pdPASS (1) or errQUEUE_FULL (0)
 **********************************************************/
BaseType_t USB_SendFromTask(uint32_t BlockTime,uint8_t msgID, const char *data)
{
	ba_usbpacket_t dataPacket;

	strcpy(dataPacket.data, data);
	//dataPacket.data = data;

	dataPacket.msgID = msgID;
	dataPacket.msgLen = 2 + 4 + strlen(data);

	return xQueueSendToBack(xUSB_Transmit_Queue, &dataPacket, BlockTime );
}


/********************************************************
 * <Info> Queue received data over USB for processing.
 * 		  This is called from CDC_Receive_FS() only
 * <data> Pointer to the received data
 **********************************************************/
BaseType_t USB_Receive(uint8_t *data)
{
	ba_usbpacket_t dataPacket;

	dataPacket.msgID = ((data[0] - '0') * 10) + ((data[1] - '0')); //get the message id
	dataPacket.msgLen = ((data[2] - '0') * 1000) + ((data[3] - '0') * 100) + ((data[4] - '0') * 10) + ((data[5] - '0')); //get the message length

	strcpy(dataPacket.data , (char *)&data[6]);
	//dataPacket.data = (char *)&data[6]; //get the message data

	return xQueueSendToBackFromISR(xUSB_Receive_Queue, &dataPacket, NULL) ;
}


/********************************************************
 * <Info> USB Receive is paused due to xUSB_Receive_Queue being full
 * 		  This is called from CDC_Receive_FS() only
 * <data> Pointer to the received data
 **********************************************************/
void USB_Pause_Receive()
{
	xSemaphoreGive(xUSB_Receive_IsPaused);
}


/********************************************************
 * <Info> Converts a uint8_t (max 99) to string padded with 0s. Example 6 will return "06"
 * <value> the uint8_t value to be converted to string
 * <*retval> Pointer to char array to store the conversion - has to be 4 chars long
 **********************************************************/
static void USB_uint8_to_string(uint8_t value, char *retval)
{
	value %= 100;  //get rid of anything above 100
	retval[0] = (value / 10) + '0';
	value %= 10;
	retval[1] = value + '0';
	retval[2] = '\0';
}

/********************************************************
 * <Info> Converts a uint16_t (max 9999) to string padded with 0s. Example 32 will return "0032"
 * <value> the uint16_t value to be converted to string
 * <*retval> Pointer to char array to store the conversion - has to be 5 chars long
 **********************************************************/
static void USB_uint16_to_string(uint16_t value, char *retval)
{
	retval[0] = (value / 1000) + '0';
	value %= 1000;
	retval[1] = (value / 100) + '0';
	value %= 100;
	retval[2] = (value / 10) + '0';
	value %= 10;
	retval[3] = value + '0';
	retval[4] = '\0';
}

void StringToUpper(uint8_t *pdata)
{
	 for ( ; *pdata; ++pdata) *pdata = toupper(*pdata);
}
