/*
 * USB_Task.h
 *
 *  Created on: Feb 15, 2017
 *      Author: Ted
 */

#ifndef RTOSTASKS_USB_TASK_H_
#define RTOSTASKS_USB_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

void USB_TaskStart();

BaseType_t USB_SendFromTask(uint32_t BlockTime,uint8_t msgID, const char *data);
BaseType_t USB_SendFromIRQ(char *data);

//the following two are called from CDC_Receive_FS only
BaseType_t USB_Receive(uint8_t *data);
void USB_Pause_Receive();



typedef enum
{
	BA_msgID_Command,
	BA_msgID_AddFlow,
	BA_msgID_StartFlow,
	BA_msgID_ClearFlow,
	BA_msgID_AbortFlow,
	BA_msgID_LoadFlow,
	BA_msgID_DoorState,
	BA_msgID_StatusEnable,
	BA_msgID_StatusData,
	BA_msgID_StatusDisable,
	BA_msgID_LogEnable,
	BA_msgID_LogDisable,
	BA_msgID_Data   ,
	BA_msgID_Error  ,
	BA_msgID_Stream ,
	BA_msgID_GetMem,
	BA_msgID_ReadDir,
	BA_msgID_StreamDirList,
	BA_msgID_StreamDirListEnd,
	BA_msgID_ReadFile,
	BA_msgID_StreamReadFile,
	BA_msgID_StreamReadFileEnd,
	BA_msgID_WriteFile,
	BA_msgID_StreamWriteFile,
	BA_msgID_StreamWriteFileRDY,		//used to signal sender that line was written
	BA_msgID_StreamWriteFileEnd,
	BA_msgID_CreateFolder,
	BA_msgID_RenameFileOrFolder,
	BA_msgID_DeleteFileOrFolder,
	BA_msgID_FirmwareVersion,
	BA_msgID_FirmwareUpgradeStart,
	BA_msgID_FirmwareUpgradeComplete,
	BA_msgID_Reboot,
	BA_msgID_RunFirmwareLoader,
}ba_usb_id_t;

typedef struct
{
	uint8_t     msgID;               //char		msgID[3];		//2 for the data and 1 for the null terminator
	uint16_t    msgLen;             //char		length[5];		//4 for the data and 1 for the null terminator
	char	data[512];			//
	//const char	*data;			//
}ba_usbpacket_t;


// USB Transmission protocol {msgID} {length} {data} {checksum}
//								00      34   A0 1 2 3 4 5 6*FF
// - The {msgID} is 2 numeric characters - 00 to 99 and represents the message ID
// - The {length} - 4 characters - 0000 to 9999 and represents the overall length of the packet including the {msgID} and the {checksum}.
// - The {data} may be 9999 - msgID length - Length length = 9999 - 2 - 4 =  9991 characters and is the actual data.


#endif /* RTOSTASKS_USB_TASK_H_ */
