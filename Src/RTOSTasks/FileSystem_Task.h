/*
 * FileSystem_Task.h
 *
 *  Created on: Mar 16, 2017
 *      Author: Ted
 */

#ifndef RTOSTASKS_FILESYSTEM_TASK_H_
#define RTOSTASKS_FILESYSTEM_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"
#include "bsp_driver_sd.h"
#include "fatfs.h"
#include "HEATER_CTRL_Task.h"
#include "file_operations.h"
#include "RTC.h"
#include "barbarossa_config.h"
#include "string.h"

void FileSystem_TaskStart();
void FileSystem_ReadDir(const char *cPath);
void FileSystem_StartLoggingData(const char *FileName);
void FileSystem_StopLoggingData();
uint8_t FileSystem_IsLoggingDataEnabled();
uint8_t FileSystem_IsLoadingAFlow();
void FileSystem_ReadDir(const char *cPath);
void FileSystem_ReadFile(const char *cFileName);
void FileSystem_WriteFile(const char *cFileName, const char *cFileData);
void FileSystem_CreateFolder(const char *cFolderName);
void FileSystem_DeleteFolderOrFile(const char *cFolderName);
void FileSystem_RenameFolderOrFile(const char *cFileNames);
void FileSystem_LoadFlow(const char *FileName);
void FileSystem_LogError(fs_errorlog_struct_t *ErrorToLog);

#define MAXPATHLENGTH		81	//max size for the path of a directory
#if !_USE_LFN
#define MAXFILENAMELENGTH	13	//max size for a filename
#else
#define MAXFILENAMELENGTH	256	//not sure what a long file name length is(?)
#endif
#define MAXFILELINESIZE		256	//max size for a line to be read from a file
#define MAXDATE 			11	//max size for an ascii representation of a file date
#define MAXTIME				6	//max size for an ascii representation of a file time
#define MAXSIZE				15	//max size for an ascii representation of file size

#endif /* RTOSTASKS_FILESYSTEM_TASK_H_ */
