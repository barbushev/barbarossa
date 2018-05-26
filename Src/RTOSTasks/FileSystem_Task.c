/*
 * FileSystem_Task.c
 *
 *  Created on: Mar 16, 2017
 *      Author: Ted
 */

#include "FileSystem_Task.h"
#include "fluid_sensor.h"
#include "Mixer_Control.h"
#include "FlowRunner_Task.h"

#include "USB_Task.h"

static osThreadId FileSystem_TaskHandle;
static EventGroupHandle_t xFileSystem_EventGroup;

static char LogFileName[19];  //longest filename name is 8 chars + "." (1 char) + 3 char file extension = 12 char + "/logs/" (6 chars) + "\0" (1 char) = 19
//static char FlowFileName[22]; //same as above, but "/scripts/" instead of "/logs/"

static char FlowFileName[MAXPATHLENGTH + MAXFILENAMELENGTH];
static char FileSystem_cPath[MAXPATHLENGTH];			//local copy of path for directory & file operations
static char FileSystem_cFileName[MAXPATHLENGTH + MAXFILENAMELENGTH];	//local copy of filename
static char FileSystem_cFileNameNew[MAXPATHLENGTH + MAXFILENAMELENGTH];	//local copy of new filename
static char FileSystem_cFileData[512];

//static char *FileSystem_cFileData;						//local copy of pointer to data buffer to write to file

#define FS_EnableDataLogging   (1<<0)    //bit 0          FS = FileSystem
#define FS_LoadFlow            (1<<1)    //bit 1
#define FS_LogErrorToFile      (1<<2)    //bit 2
#define FS_DeleteFolderOrFile  (1<<3)	 //bit 3
#define FS_CreateFolderOnSD	   (1<<4)	 //bit 4
#define FS_ReadDirFromSD	   (1<<5)	 //bit 5
#define FS_WriteToSD           (1<<6)    //bit 6
#define FS_ReadFromSD          (1<<7)    //bit 7
#define FS_RenameFolderOrFile  (1<<8)	 //bit 8

static QueueHandle_t xFATFS_ErrorLog_Queue;

//static QueueHandle_t xFATFS_Write_Queue;
//static QueueHandle_t xFATFS_Read_Queue;

//private functions
static void FileSystem(void const * argument);
static void FileSystem_LogDataToFile();
static void FileSystem_LogErrorToFile(char *ErrorLogFileName, fs_errorlog_struct_t *ErrorToLog);
static void FileSystem_ReadDirToUSB();
static void FileSystem_ReadTextFileToUSB();
static void FileSystem_WriteTextFileFromUSB();
static void FileSystem_CreateFolderFromUSB();
static void FileSystem_DeleteFolderOrFileFromUSB();
static void FileSystem_RenameFolderOrFileFromUSB();

void FileSystem_TaskStart()
{
	//init hardware init code for FATFS
	//configure interrupt for SD detect
	BSP_SD_ITConfig();

	//Start FatFS and mount card
	if (FATFS_Init() != 0) BA_ERROR_HANLDER(BA_ERROR_SDCARD_INIT, 0);

	//The FATFS error log queue will hold a max of 5 elements.
	xFATFS_ErrorLog_Queue = xQueueCreate( 5, sizeof(fs_errorlog_struct_t));

	//The FATFS read and write queue will hold a max of 5 messages. */
	//xFATFS_Write_Queue = xQueueCreate( 5, USB_TX_DATA_SIZE );
	//xFATFS_Read_Queue  = xQueueCreate( 5, USB_RX_DATA_SIZE );

	/* Create the event group. */
	xFileSystem_EventGroup = xEventGroupCreate();
	/*	Event group map
	 *	BIT_0 = Enable Data logging
	 *	BIT_1 = Load Flow
	 *	BIT_2 = Log Error To File
	 *	BIT_3 =
	 *	BIT_4 =
	 *	BIT_5 = Read directory from SD
	 *	BIT_6 = Read from SD
	 *	BIT_7 = Write to SD
	 */

	/* definition and creation of FileSystem_Task */
	osThreadDef(FileSystem_Task, FileSystem, osPriorityLow, 0, 1024);  //osPriorityBelowNormal
	FileSystem_TaskHandle = osThreadCreate(osThread(FileSystem_Task), NULL);
}

/* FileSystem function */
static void FileSystem(void const * argument)
{
	//on initial startup of this task, check RCC for Watch dog or low power reset flag and log it
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0) BA_ERROR_HANLDER(BA_ERROR_RESET_IWDG, 0);  //Independent Watchdog reset.

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0) BA_ERROR_HANLDER(BA_ERROR_RESET_LOWP, 0);  //Low Power reset.

	__HAL_RCC_CLEAR_RESET_FLAGS();

	  /*
	  @arg RCC_FLAG_BORRST: POR/PDR or BOR reset.
	*            @arg RCC_FLAG_PINRST: Pin reset.
	*            @arg RCC_FLAG_PORRST: POR/PDR reset.
	*            @arg RCC_FLAG_SFTRST: Software reset.
	*            @arg RCC_FLAG_IWDGRST: Independent Watchdog reset.
	*            @arg RCC_FLAG_WWDGRST: Window Watchdog reset.
	*            @arg RCC_FLAG_LPWRRST: Low Power reset.
	*/

  EventBits_t uxBits;

  for(;;)
  {
	  uxBits = xEventGroupWaitBits(xFileSystem_EventGroup,
			  	  	  	  	  	  FS_EnableDataLogging | FS_LoadFlow  | FS_LogErrorToFile     | FS_ReadDirFromSD    |
			  	  	  	  	  	  FS_ReadFromSD 	   | FS_WriteToSD | FS_DeleteFolderOrFile | FS_CreateFolderOnSD |
								  FS_RenameFolderOrFile, pdFALSE, pdFALSE, portMAX_DELAY);  //wait until one of these bits is set


	  if ( (uxBits & FS_EnableDataLogging))  //log to file
	  {
		  FileSystem_LogDataToFile();
		  vTaskDelay(pdMS_TO_TICKS(1000));
	  }
	  else if ((uxBits & FS_LoadFlow))  //load Flow file to memory
	  {
  	  	  FIL fileobj;

		  if (f_open(&fileobj, FlowFileName, FA_READ) == FR_OK)
		  {
			  char line[256];

			  // Read each line and queue it
			  while (f_gets(line, sizeof line, &fileobj))
			  {
				  if (!FlowRunner_BuildQueue((uint8_t *)line)) BA_ERROR_HANLDER(BA_ERROR_SDCARD_FLOW, 0);
			  }

		  /* Close the file */
		  f_close(&fileobj);
		  }
		  else BA_ERROR_HANLDER(BA_ERROR_SDCARD_FLOW, 0);

		  xEventGroupClearBits(xFileSystem_EventGroup, FS_LoadFlow); //clear the bit to indicate flow loading has completed
	  }
	  else if ((uxBits & FS_ReadDirFromSD))
	  {
		  //read directory from SD
		  FileSystem_ReadDirToUSB();
		  xEventGroupClearBits(xFileSystem_EventGroup, FS_ReadDirFromSD); //clear the bit to indicate read dir has completed
	  }
	  else if ((uxBits & FS_ReadFromSD))
	  {
		  //read from SD
		  FileSystem_ReadTextFileToUSB();
		  xEventGroupClearBits(xFileSystem_EventGroup, FS_ReadFromSD); //clear the bit to indicate read line from text file has completed
	  }
	  else if ((uxBits & FS_WriteToSD))
	  {
		 //write to SD
		  FileSystem_WriteTextFileFromUSB();
		  xEventGroupClearBits(xFileSystem_EventGroup, FS_WriteToSD); //clear the bit to indicate write line of text file has completed
	  }
	  else if ((uxBits & FS_CreateFolderOnSD))
	  	  {
	  	 	  //Create folder on SD
	  	 	  FileSystem_CreateFolderFromUSB();
	  	 	  xEventGroupClearBits(xFileSystem_EventGroup, FS_CreateFolderOnSD); //clear the bit to indicate create a folder has completed
	  	  }
	  	  else if ((uxBits & FS_DeleteFolderOrFile))
	  	  {
	  	 	  //Delete folder or file on SD
	  	 	  FileSystem_DeleteFolderOrFileFromUSB();
	  	 	  xEventGroupClearBits(xFileSystem_EventGroup, FS_DeleteFolderOrFile); //clear the bit to indicate Delete a folder or file has completed
	  	  }
	  	  else if ((uxBits & FS_RenameFolderOrFile))
	  	  {
	  	 	  //Rename folder or file on SD
	  	 	  FileSystem_RenameFolderOrFileFromUSB();
	  	 	  xEventGroupClearBits(xFileSystem_EventGroup, FS_RenameFolderOrFile); //clear the bit to indicate Rename a folder or file has completed
	  	  }
	  else if ((uxBits & FS_LogErrorToFile))  //log error to file
	  {
		  fs_errorlog_struct_t ErrorToLog;
		  if ((xQueueReceive(xFATFS_ErrorLog_Queue, &ErrorToLog, 0)) == pdPASS)
		  {
			  RTC_DateTypeDef today;
			  RTC_GetDate(&today);

			  //Date format March 28, 2017 = 03282017
			  uint32_t DateStamp = (today.Month * 1000000) + (today.Date * 10000) + (today.Year + 2000);

			  char ErrorLogFileName[22] = "/errlogs/";

			  if (today.Month < 10)  //pad with a 0
				  {
				  ErrorLogFileName[9] = '0';
				  itoa(DateStamp, &ErrorLogFileName[10], 10);
				  }
			  else itoa(DateStamp, &ErrorLogFileName[9], 10);

			  ErrorLogFileName[17] = '.';
			  ErrorLogFileName[18] = 't';
			  ErrorLogFileName[19] = 'x';
			  ErrorLogFileName[20] = 't';
			  ErrorLogFileName[21] = '\0';

			  FileSystem_LogErrorToFile(ErrorLogFileName, &ErrorToLog);
		  }
		  else xEventGroupClearBits(xFileSystem_EventGroup, FS_LogErrorToFile); //clear the bit to indicate error logging has completed
	  }
  }
}

/***********************************************************************************
 * Reads the contents of a directory on the SD card and sends it across the USB one
 * entry at a time.
 * Input:	*FileSystem_cPath must point to the path of the directory to read from
 * Output:	Contents of the directory pointed to by *FileSystem_cPath gets read and
 * 			sent across	USB one entry at a time
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_ReadDirToUSB()
{
	char cDirBuffer[MAXFILELINESIZE + MAXDATE + MAXTIME + MAXSIZE + 6];	// need room for date, time, size, & filename + spaces
	char cDateTime[MAXDATE + MAXTIME];		// file date & time
	FRESULT result;
	DIR dir;
	FILINFO fno;

	result = f_opendir(&dir, FileSystem_cPath);		// Open the directory
	if (result == FR_OK)
    {
		// sprintf(cDirBuffer, " Directory of %s\n", FileSystem_cPath);	// add to buffer
		// USB_SendFromTask(200, BA_msgID_StreamDirList, cDirBuffer);								// and sent across USB
	    for (;;)
	    {
	    	result = f_readdir(&dir, &fno);			// Read a directory item
	    	if ((result != FR_OK) || (fno.fname[0] == 0))
	    	{	// error or no more entries
    			//USB_SendFromTask(200, BA_msgID_StreamDirListEnd, NULL);	//We need to figure out a better end of transmission
	    		break;  // Break on error or end of dir
	    	}
	    	else
	    	{	// It is a file or subdirectory

	    		// convert date and time
	    		sprintf(cDateTime, "%u/%02u/%02u|%02u:%02u",  //remove /n TB 4/14/17
	    			    				(fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
	    			    		         fno.ftime >> 11, fno.ftime >> 5 & 63);

    			//put it all together
	    		sprintf(cDirBuffer, "%s|%s|%lu|%02x", fno.fname, cDateTime, fno.fsize, fno.fattrib);  //remove /n TB 4/14/17

	    		//fattrib
	    		//Indicates the file/directory attribute in combination of AM_DIR, AM_RDO, AM_HID, AM_SYS and AM_ARC.


/*	    		sprintf(cDateTime, "%u/%02u/%02u  %02u:%02u\n",
	    				(fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
	    		         fno.ftime >> 11, fno.ftime >> 5 & 63);

	    		//print "<DIR>" or filesize
	    		if (fno.fattrib & AM_DIR)
	    		{  // It is a directory
	    			sprintf(cSize, "<DIR>");
	    		}
	    		else //its a file, print size
	    		{
	    			sprintf(cSize, "%ul", fno.fsize);
	    		}

	    		sprintf(cDirBuffer, "%s  %s  %s\n", cDateTime, cSize, fno.fname);
*/
	    		USB_SendFromTask(200, BA_msgID_StreamDirList, cDirBuffer);				// and send across USB

	    	}
	    }
	    f_closedir(&dir);
    }	// end if opendir ok

	if (result != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_DREAD, result);
	}
}



/***********************************************************************************
 * Reads the contents of a text file on the SD card and sends it across the USB one
 * line at a time.
 * Input:	*FileSystem_cFileName must point to filename to read from
 * Output:	Contents of the data file pointed to by *FileSystem_cFileName gets read
 * 			and sent across USB, one line at a time until done
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_ReadTextFileToUSB()
{
	FRESULT results;
	FIL fileobj;
    char cLine[MAXFILELINESIZE];

    // Opens an existing file. If it doesn't exist, return error.
    results = f_open(&fileobj, FileSystem_cFileName, FA_READ | FA_OPEN_EXISTING);
    if (results == FR_OK)
    {
    	for (;;)
    	{
    		if (f_gets(cLine, sizeof cLine, &fileobj) != NULL)		// get a string of text
    		{
    			USB_SendFromTask(200, BA_msgID_StreamReadFile, cLine);						// and send across USB
    		}
    		else	// error or end of file
    		{
    			if (f_eof(&fileobj) == 0)	// if not end of file reached, then error
    			{
    				results = f_error(&fileobj);					// get error if there was one
    			}
    			USB_SendFromTask(200,BA_msgID_StreamReadFileEnd, NULL);	// We need to figure out a better end of transmission
    			break;
    		}
    	}
    	// Close the file
    	f_close(&fileobj);
    }

	if (results != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_FREAD, results);
	}
}

/***********************************************************************************
 * Takes a line of text and appends it to the end of an existing file.  If the file
 * doesn't exist, it will create it.  The file is opened, written to, then closed
 * on each iteration.  It could be optimized by keeping the file open, until the
 * last line it received and written but it may not be as safe.  An alternate way
 * is to implement another interface with USB to get next line.
 * Input:	*FileSystem_cFileName must point to filename to write to
 * 			*FileSystem_cFileData must point to the string of text to write
 * Output:	Data pointed to by *FileSystem_cFileData gets appended to end of
 * 			file on SD card with the name pointed to by *FileSystem_cFileName
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_WriteTextFileFromUSB()
{
	FRESULT results;
	FIL fileobj;

    // Opens an existing file. If not exist, creates a new file
    results = f_open(&fileobj, FileSystem_cFileName, FA_WRITE | FA_OPEN_ALWAYS);
    if (results == FR_OK)
    {
    	results = f_lseek(&fileobj, f_size(&fileobj));	//point to end of file
    	if (results == FR_OK)
    	{

    		if (f_printf(&fileobj, "%s\r\n",FileSystem_cFileData) == EOF)    		////if (f_puts(FileSystem_cFileData, &fileobj) == EOF)		// write a string of text
    		{
    			results = FR_DISK_ERR;					// not sure what else to put here(?)
    		}
    	}
    	f_close(&fileobj);
    }

	if (results != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, results);
	}
	else
	{
		USB_SendFromTask(200, BA_msgID_StreamWriteFileRDY, NULL);
	}
}

/***********************************************************************************
 * Creates a folder on the SD card
 * Input:	*FileSystem_cFileName must point to the pathname of the folder to create
 * Output:	Folder with the path and name pointed to by *FileSystem_cFileName is
 * 			created
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_CreateFolderFromUSB()
{
	FRESULT results;

	results = f_mkdir(FileSystem_cPath);
	if (results != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, results);
	}
}

/***********************************************************************************
 * Deletes a folder or file on the SD card
 * Input:	*FileSystem_cFileName must point to the pathname of the folder to delete
 * Output:	Folder with the path and name pointed to by *FileSystem_cFileName is
 * 			deleted
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_DeleteFolderOrFileFromUSB()
{
	FRESULT results;

	results = f_unlink(FileSystem_cPath);
	if (results != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, results);
	}
}

/***********************************************************************************
 * Renames a folder or file on the SD card
 * Input:	*FileSystem_cFileName must point to the pathname of the folder to rename
 * Output:	Folder with the path and name pointed to by *FileSystem_cFileName is
 * 			renamed
 * Returned:	None, but error is logged if there is an issue
*************************************************************************************/
static void FileSystem_RenameFolderOrFileFromUSB()
{
	FRESULT results;

	results = f_rename(FileSystem_cFileName, FileSystem_cFileNameNew);
	if (results != FR_OK)
	{
	  	BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, results);
	}
}

static void FileSystem_LogDataToFile()
{
	int32_t Temps[4], Dutys[4];
	for (uint8_t i = 0; i < 4; i++)
	{
		Temps[i] = (int32_t)(HEATER_CTRL_GetCurrentTemp(i) * 100000);
		Dutys[i] = (int32_t)HEATER_CTRL_GetCurrentDuty(i);
	}

	FRESULT fr, fileExists;
	FILINFO fno;
    FIL fileobj;

    //check if the file exists
    fileExists = f_stat(LogFileName, &fno);

    // Opens an existing file. If not exist, creates a new file.
    if (f_open(&fileobj, LogFileName, FA_WRITE | FA_OPEN_ALWAYS) == FR_OK)
    {
    	if (fileExists == FR_NO_FILE)
    	{
    	//add the header

    	if (f_printf(&fileobj, "TEMP0   |DTY|TEMP1   |DTY|TEMP2   |DTY|TEMP3   |DTY|FLD0|FLD1|FLD2|FLD3|FLD4|FLD5|FLD6|FLD7|FLD8|FLD9|FL10|MIXER|TimeTick  |\r\n")
    			== -1)
    		{
    			f_close(&fileobj);
    			BA_ERROR_HANLDER(BA_ERROR_SDCARD_DLOG, 1);
    			return;
    		}
    	}

    	// Seek to end of the file to append data
   		fr = f_lseek(&fileobj, f_size(&fileobj));
    	if (fr != FR_OK)
    		{
    			f_close(&fileobj);
    			BA_ERROR_HANLDER(BA_ERROR_SDCARD_DLOG, 0);
    			return;
    		}

    	if (f_printf(&fileobj, "%08d|%03d|%08d|%03d|%08d|%03d|%08d|%03d|%04d|%04d|%04d|%04d|%04d|%04d|%04d|%04d|%04d|%04d|%04d|%05d|%010d|\r\n",
    		Temps[0], Dutys[0], Temps[1], Dutys[1], Temps[2], Dutys[2], Temps[3], Dutys[3],
			Fluid_Sensor_GetValue(0), Fluid_Sensor_GetValue(1), Fluid_Sensor_GetValue(2), Fluid_Sensor_GetValue(3), Fluid_Sensor_GetValue(4), Fluid_Sensor_GetValue(5),
			Fluid_Sensor_GetValue(6), Fluid_Sensor_GetValue(7), Fluid_Sensor_GetValue(8), Fluid_Sensor_GetValue(9), Fluid_Sensor_GetValue(10), Mixer_Control_GetRevs(),
			xTaskGetTickCount()) == -1)	BA_ERROR_HANLDER(BA_ERROR_SDCARD_DLOG, 1);

    	// Close the file
    	f_close(&fileobj);
    } else BA_ERROR_HANLDER(BA_ERROR_SDCARD_DLOG, 0);
}

/*
 * Start logging to /logs/FileName
 */
void FileSystem_StartLoggingData(const char *FileName)
{
	strncpy(&LogFileName[0],"/logs/", 6);
	strcpy(&LogFileName[6], FileName);

	xEventGroupSetBits(xFileSystem_EventGroup, FS_EnableDataLogging);
}

/*
 * Stop Logging
 */
void FileSystem_StopLoggingData()
{
	xEventGroupClearBits(xFileSystem_EventGroup, FS_EnableDataLogging);
}

/*
 * Check if logging is enabled
 */
uint8_t FileSystem_IsLoggingDataEnabled()
{
	EventBits_t uxBits = xEventGroupGetBits(xFileSystem_EventGroup);
	return (uxBits & FS_EnableDataLogging);
}




/***********************************************************************************
 * FileSystem Task entry:   FileSystem_ReadDir
 * Makes a local copy, with task visibility, of the filename, then sets the event
 * task event group to start reading the file
 * Input:	*cFileName must point to filename to write to
 * Output:	Local copies of the input parameter gets stored locally
 * 			Appropriate task event group bits set to begin reading file from SD
 * Returned:	None, but error is logged if the filename is too large
*************************************************************************************/
void FileSystem_ReadDir(const char *cPath)
{
	if (strlen(cPath) < sizeof(FileSystem_cPath))
	{
		strcpy(FileSystem_cPath, cPath);	// make a local copy of filename for task to use
		xEventGroupSetBits(xFileSystem_EventGroup, FS_ReadDirFromSD);	// set task event group to start reading directory from SD
	}
	else
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_DREAD, 1);
	}
}

/***********************************************************************************
 * FileSystem Task entry:   FileSystem_ReadFile
 * Makes a local copy, with task visibility, of the filename, then sets the event
 * task event group to start reading the file
 * Input:	*cFileName must point to filename to write to
 * Output:	Local copies of the input parameter gets stored locally
 * 			Appropriate task event group bits set to begin reading file from SD
 * Returned:	None, but error is logged if the filename is too large
*************************************************************************************/
void FileSystem_ReadFile(const char *cFileName)
{
	if (strlen(cFileName) < sizeof(FileSystem_cFileName))
	{
		strcpy(FileSystem_cFileName,cFileName);	// make a local copy of filename for task to use
		xEventGroupSetBits(xFileSystem_EventGroup, FS_ReadFromSD);	// set task event group to start reading from SD
	}
	else
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_FREAD, 1);
	}
}

/***********************************************************************************
 * FileSystem Task entry:  FileSystem_WriteFile
 * Makes a local copy, with task visibility, of the filename and the pointer to file
 * data, then sets the event task event group to start writing the file
 * Input:	*cFileName must point to filename to write to
 * 			*cFileData must point to the string of text to write
 * Output:	Local copies of the input parameters get stored locally
 * 			Appropriate task event group bits set to begin writing file to SD
 * Returned:	None, but error is logged if the filename is too large
*************************************************************************************/
void FileSystem_WriteFile(const char *cFileName, const char *cFileData)
{
	if (strlen(cFileName) < sizeof(FileSystem_cFileName))
	{
		strcpy(FileSystem_cFileName,cFileName);	// make a local copy of filename for task to use

		strcpy(FileSystem_cFileData, cFileData);

		//FileSystem_cFileData = (char *)cFileData;		// make a local copy of file data pointer
		xEventGroupSetBits(xFileSystem_EventGroup, FS_WriteToSD);	// set task event group to start writing to SD
	}
	else
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
	}
}

/***********************************************************************************
 * FileSystem Task entry:  FileSystem_CreateFolder
 * Makes a local copy, with task visibility, of the folder path, then sets the event
 * task event group to start creating the folder
 * Input:	*cFolderName must point to path name of the folder to create
 * Output:	Local copy of the input parameter get stored locally
 * 			Appropriate task event group bits set to begin creating a folder on SD
 * Returned:	None, but error is logged if the path is too large
*************************************************************************************/
void FileSystem_CreateFolder(const char *cFolderName)
{
	if (strlen(cFolderName) < sizeof(FileSystem_cPath))
	{
		strcpy(FileSystem_cPath,cFolderName);	// make a local copy of folder for task to use
		xEventGroupSetBits(xFileSystem_EventGroup, FS_CreateFolderOnSD);	// set task event group to start creating a folder on SD
	}
	else
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
	}
}

/***********************************************************************************
 * FileSystem Task entry:  FileSystem_DeleteFolderOrFile
 * Makes a local copy, with task visibility, of the name, including path, then sets
 * the event task event group to start deleting the folder or file
 * Input:	*cFolderName must point to file name and path of the file or folder to
 * 			delete
 * Output:	Local copy of the input parameter get stored locally
 * 			Appropriate task event group bits set to begin deleting a file or folder
 * 			on SD
 * Returned:	None, but error is logged if the path is too large
*************************************************************************************/
void FileSystem_DeleteFolderOrFile(const char *cFolderName)
{
	if (strlen(cFolderName) < sizeof(FileSystem_cPath))
	{
		strcpy(FileSystem_cPath,cFolderName);	// make a local copy of folder for task to use
		xEventGroupSetBits(xFileSystem_EventGroup, FS_DeleteFolderOrFile);	// set task event group to start creating a folder on SD
	}
	else
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
	}
}

/***********************************************************************************
 * FileSystem Task entry:  FileSystem_RenameFolderOrFile
 * Makes a local copy, with task visibility, of the name, including path, then sets
 * the event task event group to start renaming the folder or file
 * Input:	*cFolderName must point to file name and path of the file or folder to
 * 			rename
 * Output:	Local copy of the input parameter get stored locally
 * 			Appropriate task event group bits set to begin renaming a folder on SD
 * Returned:	None, but error is logged if the path is too large
*************************************************************************************/
void FileSystem_RenameFolderOrFile(const char *cFileNames)
{
char *cNewFileName;
size_t stOldFileNameSize;

	// First, split the names up, expected incoming as "old|new"
	cNewFileName = strchr(cFileNames, '|');

	if (cNewFileName != NULL)
	{
		// Next copy the new filename/path
		if (strlen(cNewFileName + 1) < sizeof(FileSystem_cFileNameNew))	// the "+ 1" is to skip past the "|"
		{
			strcpy(FileSystem_cFileNameNew, cNewFileName + 1);	// make a local copy of folder for task to use

			// next copy new filename/path
			stOldFileNameSize = cNewFileName - cFileNames;		// don't need the "+ 1" here because we need to account for the NULL termination char
			if ( stOldFileNameSize < sizeof(FileSystem_cFileName))
			{
				strncpy(FileSystem_cFileName,cFileNames,stOldFileNameSize);		// make a local copy of folder for task to use (subtract out "|"

				xEventGroupSetBits(xFileSystem_EventGroup, FS_RenameFolderOrFile);	// set task event group to start creating a folder on SD
			}
			else // old file name is too long
			{
				BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
			}
		}
		else	// new name is too long
		{
			BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
		}
	}
	else	// couldn't fine the "|"
	{
		BA_ERROR_HANLDER(BA_ERROR_SDCARD_FWRITE, 1);
	}
}

/*
 * Load a flow to memory
 */

void FileSystem_LoadFlow(const char *FileName)
{
	//strncpy(&FlowFileName[0],"/scripts/", 9);
	//strcpy(&FlowFileName[9], FileName);

	strcpy(FlowFileName, FileName);

	xEventGroupSetBits(xFileSystem_EventGroup, FS_LoadFlow);
}

uint8_t FileSystem_IsLoadingAFlow()
{
	EventBits_t uxBits = xEventGroupGetBits(xFileSystem_EventGroup);
	return (uxBits & FS_LoadFlow);
}

void FileSystem_LogError(fs_errorlog_struct_t *ErrorToLog)
{
	if (xQueueSendToBack(xFATFS_ErrorLog_Queue, ErrorToLog, 0) == pdPASS) xEventGroupSetBits(xFileSystem_EventGroup, FS_LogErrorToFile);  //set the flag
	//NOTE if the queue is full, the error will not be logged.
}

static void FileSystem_LogErrorToFile(char *ErrorLogFileName, fs_errorlog_struct_t *ErrorToLog)
{
	 FRESULT fr, fileExists;
	 FILINFO fno;
	 FIL fileobj;

	 fileExists = f_stat(ErrorLogFileName, &fno);

	 // Opens an existing file. If not exist, creates a new file.
	 if (f_open(&fileobj, ErrorLogFileName, FA_WRITE | FA_OPEN_ALWAYS) == FR_OK)
	 	 {
		 	 if (fileExists == FR_NO_FILE)
		 	 	 {
				 //add the header
				 	 if (f_printf(&fileobj, "TimeStamp|ErrorCode|ErrorParam|\r\n") == -1)
				     	{
				     		f_close(&fileobj);
				     		BA_ERROR_HANLDER(BA_ERROR_SDCARD_ELOG, 1);
				     		return;
				     	}
		 	 	 }

		 	 // Seek to end of the file to append data
		 	 fr = f_lseek(&fileobj, f_size(&fileobj));
			 if (fr != FR_OK)
			 	 {
				 	 f_close(&fileobj);
				     BA_ERROR_HANLDER(BA_ERROR_SDCARD_ELOG, 0);
				     return;
			 	 }

			 if (f_printf(&fileobj, "%08d|%03d|%08d|\r\n", ErrorToLog->TimeStamp, ErrorToLog->ErrorNum, ErrorToLog->ErrorParam) == -1) BA_ERROR_HANLDER(BA_ERROR_SDCARD_ELOG, 1 );
			 // Close the file
			 f_close(&fileobj);
		}
	 else BA_ERROR_HANLDER(BA_ERROR_SDCARD_ELOG, 0);
}
