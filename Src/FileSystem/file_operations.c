/*
 * file_operations.c
 *
 *  Created on: Feb 7, 2017
 *      Author: Ted
 */

#include "file_operations.h"
#include "ff.h"

/*
configuration file: config.ini



SERN[GB5503135]				//serial number
H0MX[120]					//Heater 0 Max in Celsius
H1MX[120]					//
H2MX[170]					//
H3MX[120]
S1OF[val]					//stepper 1 offset value in steps as int32
S2OF[val]
...
S8OF[val]

T001[val]					//keep this on computer and supply command with correct values?
T002[val]
T003[val]
....
T999[val]


reset flow: reset.flw		//homes everything - same as startup.seq and abort.seq on current software
H0 1 2 3 4*					//turn off all 4 heaters
M0*							//mixer off
F0*							//fans off
A2*							//all actuators down
S4*							//home all stepper motors
//calibrate sensors?


*/

FATFS fileSystem;


/*
 * Mount the SDCARD - needs to be done every time after inserting a card
 */
FRESULT FILEOP_MountCard()
{
	  return f_mount(&fileSystem, SD_Path, 1);
}


/*
 *  Returns the available free space in KiloBytes
 */
uint32_t FILEOP_GetFreeSpace()
{
	   FATFS *fs;
	   uint32_t fre_clust, fre_sect;

       /* Get volume information and free clusters of drive 1 */
	   f_getfree("0:", &fre_clust, &fs);

       /* Get free sectors */
	   fre_sect = fre_clust * fs->csize;

	   /* Print the free space (assuming 512 bytes/sector) */
	   return (fre_sect / 2);
}

/*
 * Returns the total space in KiloBytes
 */
uint32_t FILEOP_GetTotalSpace()
{
	   FATFS *fs;
	   uint32_t fre_clust, tot_sect;

       /* Get volume information and free clusters of drive 1 */
	   f_getfree("0:", &fre_clust, &fs);

       /* Get total sectors and free sectors */
	   tot_sect = (fs->n_fatent - 2) * fs->csize;

	   /* Print the free space (assuming 512 bytes/sector) */
	   return (tot_sect / 2);
}

/*------------------------------------------------------------/
/ Open or create a file in append mode
/ (This function was superseded by FA_OPEN_APPEND at FatFs R0.12a)
/<FileToOpen> Path + file name to open
/------------------------------------------------------------*/
FRESULT FILEOP_OpenAndAppend(const char* FileToOpen, const char *string1)
{
	FRESULT fr;
    FIL fileobj;

    // Opens an existing file. If not exist, creates a new file.
    fr = f_open(&fileobj, FileToOpen, FA_WRITE | FA_OPEN_ALWAYS);

    // Seek to end of the file to append data
    if (fr == FR_OK)
    	{
        fr = f_lseek(&fileobj, f_size(&fileobj));
        if (fr != FR_OK) f_close(&fileobj);
    	}

    fr = f_printf(&fileobj, string1);

    // Close the file
    fr = f_close(&fileobj);

    return fr;
}

FRESULT FILEOP_ReadLineByLine()
{
	FRESULT fr;
	FIL fileobj;
	char line[256]; /* Line buffer */

	/* Open a text file */
	fr = f_open(&fileobj, "0:flow.txt", FA_READ);
	if (fr) return fr;

	/* Read all lines and display it */
	while (f_gets(line, sizeof line, &fileobj));
		//USB_SendFromTask(100, "%s",line);

	/* Close the file */
	fr = f_close(&fileobj);

	return fr;
}


