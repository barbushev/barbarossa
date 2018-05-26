/*
 * file_operations.h
 *
 *  Created on: Feb 7, 2017
 *      Author: Ted
 */

#ifndef FATFS_FILE_OPERATIONS_H_
#define FATFS_FILE_OPERATIONS_H_

#include "stdint.h"  //needed for uint32_T
//#include "stdarg.h"  //for vargs
#include "fatfs.h"


FRESULT FILEOP_MountCard();
uint32_t FILEOP_GetFreeSpace();
uint32_t FILEOP_GetTotalSpace();

FRESULT FILEOP_OpenAndAppend(const char* FileToOpen, const char *string1);
FRESULT FILEOP_ReadLineByLine();


#endif /* FATFS_FILE_OPERATIONS_H_ */
