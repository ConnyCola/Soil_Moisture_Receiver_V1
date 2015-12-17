/*-----------------------------------------------------------------------
/  Special SDcard Functions
/-----------------------------------------------------------------------*/


#include <msp430.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
//#include "../config.h"
//#include "../softserial.h"
#include "../pff2a/src/diskio.h"
#include "../pff2a/src/pff.h"
//#include "../print/print.h"
#include "../serial/serial.h"
#include "../drivers/spi.h"


#ifndef _SDCARD
#define _SDCARD




// resINT;
UINT FileIndex;
char FileName[12];

FRESULT res;
WORD s1, s2, ofs, bw;
FATFS fs;			/* File system object */
DIR dir;			/* Directory object */
FILINFO fno;		/* File information */
DWORD tmpFSptr;


void printRES(const void* message, int res);
void printRES2(const void* message,const void* message2, int res);

void resetSDcard(void);
void initSDcard(void);
void writeTAGDataSDcard(ENTRY *e);
void finalizeSDcard();
FRESULT openFileSDcard(const void* Filename);

void findEntryNrSdCard_READ(DWORD nr, ENTRY *e);
void findEntryNrSdCard_WRITE(DWORD nr, ENTRY *e);
void readSDcard(ENTRY *e);
void readINI( u_char *sourceIP,  u_char *gatewayIP,  u_char *subnetMask);
void delay_ms2(u_int time_ms);


#endif //_SDCARD
