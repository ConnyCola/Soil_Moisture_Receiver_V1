/*
 * serial.h
 *
 *  Created on: 14.12.2013
 *      Author: Matthias
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "../typedefs.h"



void initSerial(void);
void xtoa(unsigned long x, const unsigned long *dp);
void puth(unsigned n);
void puts(char *);
void putc(unsigned);
void sendByte(unsigned char byte );
int printf(char *format, ...);
void setToSerial(u_char tst);
void printToString(u_char byte);
void setStringPointer(void* string);


#endif /* SERIAL_H_ */
