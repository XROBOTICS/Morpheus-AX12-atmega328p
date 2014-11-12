/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/
#include "UtilsAndDefines.h"
#include <avr/io.h>
#include <ctype.h>

#define DI  0x07

 
uint8_t crc(uint8_t *data, uint8_t length, uint8_t prevCRC)
{
	uint8_t counter=0;
	uint8_t crc = prevCRC;	

	for(counter=0;counter<length;counter++)
	{
		crc += ~(*data++);
	}
	return crc;
}






void memcopy(unsigned char *destine, unsigned char *origin, unsigned int length)
{
	while(length--)
	*destine++=*origin++;
}



