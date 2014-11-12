/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include "Motor.h"
#include "UtilsAndDefines.h"
#include "Serial.h"
#include "Control.h"
#include "Eprom.h"


extern unsigned char MY_ID;
extern unsigned char CTL_UP_LIMIT;
extern unsigned char CTL_LW_LIMIT;
extern unsigned char KPp_PID;
extern unsigned char KIp_PID;
extern unsigned char KDp_PID;
extern unsigned char KPv_PID;
extern unsigned char KIv_PID;
extern unsigned char KDv_PID;
extern unsigned char UART_BAUD;



/*
#define Rsv_Dflt            0x0000
#define MY_ID_Addr 			0x0001
#define CTL_UP_LIMIT_Addr 	0x0002
#define CTL_LW_LIMIT_Addr	0x0002
#define KPp_PID_Addr		0x0030
#define KIp_PID_Addr		0x0005
#define KDp_PID_Addr		0x0001
#define KPv_PID_Addr		0x0030
#define KIv_PID_Addr		0x0005
#define KDv_PID_Addr		0x0001
#define UART_BAUD_Addr		0x0022 // 57600 baud
*/

uint8_t EESave(uint16_t Variable, uint8_t data)
{

	if(Variable==Rsv_Dflt) // no writing allowed to the Reserved space
		return 0;
	else
	if(Variable==MY_ID_Addr) // no ID should be 0 or 255(brroadcast)
	{
		if((data==0) || (data==255))
			return 0;
	}
	else
	if(Variable==UART_BAUD_Addr) // only the supported bauds shall be accepted
	{
		if((data!=Baud9600)&&(data!=Baud57600)&&(data!=Baud115200)&&(data!=Baud500000)&&(data!=Baud1000000)&&(data!=Baud2000000))
			return 0;
	}

	eeprom_write_byte((uint8_t*)Variable,data);

	return 1;

}



uint8_t EEWrite(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t temp = 0;
	while(temp<length)
	{
		if(EESave((address+temp),data[temp])!=1)
			return 0;
		temp++;
	}

	return 1;
}


void EERead(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t temp = 0;
	while(temp<length)
	{
		data[temp]=eeprom_read_byte((uint8_t*)address+temp);
		temp++;
	}

}

uint8_t EEGet(uint16_t Variable)
{
	uint8_t data=0;

	data=eeprom_read_byte((uint8_t*)Variable);

	return data;
}


void EEPromReset(void)
{
	eeprom_write_byte(Rsv_Addr,0); // set the eeprom with valid results
	EELoadVars();
}



void EELoadVars(void)
{
	

	if(EEGet(Rsv_Addr)!=Rsv_Dflt)  // it this is diferent then Rsv_Dflt then the eeprom has no valid results
	{							   // then load the defaults!
		EESave(MY_ID_Addr,MY_ID_Dflt);
		EESave(CTL_UP_LIMIT_Addr,CTL_UP_LIMIT_Dflt);
		EESave(CTL_LW_LIMIT_Addr,CTL_LW_LIMIT_Dflt);
		EESave(KPp_PID_Addr,KPp_PID_Dflt);
		EESave(KIp_PID_Addr,KIp_PID_Dflt);
		EESave(KDp_PID_Addr,KDp_PID_Dflt);
		EESave(KPv_PID_Addr,KPv_PID_Dflt);
		EESave(KIv_PID_Addr,KIv_PID_Dflt);
		EESave(KDv_PID_Addr,KDv_PID_Dflt);
		EESave(UART_BAUD_Addr,UART_BAUD_Dflt);
		eeprom_write_byte(Rsv_Addr,Rsv_Dflt); // set the eeprom with valid results
	}

	MY_ID=EEGet(MY_ID_Addr);
	CTL_UP_LIMIT=EEGet(CTL_UP_LIMIT_Addr);
	CTL_LW_LIMIT=EEGet(CTL_LW_LIMIT_Addr);

	KPp_PID=EEGet(KPp_PID_Addr);
	KIp_PID=EEGet(KIp_PID_Addr);
	KDp_PID=EEGet(KDp_PID_Addr);

	KPv_PID=EEGet(KPv_PID_Addr);
	KIv_PID=EEGet(KIv_PID_Addr);
	KDv_PID=EEGet(KDv_PID_Addr);

	UART_BAUD=EEGet(UART_BAUD_Addr);
}
