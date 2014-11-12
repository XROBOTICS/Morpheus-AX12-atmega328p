/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************

Changes
V1.0 to V1.2
This version bugs fix:
 - Serial buff reset on test to Message Length keeper and not to buff Length 
 
Improves:
 - Led does not fade, short fast blinks.
 - Fail safe counter created to avoid break on cycling for position,
   when missing previouse messages from previouse servos on chain.
 - Controlling servos just by PWM and cycling for position, the frame, now possible at least at 600fps
 - Fail safe created for speed up syncronization on frame receiving scrambling.

*************************************************/
#include <util/delay.h>
#include <avr/io.h>
//#if defined __AVR_ATmega8__
//#include <avr/iom8.h>
//#elif defined __AVR_ATmega328P__
//#include <avr/iom328p.h>
//#endif
#include <avr/interrupt.h>
#include <ctype.h>

#include "Motor.h"
#include "UtilsAndDefines.h"
#include "Serial.h"
#include "Control.h"
#include "Eprom.h"





#define INIT_DELAY 10 //ms


#define RAMVarSize (EEVarSize+0x0007)
#define GoalPositionLow_Addr	(EEVarSize+0)
#define GoalPositionHi_Addr 	(EEVarSize+1)
#define GoalSpeed_Addr      	(EEVarSize+2)
#define TemperatureLow_Addr 	(EEVarSize+3)
#define TemperatureHi_Addr		(EEVarSize+4)
#define PWMLow_Addr     		(EEVarSize+5)
#define PWMHi_Addr     			(EEVarSize+6)


uint16_t GoalPos=0;
uint16_t GoalSpeed=0;
uint16_t PreviouseGoalPos=0;
uint16_t GoalPWM=0;
uint16_t PreviouseGoalPWM=0;

uint16_t DelayMs=0;


void RAMSave (uint8_t address, uint8_t data)
{
	uint16_t temp=data;
	// if you need to have any variables on ram that can be dynamicly configured
	switch (address)
	{
		case GoalPositionLow_Addr:
			GoalPos = ((GoalPos&0xFF00)+temp);
		break;

		case GoalPositionHi_Addr:
			GoalPos = ((GoalPos&0xFF)+(temp<<8));
		break;

		case GoalSpeed_Addr:
			GoalSpeed = data;
		break;
			
		case PWMLow_Addr:
			GoalPWM=((GoalPWM&0xFF00)+temp);
			PreviouseGoalPWM=GoalPWM+1;
		break;
		
		case PWMHi_Addr:
			GoalPWM=((GoalPWM&0xFF)+(temp<<8));
			PreviouseGoalPWM=GoalPWM+1;
		break;
	}
}

uint8_t RAMGet (uint8_t address)
{
	// if you need to have any variables on ram that can be dynamicly configured
	uint16_t temp=0;

	switch (address)
	{
		case GoalPositionLow_Addr:
			temp=Position;
			//LED_ON;
			return (temp&0xFF);
		break;

		case GoalPositionHi_Addr:
			temp=Position;
			return ((temp&0xFF00)>>8);
		break;

		case GoalSpeed_Addr:
			temp=Velocity;
		    return (temp&0xFF);
		break;

		case TemperatureLow_Addr:
			temp=Temperature;
		    return (temp&0xFF);
		break;

		case TemperatureHi_Addr:
			temp=Temperature;
		    return ((temp&0xFF00)>>8);
		break;
		
		case PWMLow_Addr:
			temp=(GoalPWM&0xFF);
		break;
		
		case PWMHi_Addr:
			temp =((GoalPWM&0xFF)>>8);
		break;
	}
	return temp;
}



#define serviceRoutineReset 6
#define CycleFailSafeReplyDelay 2

int main (void)
{

	uint8_t u8Data[50];
	uint8_t u8BusInitiated=0;
	uint8_t u8ID_Previouse=0;
	uint8_t u8Master_Id=0;
	uint8_t u8SendCycle=0;
	int16_t i16Temp=0;
	uint8_t u8ChangedRom=0;
	uint8_t serviceRoutineCounter=0;
	uint8_t sreg;
	uint8_t u8PreviousCount=0;
	uint16_t LedCounter=0;
	uint16_t LedCounterPWM=3000;
	uint16_t u8Counter=0;

	#if defined __AVR_ATmega8__
	WDTCR=0;
	#elif defined __AVR_ATmega328P__
	WDTCSR=0;
	#endif

	Led_Dir |= Led;
	LED_OFF;

	EELoadVars();  // load default variables from EEProm
	InitMotor();
	InitControl();
	//InitSerial((uint16_t)UART_BAUD);
	InitSerial(UART_BAUD);

	sei();


// message composition
// header  length destine devID  Origin_ID  data1   data2         dataN   CRC
// [0xAA]  [0xnn]      [id]       [MY_ID]  [0xd1]  [0xd1]  [...] [0xdn]  [0xzz]
//                \____________________ length _______________________/
//         \________________________ crc _____________________________/
// Length is the data that we can read from the buffer once get msg just assure the message integrity by length and CRC nothing else.

	SetFreeweeling(1000);
	while(1) //initialization
	{
		
		if(LedCounter==0)//1ms
			LED_ON;	
		if(LedCounter==LedCounterPWM)
			LED_OFF;
		LedCounter++;


		if(serviceRoutineCounter==0)
		{
			if(SerialPWM != PreviouseGoalPWM)
			{
				RAMSave(PWMLow_Addr,(SerialPWM&0xFF)); // save data to RAM
				RAMSave(PWMHi_Addr,(SerialPWM>>8)); // save data to RAM
				SetFreeweeling(GoalPWM);
				PreviouseGoalPWM=GoalPWM;
			}

			//TimerInterruptServiceRoutine();
			_Control_();	

			if(DelayMs)
				DelayMs--;	

			//TimerIntON=0;
			serviceRoutineCounter=serviceRoutineReset;
		}		
		else
			serviceRoutineCounter--;

		SendChar();	
		i16Temp = GetMSG(u8Data,0,3,0);
//#########################################     START COMMAND INTERPRETER   ###################################################################
		if(i16Temp>0)// data avaliable
		{
			if(u8Data[0]==BROADCAST_ID)//destination
			{

			
				if(u8Data[2]==AX12_START)   /// start or restart the bus
				{
					u8Master_Id = u8Data[1];
					u8BusInitiated=1;
					u8ID_Previouse=0;
					DelayMs=(MY_ID*INIT_DELAY*10);
				}
				else
				if(u8Data[2]==AX12_WRITE)  /// write
				{
				
					i16Temp-=4; // take out the length of destineID + originID + Command + address_of_writing
					GetMSG(&u8Data[1],3,1,0); //get the address
					u8Data[2]=4;
					while(i16Temp)
					{
						GetMSG(&u8Data[0],u8Data[2]++,1,0); //get the data
						if(u8Data[1]<EEVarSize) // test if address is from eeprom
						{
							u8ChangedRom=1;
						//	sreg=SREG;
							cli();
							EESave(u8Data[1]++,u8Data[0]); // save data to eeprom
					//		SREG=sreg;
							sei();
						}
						else
						if(u8Data[1]<RAMVarSize) // test if address if from RAM
						{
							RAMSave(u8Data[1]++,u8Data[0]); // save data to RAM
						}
						i16Temp--;
					}

					if(GoalPos!=PreviouseGoalPos)
					{
						PreviouseGoalPos=GoalPos;
						SetPosition (GoalPos,GoalSpeed, 100, 0);
						SerialPWM=GoalPWM=PreviouseGoalPWM=1000;
					}
					
					if(GoalPWM!=PreviouseGoalPWM)
					{
						SetFreeweeling(GoalPWM);
						SerialPWM=PreviouseGoalPWM=GoalPWM;
						PreviouseGoalPos=0;
						GoalPos=0;
					}
					
					if(u8ChangedRom)
					{
					//	sreg=SREG;
						cli();
						EELoadVars();
					//	SREG=sreg;
						sei();
						InitSerial(UART_BAUD);
						u8ChangedRom=0;
					}
					//no reply to Master once is a broadcast
				

				}
				else
				if(u8Data[2]==AX12_SYNC_WR)   /// sync write
				{
					// do nothing
					
					// temp // total packet length
					GetMSG(u8Data,3,2,0);
					//data[0];  // is the adress where to start writing
					//data[1];  // is the amount of data to write
					
					GetMSG(&u8Data[2],6,u8Data[1],1); // read to *data[2] starting on 5 a length of data[1]+1*(ID)
					//write data to memory eeprom or ram and end
					for(u8Counter=0;u8Counter<u8Data[1];u8Counter++)
					{
						if(u8Data[0]<EEVarSize) // test if address is from eeprom
						{
							u8ChangedRom=1;	
						//	sreg=SREG;					
							cli();
							EESave(u8Data[0]++,u8Data[2+u8Counter]); // save data to eeprom
						//	SREG=sreg;
							sei();
						}
						else
						if(u8Data[0]<RAMVarSize) // test if address if from RAM
						{
							RAMSave(u8Data[0]++,u8Data[2+u8Counter]); // save data to RAM
						}
					}

					if(GoalPos!=PreviouseGoalPos)
					{
						PreviouseGoalPos=GoalPos;
						SetPosition (GoalPos,GoalSpeed, 100, 0);
						SerialPWM=GoalPWM=PreviouseGoalPWM=1000;
					}
					
					if(GoalPWM!=PreviouseGoalPWM)
					{
						SetFreeweeling(GoalPWM);
						SerialPWM=PreviouseGoalPWM=GoalPWM;
						PreviouseGoalPos=0;
						GoalPos=0;
					}
					
					if(u8ChangedRom)
					{
					//	sreg=SREG;
						cli();
						EELoadVars();
					//	SREG=sreg;
						sei();
						InitSerial(UART_BAUD);
						u8ChangedRom=0;
					}
				}
				else
				if(u8Data[2]==AX12_CYCLE)    /// start the sending cycle
				{
					if(u8BusInitiated==2) // if the BUS is initiated
					{
						u8Master_Id = u8Data[1];
						if(u8ID_Previouse==0) // means  that there is no servos before me i'm the first 
						{
							u8Data[0] = ((Position)>>8); //MSB
							u8Data[1] = Position; //LSB
							SendMSG(u8Data,2,u8Master_Id);
	
							u8SendCycle=0;
						}
						else
						{
							u8SendCycle=1;
							DelayMs=(CycleFailSafeReplyDelay*UART_BAUD*u8PreviousCount);
						}
					}
					//else do nothing
				}
				else
				if(u8Data[2]==AX12_RESET)
				{
				//	sreg=SREG;
					cli();
					EEPromReset();
				//	SREG=sreg;
					sei();
					InitSerial(UART_BAUD);		
				}
				else
				{
					u8Master_Id = u8Data[1];
					// no recognised command
					u8Data[0] = AX12_NOK;  // send NOK
					SendMSG(u8Data,1,u8Master_Id);
				}
			}
			else
			if(u8Data[0]==MY_ID)   //destination is Me
			{


				u8Master_Id = u8Data[1];
				if(u8Data[2]==AX12_WRITE)  /// write
				{
					i16Temp-=4; // take out the length of destineID + originID + Command + address_of_writing
					GetMSG(&u8Data[1],3,1,0); //get the address

					if((i16Temp+u8Data[1])>(RAMVarSize)) // if address+length > RAMVarSize+EEVarSize   send NOK
					{
						// no recognised command
						u8Data[0] = AX12_NOK;  // send NOK
						SendMSG(u8Data,1,u8Master_Id);
					}
					else
					{
						u8Data[2]=4;

						while(i16Temp)
						{
							GetMSG(&u8Data[0],u8Data[2]++,1,0); //get the data

							if(u8Data[1]<EEVarSize) // test if address is from eeprom
							{
								u8ChangedRom=1;
							//	sreg=SREG;
								cli();
								EESave(u8Data[1]++,u8Data[0]); // save data to eeprom
							//	SREG=sreg;
								sei();
							}
							else
							if(u8Data[1]<RAMVarSize) // test if address if from RAM
							{
								RAMSave(u8Data[1]++,u8Data[0]); // save data to RAM
							}
							i16Temp--;
						}
						
						if(GoalPos!=PreviouseGoalPos)
						{
							PreviouseGoalPos=GoalPos;
							SetPosition (GoalPos, GoalSpeed, 100, 0);
							SerialPWM=GoalPWM=PreviouseGoalPWM=1000;
						}
						
						if(GoalPWM!=PreviouseGoalPWM)
						{
							
							SetFreeweeling(GoalPWM);
							SerialPWM=PreviouseGoalPWM=GoalPWM;
							PreviouseGoalPos=0;
							GoalPos=0;
						}

						// Load the variables from EEPROM
						
						if(u8ChangedRom)
						{
						//	sreg=SREG;
							cli();
							EELoadVars();
						//	SREG=sreg;
							sei();

							InitSerial(UART_BAUD);
							u8ChangedRom=0;
						}
						
						u8Data[0] = AX12_OK;  // send OK
					    SendMSG(u8Data,1,u8Master_Id);
					}

				}
				else
				if(u8Data[2]==AX12_READ)
				{
					GetMSG(&u8Data[2],3,2,0); //get the address and length

     				if((u8Data[2]+u8Data[3])>(RAMVarSize)) // if address+length > RAMVarSize+EEVarSize   send NOK
					{
						// no recognised command
						u8Data[0] = AX12_NOK;  // send NOK
						SendMSG(u8Data,1,u8Master_Id);
					}
					else
					{
						
						for(i16Temp=0; i16Temp<u8Data[3] ; i16Temp++)
						{
					
							if(u8Data[2]<EEVarSize) // test if address is from eeprom
							{
							//	sreg=SREG;
								cli();
								u8Data[4+i16Temp] = EEGet(u8Data[2]+i16Temp); // get data from eeprom
								//SREG=sreg;
								sei();
								
							}
							else
							if(u8Data[2]<RAMVarSize) // test if address if from RAM
							{
								u8Data[4+i16Temp] = RAMGet(u8Data[2]+i16Temp); // get data from RAM
							}
						}
						

						SendMSG(&u8Data[4],i16Temp,u8Master_Id); // send data to the requester
					}
				
				}
				else
				if(u8Data[2]==AX12_PING) 
				{
					u8Data[0] = AX12_PING; 
					SendMSG(u8Data,1,u8Master_Id);
				}
				else
				if(u8Data[2]==AX12_RESET)
				{
					//sreg=SREG;
					cli();
					EEPromReset();
					//SREG=sreg;
					sei();
					InitSerial(UART_BAUD);		
					u8Data[0] = AX12_PING; 
					SendMSG(u8Data,1,u8Master_Id);
				}
				else
				{
					// no recognised command
					u8Data[0] = AX12_NOK;  // send NOK
					SendMSG(u8Data,1,u8Master_Id);
				}
				
			}
		    else
			if(u8Data[1]==u8ID_Previouse) // previouse device sent packet its my turn now
			{
				if(u8BusInitiated==2)
				{
					if(u8SendCycle==1)
					{
						u8SendCycle=0;
						
						u8Data[0] = ((Position)>>8); //MSB
						u8Data[1] = Position; //LSB
						SendMSG(u8Data,2,u8Master_Id);
					}
				}
			}
			else
			{	// in here we can receive data from other servos


				if(u8BusInitiated==1) // get the previouse ID to know the place in the line
				{
					u8ID_Previouse = u8Data[1];
					u8PreviousCount++;
				}
			}

			// free buffer
		    GetMSG(0,0,0,1);

		}
//#########################################     END COMMAND INTERPRETER   ###################################################################
		
		if(u8BusInitiated==1)
		{
			if(DelayMs==0) // our turn to send the data packet for ping of initiation
			{
				
				u8Data[0] = AX12_PING;
				SendMSG(u8Data,1,u8Master_Id);
				u8BusInitiated=2;
			}
		}
		else if((u8BusInitiated==2)&&(u8SendCycle==1))
		{
			if(DelayMs==0)
			{ // this is a fail safe, in case of missing the pack from the previouse servo, does not break the reply chain to the master
				u8SendCycle=0; 
				u8Data[0] = ((Position)>>8); //MSB
				u8Data[1] = Position; //LSB
				SendMSG(u8Data,2,u8Master_Id);
			}
		}
	}

	



}











