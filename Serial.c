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
#include <avr/interrupt.h>
#include <ctype.h>
#include "UtilsAndDefines.h"
#include "Serial.h"
#include "Eprom.h"




// message composition
// header  length destine devID  Origin_ID  data1   data2         dataN   CRC
// [0xAA]  [0xnn]      [id]       [MY_ID]  [0xd1]  [0xd1]  [...] [0xdn]  [0xzz]
//                \____________________ length _______________________/
//         \________________________ crc _____________________________/




uint8_t DataOut[bufferSize+1];
uint8_t DataOutEndPointer=0;
uint8_t DataOutStartPointer=0;
uint8_t DataOutDiffPointer=0;

uint8_t DataIn[2][bufferSize2+1];
uint8_t DataInLen[2]={0,0};
uint8_t DataInBufSelec = 0;


extern uint16_t SerialPWM =0;


void SendMSG(uint8_t *Array, uint8_t length,uint8_t DestinationID)
{
	uint8_t temp = 0xAA; // header
	uint8_t calcCRC=0;	

	SendData(&temp,1);

	temp = (length+2); // data length plus 2 that is the ID + MY_ID length
	calcCRC= crc(&temp, 1, 0); // CRC
	SendData(&temp,1);

	temp = DestinationID;
	calcCRC += crc(&temp, 1, 0); // CRC
	SendData(&temp,1); // ID

	temp = MY_ID;
	calcCRC += crc(&temp, 1, 0); // CRC
	SendData(&temp,1); // MY ID

	SendData(Array,length); //data
	calcCRC +=crc(Array, length, 0); // CRC

	SendData(&calcCRC,1);
}



int8_t GetMSG(uint8_t *Array,uint8_t Start,uint8_t Length, uint8_t free)
{
	static uint8_t selector=0;
	static uint8_t length=0;
	static uint8_t usingMessage=0;
	
	if(usingMessage==0)
	{
		cli();
		selector=(1-DataInBufSelec);
		if((DataInLen[selector] != 0) && (Length>0))
		{
			if((Start+Length) > DataInLen[selector])
			{
				free=1;
			}
			else
			{
				usingMessage=1;
				length = DataInLen[selector];			
				DataInLen[selector]=0xFF;  /// place 0xFF on the data length so on getChar() no overrite to any data is done,
											   /// forcing  to rewrite to the same buffer again once this is being used
				sei();
				memcopy(Array,(DataIn[selector]+Start),Length);
			}
		}
		sei();
	}
	else
	{
		if(Length>0)
		{
			if((Start+Length) > length)
			{
				free=1;
			}
			else
			memcopy(Array,(DataIn[selector]+Start),Length);
		}

		if(free==1) //once free is set, reset the buffer to be avaliable
		{
			length=DataInLen[selector]=0x0;
			usingMessage=0;
		}
	}
	return length;
}




uint16_t SendData(uint8_t *Array, uint8_t length)
{
	uint8_t counter=0;

	for(counter=0;counter<length;counter++)
	{
		DataOut[DataOutEndPointer++]=*Array++;
		
		if(DataOutEndPointer==bufferSize)
		{
			DataOutEndPointer=0;
		}

		if(DataOutDiffPointer<bufferSize)
		{
			DataOutDiffPointer++;
		}
		else
		{
			length=0;
		}
	}

	return counter;
}





void InitSerial(uint16_t SerialSpeed)
{
	HalfDuplexTransceiver_Dir  |=	(HalfDuplexTransceiver_PinOUT|HalfDuplexTransceiver_PinIN);
	
	UCSRA = bit(U2X); //using double speed
	UCSRB = (bit(RXCIE)|/*bit(UDRIE)|*/bit(RXEN)|bit(TXEN)/*|bit(TXCIE)*/); // enable RX and TX interrupt RX and TX

	UCSRB &= ~(bit(RXB8)|bit(TXB8)|bit(UCSZ2));   // disable  TX and RX nineth bit and reset UCSZ2, this is used with UCSZ1 and UCSZ0 to determine the length of the word
	UCSRC = ~(bit(UMSEL)|bit(UPM0)|bit(UPM1)|bit(USBS)|bit(UCPOL));//disable parity and set assyncronous, set 1stopBit and polarity on rising edge
	UCSRC |= (bit(UCSZ1)|bit(UCSZ0)|bit(UCPOL));
	//   UCSZ2=0 UCSZ1=1 UCSZ0=1 => 8bit length
	UBRRH = ((SerialSpeed)>>8);   //set baud counter high
	UBRRL = (SerialSpeed&0xFF); 			//set baud counter low																															
	UCSRA|=bit(TXC);
}





void PrintDecimal (int Data)
{

	volatile uint16_t counter=5, firstNum=0, result=0, temp=0;
	volatile uint16_t divisor=10000;

	if(Data>0)
	{
		while(counter--)
		{
			result=Data/divisor;
			if( (result>0) || (firstNum==1))
			{
				firstNum=1;
				result+=0x30;
				SendData((uint8_t *)&result,1);
				//while(!SendChar());
				result-=0x30;
			}
			Data-=(result*divisor);
			divisor=(divisor/10);
		}
	}
	else
	if(Data<0)
	{
		temp=(Data-0xFFFE);
		while(counter--)
		{
			result=temp/divisor;
			if( (result>0) || (firstNum>0))
			{
				firstNum++;
				result+=0x30;
				if(firstNum==1)
				{
					SendData("-",1);
					//while(!SendChar());
				}

				SendData((uint8_t *)&result,1);
				//while(!SendChar());
				result-=0x30;
			}
			Data-=(result*divisor);
			divisor=(divisor/10);
		}
	}
	else
	{
		result=0x30;
		SendData((uint8_t*)&result,1);
	}

}


#define resetTim_Default bufferSize


uint8_t GetChar(void)
{
	uint8_t incomeChar=0;
	static uint8_t msgLEN=0;
	static uint8_t msgLen=0;
	static uint8_t msgCrc=0;
	static uint8_t select=0;
	static uint8_t syncWrLen=0;
	static uint8_t syncWrLength=0;
	static uint8_t syncWrCount=0;
	static uint16_t resetTim=resetTim_Default;

	if(UCSRA&bit(RXC))
	{

		if(resetTim==0)
		{
			select = 0;
			msgCrc = 0;
			msgLEN = 0;
			msgLen = 0;
			syncWrLength=0;
			syncWrLen=0;
			syncWrCount=0;
			DataInLen[0]=0;
			DataInLen[1]=0;
			DataInBufSelec=0;
			resetTim=resetTim_Default;
		}
		else
			resetTim--;

		incomeChar=UDR;	
		
		switch(select)
		{
			case 0:
			
				if(incomeChar==0xAA) // header found
				{
					//LED_ON;
					if( DataInLen[DataInBufSelec] == 0xFF ) // this is just to prevent write to the buffer being readed, on this case, reselect the same buffer to write the new data
					{
						DataInBufSelec = (1-DataInBufSelec);
					}
					
					DataInLen[DataInBufSelec]=0; // this done here to signal that is beeing used by the reception
					select = 1;	
					//SendData("1",1);
					resetTim=resetTim_Default;
				}
				break;
					
			case 1:
				msgLen = 0;
				if(incomeChar>bufferSize)
				{
					select=0;
				}
				else
				{
					msgLEN = incomeChar; // Len getting
					msgCrc=(~incomeChar);
					select = 2;		
				}	
				break;
			
			case 2:
				
				DataIn[DataInBufSelec][msgLen++]=incomeChar;
				// else the CRC will get error so we can discard the frame that is bigger than the buffer

				msgCrc+=(~incomeChar);
				if(msgLen==3)
				{
					if(incomeChar==AX12_SYNC_WR)
						select = 4; // syncwrite packet incomming
					else
						select = 3; // normal data incomming packet
				}
				
				break;
				
			case 3:

				DataIn[DataInBufSelec][msgLen]=incomeChar;
				// else the CRC will get error so we can discard the frame that is bigger than the buffer
				
				
				if(msgLen++ == msgLEN)
				{
					
					if(msgCrc==incomeChar)
					{
						DataInLen[DataInBufSelec]= msgLEN;
						DataInBufSelec = (1-DataInBufSelec);
					}
					select = 0;
					msgCrc = 0;
					msgLEN = 0;
					msgLen = 0;
					//LED_OFF
				}
				else
					msgCrc+=(~incomeChar);
				//SendData("3",1);
				break;
				
			case 4:
				// get address
				DataIn[DataInBufSelec][msgLen]=incomeChar;
				
				msgCrc+=(~incomeChar);

				msgLen++;

				select=5;
				break;
				
			case 5:
				// get length
				DataIn[DataInBufSelec][msgLen]=incomeChar;
				syncWrLen=incomeChar;
				msgLen++;
				msgCrc+=(~incomeChar);
				syncWrCount=(syncWrLen+1);
				select=6;
				syncWrLength=msgLen;
				break;
				
			case 6:
				// look for My ID or end of packet

				if(syncWrCount==(syncWrLen+1))
				{
					syncWrCount=0;
					if(msgLen == msgLEN)
					{
						select = 0;
						msgCrc = 0;
						msgLEN = 0;
						msgLen = 0;
					}
					else
					if(incomeChar == MY_ID)
					{
						DataIn[DataInBufSelec][syncWrLength++]=incomeChar;
						select = 7;
					}
				}
				syncWrCount++;
				msgCrc+=(~incomeChar);
				msgLen++;
				break;
				
			case 7:

				DataIn[DataInBufSelec][syncWrLength++]=incomeChar;
				msgCrc+=(~incomeChar);
				msgLen++;
				
				if(syncWrCount++ == syncWrLen )
				{

					if( DataIn[DataInBufSelec][syncWrLength-5] == 0x10) // adress for PWM
					{
						SerialPWM=(DataIn[DataInBufSelec][syncWrLength-2]+(DataIn[DataInBufSelec][syncWrLength-1]<<8));
					}

					select = 8;
				}
				break;
				
			case 8:

				if(msgLen == msgLEN)
				{	
					if(msgCrc==incomeChar)
					{
						if(DataIn[DataInBufSelec][syncWrLength-5] != 0x10) // adress of PWM
						{
							DataInLen[DataInBufSelec]= (syncWrLen+6);
							DataInBufSelec = (1-DataInBufSelec);
						}
					}

					select = 0;
					msgCrc = 0;
					msgLEN = 0;
					msgLen = 0;
					syncWrLength=0;
				}
				else
				{
					msgCrc+=(~incomeChar);
					msgLen++;
				}
		
				
				break;
				
		}
		return 1;
	}
	return 0;
}

int8_t SendChar(void)
{
	static uint8_t readWrite=0; // this exists only to spare some time on the continuous polling
	
	if(UCSRA&bit(UDRE))
	{
	
		if(DataOutDiffPointer>0)
		{
			if(readWrite==0)
			{
				readWrite=1;
				SET_WRITE;
			}
			UDR=DataOut[DataOutStartPointer];	

			DataOutStartPointer++;
			DataOutDiffPointer--;

			if(DataOutStartPointer==bufferSize)
				DataOutStartPointer=0;

			return 1;
		}
	}

	if(DataOutDiffPointer==0)
	{
		if(UCSRA&bit(TXC))
		{
			readWrite=70;
			while(readWrite--){asm("nop");} // this is just to allow the  sending of the last byte complitly before disconnect the reading
			readWrite=0;
			SET_READ;
			UCSRA|=bit(TXC);
		}
		else
		return 0;// mean that ther's nothing to be sent
	}
	else  // mean that uart hard buff is full and no byte could be sent there
	{
		return -1;
	}
}

#if defined __AVR_ATmega8__
ISR (USART_RXC_vect)
#elif defined __AVR_ATmega328P__
ISR (USART_RX_vect)
#endif
{
	
	GetChar();
	
}
