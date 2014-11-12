/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/

#define BIT0 0x1
#define BIT1 0x2
#define BIT2 0x4
#define BIT3 0x8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define Led      BIT2
#define Led_Port PORTD
#define Led_Dir  DDRD


#define LED_ON  Led_Port &= ~Led;
#define LED_OFF Led_Port |= Led;


#define bit(param) (1<<(param))




/**
*@brief Calcule the CRC for a given array with one or more bytes
*@var data Poiter to the array of data
*@var length Length of the array of data in number of bytes
*@var prevCRC CRC from where to start calculating the rest 
*@return calculated CRC
*/
unsigned char crc(unsigned char *data, unsigned char length, unsigned char prevCRC);



/**
*@brief Simple data copier
*@var destine Poiter to the array of data where to copy data
*@var destine Poiter to the array of data to copy from
*@var length Length of the array of data in number of bytes
*@return none
*/
void memcopy(unsigned char *destine,unsigned char *origin, unsigned int length);
