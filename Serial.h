/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/

#if defined __AVR_ATmega328P__
#define UCSRA			UCSR0A
#define UCSRB			UCSR0B
#define UCSRC			UCSR0C
#define U2X				U2X0
#define RXCIE			RXCIE0
#define RXEN			RXEN0
#define TXEN			TXEN0
#define RXB8			RXB80
#define TXB8			TXB80
#define UCSZ0			UCSZ00
#define UCSZ1			UCSZ01
#define UCSZ2			UCSZ02
#define UMSEL			UMSEL00
#define UPM0			UPM00
#define UPM1			UPM01
#define USBS			USBS0
#define UCPOL			UCPOL0
#define UBRRH			UBRR0H
#define UBRRL			UBRR0L
#define TXC				TXC0
#define RXC				RXC0
#define UDR				UDR0
#define UDRE			UDRE0
#endif

#define AX12_PING		0x01
#define AX12_READ		0x02
#define AX12_WRITE		0x03
#define AX12_OK         0x04
#define AX12_MUTE       0x05
#define AX12_START      0x06
#define AX12_CYCLE      0x07
#define AX12_NOK        0x08
#define AX12_RESET		0x09
#define AX12_SYNC_WR	0x83

#define MASTER_ID		254
#define BROADCAST_ID    255

#define HalfDuplexTransceiver_Port		PORTD
#define HalfDuplexTransceiver_Dir		DDRD
#define HalfDuplexTransceiver_PinOUT	BIT7
#define HalfDuplexTransceiver_PinIN		BIT6

#define Baud9600    207
#define Baud57600   34
#define Baud115200  16
#define Baud500000  3
#define Baud1000000 1
#define Baud2000000 0

#define bufferSize 200
#define bufferSize2 250

#define TXByteEmpty (((UCSRA&bit(UDRE))>0)?1:0)
#define TXBuffEmpty (((UCSRA&bit(TXC))>0)?1:0)

#define SET_WRITE HalfDuplexTransceiver_Port|=HalfDuplexTransceiver_PinOUT;HalfDuplexTransceiver_Port&=~HalfDuplexTransceiver_PinIN;

#define SET_READ HalfDuplexTransceiver_Port&=~HalfDuplexTransceiver_PinOUT;HalfDuplexTransceiver_Port|=HalfDuplexTransceiver_PinIN;

uint16_t SerialPWM;

/**
*@brief Function to send raw data to the bus
*@var Array Pointer to the array of data to be sent to the bus
*@var length Length of the data to be sent
*@return none
*/
uint16_t SendData(uint8_t *Array, uint8_t length);

/**
*@brief Function to init all the serial communication system, prepare buffers, port baudrate etc
*@var SerialSpeed Predifined value (on serial.h) to attribute to the serial port prescaler to work at the desired speed
*@return none
*/
void InitSerial(uint16_t SerialSpeed);

/**
*@brief Utility Function to print ascii coded decimal values to the bus
*@var Data value to be printed
*@return none 
*/
void PrintDecimal (int Data);

/**
*@brief Function to be called by main working as polling to the incomming data, and unpack messages from bus
*@var none
*@return 1 if any data on comm port buffer\n0 if none
*/
uint8_t GetChar(void);

/**
*@brief Function to be called by main working as polling to the outgoing data
*@var none
*@return 1 if byte sent to hard buff\n -1 if hard buff full and no byte sent there\n 0 if uart buff empty
*/
int8_t SendChar(void);

/**
*@brief Function to get data message incoming from bus, contained on messages buffer
*@var Array Pointer to the container where to copy data to
*@var Start Buffer postition where to start getting the message
*@var Length Number of bytes to read
*@var free Set buffer free for new messages
*@return 0 if no data message ready to read\n -1 if the Start plus length are bigger than message length\nMessage length if avaliable data to read
*/
int8_t GetMSG(uint8_t *Array,uint8_t Start,uint8_t Length, uint8_t free);

/**
*@brief Function to pack data and send it in a message pack to the bus
*@var Array Pointer to the array of data to be sent
*@var length Length of data to be send
*@return none
*/
void SendMSG(uint8_t *Array, uint8_t length, uint8_t DestinationID);
