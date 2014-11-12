/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/
#define Rsv_Addr            0x0000
#define MY_ID_Addr 			0x0001
#define CTL_UP_LIMIT_Addr 	0x0002
#define CTL_LW_LIMIT_Addr	0x0003
#define KPp_PID_Addr		0x0004
#define KIp_PID_Addr		0x0005
#define KDp_PID_Addr		0x0006
#define KPv_PID_Addr		0x0007
#define KIv_PID_Addr		0x0008
#define KDv_PID_Addr		0x0009
#define UART_BAUD_Addr		0x000A

#define EEVarSize              0x000B



#define Rsv_Dflt            0x00AA
#define MY_ID_Dflt 			0x0001
#define CTL_UP_LIMIT_Dflt 	0x0002
#define CTL_LW_LIMIT_Dflt	0x0002
#define KPp_PID_Dflt		0x0028
#define KIp_PID_Dflt		0x0001
#define KDp_PID_Dflt		0x000A
#define KPv_PID_Dflt		0x0028
#define KIv_PID_Dflt		0x000A
#define KDv_PID_Dflt		0x0000
#define UART_BAUD_Dflt		0x0001 // 1000000 baud



unsigned char MY_ID;
unsigned char CTL_UP_LIMIT;
unsigned char CTL_LW_LIMIT;
unsigned char KPp_PID;
unsigned char KIp_PID;
unsigned char KDp_PID;
unsigned char KPv_PID;
unsigned char KIv_PID;
unsigned char KDv_PID;
unsigned char UART_BAUD;



/**
*@brief Function to load all the variables from the EEPROM
*@var none
*@return none
*/
void EELoadVars(void);

/**
*@brief Function to load one the variables (or memory position) from the EEPROM
*@var Variable Memory position to get the value from.
*@return Value contained on the selected memory position of the EEPROM
*/
uint8_t EEGet(uint16_t Variable);


/**
*@brief Function to save a value to one variable (memory position) on the EEPROM, and test the controlled variables such as Baudrate or ID
*@var Variable Memory position to save the value to
*@var data Data to be saved
*@return 1 if everything ok\n 0 if an invalid address or the value is out of the limits to that variable
*/
uint8_t EESave(uint16_t Variable, uint8_t data);



/**
*@brief Function save one or more values to the EEPROM, starting on the given address
*@var address Address where to start reading the values
*@var data Pointer to the array where to save the readen values from EEPROM
*@var length Number of bytes to read
*@return 1 if everything ok\n 0 if an invalid address or any controlled value is out of the limits to that variable
*/
uint8_t EEWrite(uint16_t address, uint8_t *data, uint8_t length);


/**
*@brief Function to load one or more values from the EEPROM, starting on the given address
*@var address Address where to start writing the values
*@var data Pointer to the array of values to be saved on EEPROM
*@var length Number of bytes to writed
*@return none
*/
void EERead(uint16_t address, uint8_t *data, uint8_t length);


/**
*@brief Reset ALL the variables of the Servo, including ID, to the default variables
*@var none
*@return none
*/
void EEPromReset(void);
