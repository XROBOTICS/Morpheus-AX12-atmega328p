/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/


#define Position_Dir    	DDRC
#define Position_Port		PORTC
#define Position_Pin		BIT0

#define Temperature_Dir    	DDRC
#define Temperature_Port	PORTC
#define Temperature_Pin		BIT2

#define Motor_Dir				DDRB
#define Motor_Port				PORTB
#define Motor_Clockwise			BIT2
#define Motor_Anticlockwise		BIT1







/**
*@var Temperature Global Variable of temperature given from 0 to 0x3ff.
*/
uint16_t Temperature;

/**
*@var Position Global Variable of Position given from 0 to 0x3ff.
*/
int16_t Position;

/**
*@var Velocity Global Variable of velocity given in positions/12ms.
*/
int16_t Velocity;



/**
*@brief Function called by a 100us timer, that gets temperature and position values constantly updated\n
This function also refreshes the temperature once every second and the Position every milisecond
*/
void _Control_(void);

/**
*@brief Function (method) to initialize ADs and variables needed for control.
*@var none
*@return 0 if UpperMargin>1022 or LowerMargin<1 or LowerMargin>UpperMargin\n
1 if all OK
*/
uint8_t InitControl(void);

/**
*@brief Function to be called by the Interrupt Service Routine of the ADs sampling ended
*@var none
*@return none
*/
//void ISR_AD (void);

/**
*@brief Function to change motor position
*@var pos New position where to go
*@var speed Velocity from 0 to 10 (10% steps) until reach the New position, during the moving period
*@var MaxPower Motor maximum power between 0% and 100% of the power avaliable, during moving and non-moving periods
*@var spring Hardiness to be given to the motor on the non-moving period, after reach the New position, goes from -20 to 20
*@return 0 if any parameter out of limits\n1 if every parameters ar within the limits
*/
uint8_t SetPosition (int16_t pos, int16_t speed, int16_t MaxPower, int16_t spring);

/**
*@brief Function set a free weel work, chose a Power to apply and the direction
*@var Power Power to apply from 0 to 2000 meaning this -100,0% to 100,0% of the avaliable, being the minus the rotation clockwise or anti clockwise
*@return none
*/
uint8_t SetFreeweeling (uint16_t Power);
//uint8_t SetFreeweeling (uint16_t Power, int8_t Direction);


