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

#define Motor_Clockwise 	BIT2
#define Motor_Anticlockwise	BIT1
#define Motor_Port 			PORTB
#define Motor_Dir 			DDRB



/**
*@brief simply control PWM for the motor.\n
The pwm must be givem with values between 0 and 2000.\n
The 1000 is the motor stopped. The 0 is the 100% PWM rotating Anticlockwise and the 2000 is the 100% PWM rotating Clockwise
The PWM has increments of one step with 1000 steps of resolution.
*@var pwm This is the pwm to set on the motor
*@return 2 if rotating clockwise.\n
1 if stopped.\n
0 if anticlockwise.\n
-1 if fail or bad value or limit reached.
*/
int8_t SetPWM(uint16_t pwm);


/**
*@brief Init the motor low level control, on this case pins, timers, pwm variables, etc.
*/
void InitMotor(void);

uint16_t PWM;