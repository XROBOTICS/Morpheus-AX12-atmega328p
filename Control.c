/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/
/************************************************
Fixed bugs:
Limits implemented on the Integral accumulative error
so it woulddn't overflow and start over, causing instability 
and oscilation on the speed PID controller.
These limits also solve the problem of torque compensation to
preform speed control with forces being applied against
and in favor of the rotation.

*************************************************/



#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include "UtilsAndDefines.h"
#include "Control.h"
#include "Motor.h"
#include "Eprom.h"


uint8_t GET_Temperature=0;
uint8_t GET_Position =0;

extern uint16_t Temperature=0;
extern int16_t Position=0;
extern int16_t Velocity=0;


#define NumberOfPositions 15 // this deppends directly on the number os samplings per second

uint8_t PositionLast=0;
int16_t LastPositions[NumberOfPositions]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


uint8_t ADCFree=1;
uint8_t NewPoint=0;

int16_t MaximumPower = 0; // percentage 0 to 100
int16_t GoalPosition=0;   // 0 to 0x3FF
int16_t GoalVelocity=0;   // 0 to 10
int16_t Springness = 0;   // from -20(very lose) to 20(very hard)  
uint16_t Freeweeling =0;


int16_t Speed_P=0;
int16_t Speed_I=0;
int16_t Speed_D=0;

int16_t Position_P=0;
int16_t Position_I=0;
int16_t Position_D=0;

int16_t Speed_Error=0;
int16_t Speed_OldError=0;

int16_t Position_Error=0;
int16_t Position_OldError=0;


void Control_Loop (void);



uint8_t SetFreeweeling (uint16_t Power)
{
	if(Power<=2000)
	{
		Freeweeling=Power;
		return 1;
	}
	return 0;
}


uint8_t SetPosition (int16_t pos, int16_t speed, int16_t MaxPower, int16_t spring)
{
	if((pos<CTL_LW_LIMIT) || (pos>(1023-CTL_UP_LIMIT)))
		return 0;

	if((speed<0) || (speed>10))
		return 0;

	if((MaxPower<0) || (MaxPower>100))
		return 0;

	if((spring<-20) || (spring>20))
		return 0;


	GoalPosition=pos;
	GoalVelocity = speed;
	Springness=spring;
	MaximumPower = MaxPower;
	NewPoint=1;
	Freeweeling=2001;
	
	return 1;
}




void GetPosition(void)
{
	ADMUX=0;//bit(REFS0); // REFS = 0 1  Mean ARFPIN as reference, 5V refence on this case
	//ADCSRA|=bit(ADIF);//clean flag
	ADCSRA|=bit(ADSC);  // start the conversion processes
	GET_Position=2;
	ADCFree=0;
}


void GetTemperature(void)
{
	ADMUX =bit(MUX1);
	//ADCSRA|=bit(ADIF);//clean flag
	ADCSRA|=bit(ADSC);  // start the conversion processes
	GET_Temperature=2;
	ADCFree=0;
}


uint8_t InitControl(void)
{
	//init ad for positioning
	//0 to 5v
	// the best way to have accurate and fast position sampling, it leaving the ADC0 int free running mode, wich 
	// means the the ADC is constantly sampling, and we can read the register any time that is requested.
	// no need to wait for conversion
	//IMPORTANTE
	// if need to get temperature, need to have in mind that in this mode every channel selection reflects the previous
	// channel as result, just on the next cycle of sampling we will have the selected channel value.

	Motor_Dir|= (Motor_Clockwise|Motor_Anticlockwise);
	
	Position_Dir &= Position_Pin;
	Temperature_Dir &= Temperature_Pin;

	ADCSRA|=(bit(ADPS2)|bit(ADPS1)|bit(ADPS0)); // this selects the adc prescaler to XTALfreq/128= 125KHz
												// this is needed once the best resolution is got with 50 to 200KHz freq

	ADMUX|=bit(REFS0); // REFS = 0 1  Mean ARFPIN as reference, 5V refence on this case
	
	//ADCSRA|=bit(ADSC);  // start the conversion processes
	//ADCSRA|=bit(ADFR); //enable the free running, mean continuousely sampling
	ADCSRA|=bit(ADIE); // enable the adc interrupt
	
	ADCSRA|=bit(ADEN); // enable adc
	ADCSRA&=~bit(ADIF); // clear adc interrupt flag
	return 0;
	//GetPosition();
}

void RotatePosBuff(int16_t lastPos)
{
	
static uint8_t initPosBuff=0;
uint8_t i=0;

	if(initPosBuff==0)
	{
		for(i=0;i<(NumberOfPositions-1);i++)
		{	
			LastPositions[i]=lastPos;
		}
		initPosBuff=1;
	}
	else
	{
		LastPositions[PositionLast]=Position;
		PositionLast++;
		if(PositionLast==NumberOfPositions)
			PositionLast=0;
	}
}




void ISR_AD (void)
{
	//uint16_t returned=0;
	uint8_t temp=0;

	if(GET_Temperature==2)
	{
			temp=0;
			//returned=0;
			temp = ADCL;
			Temperature = ADCH;
			Temperature = ((Temperature<<8)+temp);
			//Temperature = returned; // this represents a 16bit variable
			GET_Temperature=0;
	}
	else
	if(GET_Position==2)
	{
			temp=0;
			//returned=0;
			temp = ADCL;
			Position = ADCH;
			Position = ((Position<<8)+temp);
			//Position = returned; // this represents a 16bit variable
			RotatePosBuff(Position);
			Velocity = (Position-LastPositions[PositionLast]); // (Position-LastPosition2)/2 ms
			if(Velocity<0) 
			{
				Velocity=-Velocity;
			}
			Control_Loop ();
			GET_Position=0;
	}
	ADCFree=1;
}




uint8_t Interrupt_ON=0;



void _Control_(void)
{
static uint8_t PositionCounter=0;
static uint16_t TemperatureCounter=0;

if(Interrupt_ON==1)
{
	ISR_AD();
	Interrupt_ON=0;
}

if(PositionCounter==10)
{
	PositionCounter=0;
	GET_Position=1;

	if(TemperatureCounter==1000)
	{
		TemperatureCounter=0;
		GET_Temperature=1;
	}
	else
	{
		TemperatureCounter++;
	}
}
else
{
	PositionCounter++;
}


	if(ADCFree==1)
	{
		if(GET_Temperature==1) // less important
		{
			GetTemperature();
		}
		else
		if(GET_Position==1) // most important
		{

			GetPosition();
		}
	}
}









ISR(ADC_vect) 
{
	Interrupt_ON=1;
	ADCSRA|=bit(ADIF);
}




void Control_Loop (void)
{
	int16_t CalcPwm=0;
	uint8_t PWMOutLimit=0;

	

	if(Freeweeling<2001)
	{
		CalcPwm=Freeweeling;
	}
	else
	{
		if(PWMOutLimit==0)
		{
			if(Position>(1023-CTL_UP_LIMIT))
			{
				CalcPwm=300;
				PWMOutLimit = 1;
			}
			else
			if(Position<CTL_LW_LIMIT)
			{
				CalcPwm=1700;
				PWMOutLimit = 1;
			}
			else
			{
				if((GoalPosition>(Position+GoalVelocity)) && (NewPoint==1)) //positive goal speed
				{
					//Position_I=0;
					//Position_Error=0;	
					
					Speed_OldError = Speed_Error;
					Speed_Error = (GoalVelocity-Velocity);
					Speed_P = Speed_Error;// hardly the error will be superior than 10

					if( (((KIv_PID*Speed_I)>10000) && (Speed_OldError>0)) || (((KIv_PID*Speed_I)<-10000) && (Speed_OldError<0)) )
					{
						// do nothing
					}
					else
					{
						Speed_I += Speed_OldError;
					}
					Speed_D = (Speed_Error - Speed_OldError);
					CalcPwm = ((KPv_PID*Speed_P) + ((KIv_PID*Speed_I)/10) +  (KDv_PID*Speed_D));
					CalcPwm = (1000+CalcPwm);

				}
				else
				if((GoalPosition<(Position-GoalVelocity)) && (NewPoint==1)) //negative goal speed
				{
					//Position_I=0;
					//Position_Error=0;				
					
					Speed_OldError = Speed_Error;
					Speed_Error = (GoalVelocity-Velocity);	
					Speed_P = Speed_Error;

					if( (((KIv_PID*Speed_I)>10000) && (Speed_OldError>0)) || (((KIv_PID*Speed_I)<-10000) && (Speed_OldError<0)) )
					{
						// do nothing
					}
					else
					{
						Speed_I += Speed_OldError;
					}
					Speed_D = (Speed_Error - Speed_OldError);
					CalcPwm = ((KPv_PID*Speed_P) + ((KIv_PID*Speed_I)/10) +  (KDv_PID*Speed_D));
					CalcPwm = (1000-CalcPwm);

				}
				else// no	speed
				{
					Speed_Error=0;
					Speed_I=0;

					Position_OldError = Position_Error;
					Position_Error = (GoalPosition-Position);

					// note	
					Position_P = Position_Error;

					if( (((KIp_PID*Position_I)>10000) && (Position_OldError>0)) || (((KIp_PID*Position_I)<-10000) && (Position_OldError<0)) )
					{
						// do nothing
					}
					else
					{
						Position_I += Position_OldError;
					}
					Position_D = (Position_Error - Position_OldError);
					NewPoint=0;
					CalcPwm = ((KPp_PID*Position_P) + ((KIp_PID*Position_I)/100) + (KDp_PID*Position_D));
					CalcPwm = (1000+CalcPwm);

				}
			}
		}
		else
		{
			if((Position>0) && (Position<CTL_LW_LIMIT))
				PWMOutLimit=0;

			if((Position>(1023-CTL_UP_LIMIT)) && (Position<1023))
				PWMOutLimit=0;
		}

		if(CalcPwm>(1000+(MaximumPower*10)))
		CalcPwm=(1000+(MaximumPower*10));
			
		if(CalcPwm<(1000-(MaximumPower*10)))
		CalcPwm=(1000-(MaximumPower*10));
	}
			

	SetPWM(CalcPwm);

	
}


