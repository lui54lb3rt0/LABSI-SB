/********************************************************
Language            :      C
Year                :      2019/2020
Class               :      LABSI
Authors             :      Luis Silva | Joao Loureiro
E-mail              :      1101420@isep.ipp.pt | 1131109@isep.ipp.pt
*********************************************************/

#define F_CPU 16000000UL                                                  /* Define CPU clock Frequency e.g. here its 16MHz */
#include <avr/io.h>                                                              /* Include AVR std. library file */
#include <avr/interrupt.h>                                                /* Include AVR interrupt library file */
#include <util/delay.h>                                                          /* Include delay header file */
#include <inttypes.h>                                                            /* Include integer type header file */
#include <stdlib.h>                                                              /* Include standard library file */
#include <stdio.h>
#include <stdint.h>
#include "MPU6050_res_define.h"                                           /* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"                                            /* Include I2C Master header file */

typedef struct USARTRX
{
	char receiver_buffer;
	unsigned char status;
	unsigned char receive 	: 1;
	unsigned char error	: 1;

}USARTRX_st;

volatile USARTRX_st rxUSART = {0,0,0,0};
char TxBuffer[50];
char float_[10];

/* Functions */

void SendMessage(char * buffer)
{
	unsigned char i=0;
	while(buffer[i]!='\0')
	{
		while((UCSR0A & 1<<UDRE0)==0);
		UDR0=buffer[i];
		i++;
	}
}

volatile uint8_t timerFlag = 0;
volatile float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;
volatile int ledCounter = 0;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime = 0.001408;           //TS - time sample
float rad_to_deg = 180/3.141592654;

int PWM, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
float pid_tot=0;
float previous_pid_tot = 0;

/////////////////PID CONSTANTS/////////////////
float kp = 0;
float ki = 0;
float kd = 0;
///////////////////////////////////////////////

float desired_angle = 0; //This is the angle in which we want the balance to stay steady

/* Functions start here */
void inic()
{
	
	DDRC = 0x0F;		// Set PC0 & PC1 as output to Motor direction & scope test probes
	DDRB = 0x06;		// Set PB1 and PB2 as output OC1A & OC1B
	DDRD = (1<<DDD3) | (1<<DDD4) | (1<<DDD5) | (1<<DDD6) ;

	PORTD = (0<<DDD2) | (1<<DDD3)  | (1<<DDD5) | (1<<DDD6);
	PORTC = 0b00000011;

	/* Timer 1 */
	TCCR1A = ((1 << COM1A0) | (1 << COM1B0));									// Enable OC1A e OC1B to motors
	TCCR1B = ((1 << CS12) | (1 << WGM12));										// CLKIO/256 (From prescaler) | Mode CTC
	OCR1A = 65535;																// motor speed right
	OCR1B = 65535;																// motor speed left


	/* Timer 2 */
	TCCR2A = (1<<WGM21);														// MODE CTC
	TCCR2B = ((1 << CS21) | (1 << CS22));										// CLKIO/256 (From prescaler)
	TIMSK2 = (1 << OCIE2A);													// Enable TIMER0
	OCR2A = 88;																	// ~0.001088s  ~900Hz    68

	/* USART */
	UBRR0H=0;
	UBRR0L=51;									// BAUDRATE 38400
	UCSR0A=(1<<U2X0);							// Double Speed
	UCSR0B=(1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);	// RX Complete Interrupt Enable, Receiver Enable; transmite Enable
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);				// 8 bits
	
	sei();
	
	sprintf(TxBuffer, "Init successfully!!!\r\n");
	SendMessage(TxBuffer);

}

void MPU6050_Init()                                     /* Gyro initialization function */
{
	_delay_ms(250);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);                              /* Write to power management register */
	I2C_Write(0x01);                                    /* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);                                  /* Write to Configuration register */
	I2C_Write(0x00);                                    /* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);                                    /* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);							/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
	I2C_Repeated_Start(0xD1);						/* I2C start with device read address */
}

void Read_RawValue()
{
	
	MPU_Start_Loc();            /* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();

}

void AngleCalc()
{
	//---ACC  Y---
	Acceleration_angle[1] = atan(-1*(Acc_x/16384.0)/sqrt(pow((Acc_y/16384.0),2) + pow((Acc_z/16384.0),2)))*rad_to_deg;

	//---GYRO Y---
	Gyro_angle[1] = Gyro_y/131.0;
	
	//Filtro complementar
	//---Y axis angle--- Pitch
	Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
}

float sat(float _pid, int umin, int umax)
{
	if (_pid <= umin)
	{
		_pid = umin;
	}

	if (_pid >= umax)
	{
		_pid = umax;
	}
	return _pid;
}

void PID()
{
	error = Total_angle[1] - desired_angle;
	
	pid_i =+ki*(error+previous_error);
	pid_i = sat(pid_i,-1.5,2);
	pid_tot = previous_pid_tot + kp* (error-previous_error) + pid_i;
	
	//change motors direction
	if (error < 0)
	{
		PORTC = 0b00000000;
	}
	if (error >= 0)
	{
		PORTC = 0b00000011;
	}
	
	pid_tot = sat(pid_tot,-9,9);		//-10 10
	
	previous_error = error;
	previous_pid_tot = pid_tot;
	
}


int main()
{
	inic();                                              // Main configurations
	_delay_ms(500);
	I2C_Init();                                          // Initialize I2C
	MPU6050_Init();                                      // Initialize MPU6050
	desired_angle = 0;
	kp =0.55;
	ki = ((kp/0.9)*(elapsedTime/2));

	while(1)
	{
		//Set SETPOINT
		if((PIND & (1 << PIND2)) == 0 ) {
			OCR1A = 65535;
			OCR1B = 65535;
			desired_angle = Total_angle[1];
		}
		
		if (timerFlag==1)
		{
			Read_RawValue();	// Read gyro and acc from MPU6050
			AngleCalc();
			
			PID();
			
			PWM = 40.302*exp(-0.31*abs(pid_tot)); // a que funciona fixe
			if ((error > -0.05) && (error <= 0.05)) PWM =10800;
			
			if ( (Total_angle[1] >= 0) && (Total_angle[1] <= 0.5)) {
				PWM = 65000;
			}
			else if ((Total_angle[1] >= -0.5) && (Total_angle[1] <= 0)) {
				PWM = 65000;
			}
			
			if ((Total_angle[1]>30) | (Total_angle[1]<-30))
			{
				PWM = 65535;
				pid_tot=0;
				pid_i=0;
				previous_error=0;
				previous_pid_tot=0;
			}


			if (TCNT1 >= PWM){
				TCNT1 = 0;
				OCR1A = PWM;
				OCR1B = PWM;
			}
			else
			{
				OCR1A = PWM;
				OCR1B = PWM;
			}
			
			timerFlag=0;
		}
	}
}


/* Interrupts */
ISR(TIMER2_COMPA_vect)
{
	
	if (timerFlag==0)
	{
		timerFlag=1;
	}
	if (ledCounter==500) {
		PORTD ^= (1<<DDD3);
		ledCounter = 0;
	}
	ledCounter ++;
}

ISR (USART_RX_vect)
{
	rxUSART.status = UCSR0A;
	if(rxUSART.status & ((1<<FE0) | (1<<DOR0) | (1<<UPE0)))
	rxUSART.error = 1;
	rxUSART.receiver_buffer = UDR0;
	rxUSART.receive = 1;
}