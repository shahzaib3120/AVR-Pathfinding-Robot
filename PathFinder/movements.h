/*
 * movements.h
 *
 * Created: 6/4/2023 12:00:13 PM
 *  Author: HP
 */

#ifndef MOVEMENTS_H_
#define MOVEMENTS_H_

#define RF PORTB0 // IN1 = RF
#define RR PORTB1 // IN2 = RR
#define LR PORTB2 // IN3 = LR
#define LF PORTB4 // IN4 = LF
#define BLOCK_SIZE 40
#define led2 PORTA1
double ticksPerCm = 22.0 / 30.0;

int turns = 8;
int timeTurn = 20800;
double straight = BLOCK_SIZE * ticksPerCm;

int leftCount = 0;
int rightCount = 0;
volatile unsigned char obstacle = 0;

double rightDutyCycle = 79.5;
double leftDutyCycle = 78.8;

void setRightDuty(double perc)
{
	OCR0 = (perc / 100) * 255;
}

void setLeftDuty(double perc)
{
	OCR2 = (perc / 100) * 255;
}

void Forward()
{
	leftCount = 0;
	rightCount = 0;
	while (leftCount <= straight && rightCount <= straight)
	{
		PORTB |= (1 << RF);
		PORTB |= (1 << LF);
		// PORTA &= ~(1<<led2);
		_delay_ms(100);
		// PORTA |= (1<<led2);
		PORTB &= ~(1 << LF);
		PORTB &= ~(1 << RF);
	}
	_delay_ms(1000);
}

void Left_Turn()
{
	leftCount = 0;
	rightCount = 0;
	_delay_ms(100);
	while (!(leftCount > turns && rightCount > turns))
	{
		PORTB |= (1 << RF);
		PORTB |= (1 << LR);
	}
	// for(int i=0; i<timeTurn; i++){
	// PORTB |= (1<<RF);
	// PORTA |= (1<<LR);
	//}
	PORTB &= ~(1 << RF);
	PORTB &= ~(1 << LR);
	leftCount = 0;
	rightCount = 0;
	_delay_ms(1000);
}

void Right_Turn()
{
	leftCount = 0;
	rightCount = 0;
	_delay_ms(100);
	while (!(leftCount > turns + 1 && rightCount > turns))
	{
		PORTB |= (1 << LF);
		PORTB |= (1 << RR);
	}
	// for(int i=0; i<timeTurn; i++){
	// PORTB |= (1<<LF);
	// PORTB |= (1<<RR);
	//}
	PORTB &= ~(1 << LF);
	PORTB &= ~(1 << RR);
	leftCount = 0;
	rightCount = 0;
	_delay_ms(1000);
}

#endif /* MOVEMENTS_H_ */