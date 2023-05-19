#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
// Differential PWM signals
#define rightPWM	PORTD4	// OC1B
#define leftPWM		PORTD5	// OC1A

double rightDutyCycle = 0;
double leftDutyCycle =0;

void pwmSetup(){
	
}

int main(void)
{
	DDRD |= (1<<rightPWM) | (1 << leftPWM);
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM12) | (1<<WGM10);
	TIMSK = (1<<TOIE1);
	OCR1B = (rightDutyCycle/100)*255;
	OCR1A = (leftDutyCycle/100)*255;
	sei();
	// timer starts
	TCCR1B |= (1<<CS10);
    while(1)
    {
        //TODO:: Please write your application code 
		_delay_ms(10);
		rightDutyCycle += 10;
		leftDutyCycle += 20;
		if(rightDutyCycle > 100){
			rightDutyCycle =0;
		}
		if(leftDutyCycle >100){
			leftDutyCycle =0;
		}
	}
}

ISR(TIMER1_OVF_vect){
	OCR1B = (rightDutyCycle/100)*255;
	OCR1A = (leftDutyCycle/100)*255;
}