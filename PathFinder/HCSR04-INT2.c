#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define led PORTA0
#define led2 PORTA3

// sonar pins
#define trigger PORTD0
#define echo	PORTD2

uint16_t distance = 0;
static volatile int i = 0;
int threshold = 30;

void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void)
{
	DDRA |= (1<<led2) | (1<<led);
	
	DDRD |= (1<<trigger);
	DDRD &= ~(1<<echo);
	
	// setup for Sonar
	GICR|=(1<<INT0);	// Enabling external interrupt
	MCUCR|=(1<<ISC00); 	// Any logical change on INT0 generates an interrupt request.
	
	TCCR1A = 0;			// Normal mode
	
	sei();
	
	// set trigger signal once
	triggerSonar();
	
	while(1)
	{
	}

}

ISR(INT0_vect){
	if (i==1) {
		TCCR1B = 0;		// stops the timer when echo is registered again
		distance = TCNT1/58;
		TCNT1=0;		// resets timer
		if(distance < 10){
			PORTA |= (1<<led2);
		}
		i=0;			// resets echo teller state
		// set the trigger signal again
		triggerSonar();
		PORTA &= ~(1<<led2);
	}
	if (i==0) {
		TCCR1B|=(1<<CS10);	// starts timer 0 with no prescalar when echo is registered for the first time
		i=1;				// tells echo has been registered
	}
}