#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define RF	PORTB0	// IN1 = RF
#define RR	PORTB1	// IN2 = RR
#define LR	PORTB2	// IN3 = LR
#define LF	PORTB3	// IN4 = LF
#define led PORTA0
#define trigger PORTD0
#define echo	PORTD2

static volatile int pulse = 0;
uint16_t distance = 0;
char i = 0;

void moveForward() {
	for(char x=0; x<100; x++){
		PORTB |= (1<<RF);
		PORTB |= (1<<LF);
		_delay_ms(10);
		PORTB &= ~(1<<RF);
		PORTB &= ~(1<<LF);
		_delay_ms(20);
	}
}

void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void) {
	DDRA = (1<<led);
	DDRB |= (1<<RF) | (1<<RR) | (1<<LF) | (1<<LR);
	DDRD = (1<<trigger);
	DDRD &= ~(1<<echo);
	PORTA=0x00;
	
	// setup for Sonar
	GICR|=(1<<INT0);	// enable external interrupt
	MCUCR|=(1<<ISC00);	// Any logical change on INT0 generates an interrupt request.
	TCCR1A = 0;			// normal mode
	sei();
	
	// set trigger signal once
	triggerSonar();
	
	while (1) {
		// PORTA|=	(1<<0);
		// _delay_ms(500);
		// moveForward();
		// PORTA &= ~(1<<0);
		// _delay_ms(500);
		
		if(distance	< 10){
			PORTA |= (1<<led);
			_delay_ms(100);
		}else{
			PORTA &= ~(1<<led);
			_delay_ms(100);
			moveForward();
		}
	}
}

ISR(INT0_vect) {
	if (i==1) {
		TCCR1B=0;		// stops the timer when echo is registered again
		distance = TCNT1/58;
		TCNT1=0;		// resets timer
		i=0;			// resets echo teller state
		// set the trigger signal again
		triggerSonar();
	}
	if (i==0) {
		TCCR1B|=(1<<CS10);	// starts timer 1 with no prescalar when echo is registered for the first time
		i=1;				// tells echo has been registered
	}
}
