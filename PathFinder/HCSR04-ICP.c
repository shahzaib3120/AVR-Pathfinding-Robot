#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define led PORTA0
#define led2 PORTA3

// sonar pins
#define trigger PORTD0
#define echo	PORTD2

uint16_t risingTime = 0;
uint16_t fallingTime = 0;
uint16_t distance = 0;
volatile unsigned char rising = 1;

void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void) {	
	DDRA |= (1<<led2) | (1<<led);
	//PORTA |= (1<<led);
	DDRD |= (1<<trigger);
	DDRD &= ~(1<<echo);

	TCCR1A = 0;	
	TCCR1B |= (1<<ICES1); // detect rising edge
	TIMSK |= (1<<TICIE1);

	sei();
	triggerSonar();
	while(1)
	{
		//triggerSonar();
	}

}

ISR(TIMER1_CAPT_vect) {
	if(rising){
		risingTime = ICR1;
		TCCR1B &= ~(1<<ICES1);	// detect falling edge next time
		TCCR1B |= (1<<CS10);	// start timer
		rising =0;
	}else{
		fallingTime = ICR1;
		TCCR1B |= (1<<ICES1);	// detect rising edge next time
		rising = 1;
		TCCR1B &= ~(1<<CS10);	// stop the timer
		TCNT1 = 0;
		distance = (fallingTime)/58;
		triggerSonar();
		if(distance < 20){
			PORTA |= (1<<led);
			}else{
			PORTA &= ~(1<<led);
		}
	}

	
}