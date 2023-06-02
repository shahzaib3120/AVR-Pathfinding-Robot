#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define rightLed PORTA6
#define leftLed PORTA4
#define led PORTA0

#define int2 PORTB2
#define int1 PORTD3

int main(void){
	DDRD = 0;
	DDRB = 0;
	DDRA = 0xFF;
	
	GICR |= (1<<INT1);
	MCUCR|= (1<<ISC11) | (ISC10);
	MCUCSR |= (1<<ISC2);
	GICR |= (1<<INT2);
	sei();
	
	while(1){
		PORTA ^= (1<<led);
		_delay_ms(100);
	}
}

ISR(INT2_vect){
	PORTA |= (1<<rightLed);
	_delay_ms(10);
	PORTA &= ~(1<<rightLed);
}
ISR(INT1_vect){
	PORTA |= (1<<leftLed);
	_delay_ms(10);
	PORTA &= ~(1<<leftLed);
}