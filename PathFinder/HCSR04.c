#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define trigger PORTD0
#define echo PORTD2
static volatile int pulse = 0;
static volatile int i = 0;

int main(void)
{

	DDRA = 0xFF;
	DDRD = 0b11111011;
	_delay_ms(50);
	GICR|=(1<<INT0);
	MCUCR|=(1<<ISC00);
	TCCR1A = 0;
	int16_t COUNTA = 0;
	sei();
	while(1)
	{
		PORTD|=(1<<trigger);
		_delay_us(15);
		PORTD &=~(1<<trigger);
		COUNTA = pulse/58;
		if(COUNTA < 10){
			PORTA |= (1<<0);
		}else{
			PORTA &= ~(1<<0);
		}
	}

}

ISR(INT0_vect) {
	if (i==1)
	{
		TCCR1B=0;
		pulse=TCNT1;
		TCNT1=0;
		i=0;
	}
	if (i==0)
	{
		TCCR1B|=(1<<CS10);
		i=1;
	}
}