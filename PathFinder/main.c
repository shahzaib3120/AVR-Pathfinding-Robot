#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Differential PWM signals
#define rightPWM	PORTB3	// OC0
#define leftPWM		PORTD7	// OC2

#define RF	PORTB0	// IN1 = RF
#define RR	PORTB1	// IN2 = RR
#define LR	PORTA2	// IN3 = LR
#define LF	PORTB4	// IN4 = LF

#define led PORTA0

// encoder pins
#define leftEncoder PORTB2
#define  rightEncoder PORTD3
#define leftLed PORTA6
#define rightLed PORTA4


// sonar pins
#define trigger PORTD0
#define echo	PORTD2

static volatile int pulse = 0;
uint16_t distance = 0;
char i = 0;
int threshold = 10;

double rightDutyCycle = 74;
double leftDutyCycle = 75;

void moveForward() {
	for(char x=0; x<100; x++){
		//if(distance > 20){
			PORTB |= (1<<RF);
			PORTB |= (1<<LF);
		//}
	}
}

void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void) {
	//DDRA |= (1<<led) | (1<<PORTA1) | (1<<LR);
	DDRA = 0xFF;
	DDRB |= (1<<RF) | (1<<RR) | (1<<LF);
	DDRB &= ~(1<<leftEncoder);
	DDRD |= (1<<trigger);
	DDRD &= ~(1<<echo);
	DDRD &= ~(1<<rightEncoder);
	PORTA=0x00;
	
	// PWM Setup
	DDRB |= (1<<rightPWM);
	DDRD |= (1<<leftPWM);
	TCCR0 = (1<<COM01) | (1<<WGM00) | (1<<WGM01); // set non-inverting Fast PWM mode on timer0
	TCCR2 = (1<<COM21) | (1<<WGM20) | (1<<WGM21); // set non-inverting Fast PWM mode on timer2
	TIMSK = (1<<TOIE0) | (1<<TOIE2);
	OCR0 = (rightDutyCycle/100)*255;
	OCR2 = (leftDutyCycle/100)*255;
	
	// setup for Sonar
	GICR|=(1<<INT0);	// enable external interrupt
	MCUCR|=(1<<ISC00) | (1<<ISC01);	// Any logical change on INT0 generates an interrupt request (rising)
	TCCR1A = 0;			// normal mode
	
	// setup for speed encoders
	GICR |= (1<<INT1);
	MCUCR|= (1<<ISC11) | (ISC10);
	MCUCSR |= (1<<ISC2);
	GICR |= (1<<INT2);
	
	
	sei();
	
	// timer starts for PWM
	TCCR0 |= (1<<CS00);
	TCCR2 |= (1<<CS20);
	
	// set trigger signal once
	triggerSonar();
	while (1) {
		/*PORTA |= (1<<PORTA1);
		_delay_ms(100);
		PORTA &= ~(1<<PORTA1);
		_delay_ms(100);*/
		moveForward();
	}
}

ISR(INT0_vect) {
		
		// left wheel
		PORTA |= (1<<leftLed);
		_delay_ms(10);
		PORTA &= ~(1<<leftLed);
		
		
}

ISR(TIMER0_OVF_vect){
	OCR0 = (rightDutyCycle/100)*255;
	//rightDutyCycle = rightDutyCycle==250?0:rightDutyCycle;
	//OCR0 = rightDutyCycle++;
}

ISR(TIMER2_OVF_vect){
	OCR2 = (leftDutyCycle/100)*255;
	//leftDutyCycle = leftDutyCycle==250?0:leftDutyCycle;
	//OCR2 = leftDutyCycle++;
}

	
ISR(INT1_vect){
	// right wheel
	PORTA |= (1<<rightLed);
	_delay_ms(10);
	PORTA &= ~(1<<rightLed);
}

ISR(INT2_vect){

	if (i==1) {
		TCCR1B = 0;		// stops the timer when echo is registered again
		distance = TCNT1/58;
		TCNT1=0;		// resets timer
		if(distance < 20){
			PORTA |= (1<<led);
		}
		i=0;			// resets echo teller state
		// set the trigger signal again
		triggerSonar();
		PORTA &= ~(1<<led);
	}
	if (i==0) {
		TCCR1B|=(1<<CS10);	// starts timer 0 with no prescalar when echo is registered for the first time
		i=1;				// tells echo has been registered
	}
}