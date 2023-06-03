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
#define led2 PORTA3

// encoder pins
#define leftEncoder		PORTD2	// int0
#define rightEncoder	PORTD3	// int1

#define leftLed			PORTA6
#define rightLed		PORTA4


// sonar pins
#define trigger PORTD0
#define echo	PORTD6

uint16_t fallingTime = 0;
uint16_t distance = 0;
volatile unsigned char rising = 1;
int threshold = 10;

double rightDutyCycle = 80;
double leftDutyCycle = 79;


int leftCount = 0;
int rightCount = 0;

void moveForward() {
	PORTB |= (1<<RF);
	PORTB |= (1<<LF);
	_delay_ms(10);
	PORTB &= ~(1<<RF);
	PORTB &= ~(1<<LF);
}

void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void) {
	//DDRA |= (1<<led) | (1<<PORTA1) | (1<<LR);
	DDRA = 0xFF;
	DDRA &= ~(1<<LR);
	
	
	DDRB |= (1<<RF) | (1<<RR) | (1<<LF);
	
	DDRD &= ~(1<<echo);
	DDRD |= (1<<trigger);
	DDRD &= ~(1<<leftEncoder);
	DDRD &= ~(1<<rightEncoder);
	
	// PWM Setup
	DDRB |= (1<<rightPWM);
	DDRD |= (1<<leftPWM);
	
	TCCR0 = (1<<COM01) | (1<<WGM00) | (1<<WGM01); // set non-inverting Fast PWM mode on timer0
	TCCR2 = (1<<COM21) | (1<<WGM20) | (1<<WGM21); // set non-inverting Fast PWM mode on timer2
	TIMSK = (1<<TOIE0) | (1<<TOIE2);
	OCR0 = (rightDutyCycle/100)*255;
	OCR2 = (leftDutyCycle/100)*255;
	
	// setup for Sonar
	TCCR1A = 0;
	TCCR1B |= (1<<ICES1); // detect rising edge
	TIMSK |= (1<<TICIE1);
	
	// setup for speed encoders
	GICR |= (1<<INT1) | (1<<INT0); // enable INT0 and INT1
	MCUCR|= (1<<ISC11) | (ISC10) | (1<<ISC01) | (1<<ISC00); // detect changes on rising edge for INT1 and INT0
	
	sei();
	
	 //timer starts for PWM
	 TCCR0 |= (1<<CS00);
	 TCCR2 |= (1<<CS20);
	
	// set trigger signal once
	triggerSonar();
	int thresh = 80;
	while (1) {
		if(leftCount <= thresh){
			//PORTA |= (1<led);
			PORTB |= (1<<LF);
		}else{
			//PORTA &= ~(1<led);
			PORTB &= ~(1<<LF);
		}
			
		if(rightCount <= thresh){
			//PORTA |= (1<led2);
			PORTB |= (1<<RF);
		}else{
			//PORTA &= ~(1<led2);
			PORTB &= ~(1<<RF);
		}
	}
}

ISR(INT0_vect) {
	// left wheel
	PORTA ^= (1<<leftLed);
	leftCount++;
}

ISR(TIMER0_OVF_vect){
	// right
}

ISR(TIMER2_OVF_vect){
	//left
}

ISR(INT1_vect){
	// right wheel
	PORTA ^= (1<<rightLed);
	rightCount++;
}

ISR(TIMER1_CAPT_vect) {
	if(rising){
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