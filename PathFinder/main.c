//#define F_CPU 8000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "movements.h"

typedef unsigned char byte;

//#define bool char
//#define true 1
//#define false 0

// Map size 6*6
#define row 6
#define col 6

// A Star definitions
byte goalN; // goal position on grid
byte openList[50]; // contains all the possible paths
byte closedList[50]; // contains the path taken
byte Path[50];
byte oLN=0, cLN=0;//the counters for the openList and closedList
byte curBotPos = 0 ; // holds current bot position
byte curBotDir = 1 ; // holds current bot facing direction(1 up  2 down 3 left  4 right)
byte curBotPos2;
byte curBotDir2;
byte destination;


// Differential PWM signals
#define rightPWM	PORTB3	// OC0
#define leftPWM		PORTD7	// OC2

#define RF	PORTB0	// IN1 = RF
#define RR	PORTB1	// IN2 = RR
#define LR	PORTA2	// IN3 = LR
#define LF	PORTB4	// IN4 = LF

#define led PORTA0
#define led2 PORTA1

// encoder pins
#define leftEncoder		PORTD2	// int0
#define rightEncoder	PORTD3	// int1

#define leftLed			PORTA6
#define rightLed		PORTA4


// sonar pins
#define trigger PORTD0
#define echo	PORTD6

// source pins
#define source0 PORTC0
#define source1 PORTC1

// dest pins
#define dest0 PORTC2
#define dest1 PORTC3

uint16_t fallingTime = 0;
uint16_t distance = 999;
volatile unsigned char rising = 1;
int threshold = 40;

void setSource(){
	char x = PINA;
	curBotPos = x&0xC0;
	curBotPos = (curBotPos>>6);
}

void setDest(){
	char x = PINA;
	byte dest = x&0x30;
	dest = (dest>>4);
	destination = dest+30;
}

// A star structs

typedef struct Node
{
	byte g, h, f;
	byte parent;
	byte index;
	byte gridNom;
} Node;

struct Grid
{
	Node Map[row][col];
} PF ;

byte H(byte curR, byte curC, byte goalS)  // manhattan distance heauristics function
{
	byte rowg, colg;
	byte manhattan=0;

	
	rowg = (byte)goalS/6;
	colg = goalS%6;
	manhattan += (abs(curR - rowg) + abs(curC - colg));
	
	return manhattan;
}

byte G(byte curR, byte curC)  // returns the number of gride have been traverd
{
	byte gValue, parInd;
	byte rowg, colg;
	parInd = PF.Map[curR][curC].parent;

	rowg = (byte)parInd/6;
	colg = parInd%6;
	gValue = PF.Map[rowg][colg].g;
	
	return (gValue+1);
}

byte FV(byte curG, byte curH) // the total "cost" of the path taken; adds H and G values for each tile
{
	byte fValue;
	
	fValue = curG + curH;
	return fValue;
}

bool isGoal(byte ig) // checks if the goal has been reached
{
	if (ig == goalN)
	{
		return true;
		PORTA |= (1<<led2);
	}
	else{
		PORTA &= ~(1<<led2);
		return false;	
	}
	
}

bool alreadyOnOL(byte rowaol, byte colaol) // checks if the tile is already on the openList
{
	byte indexol;
	bool on = false;

	indexol = rowaol*6 + colaol;
	for (byte i = 0; i < oLN; i++)
	{
		if (openList[i] == indexol)
		{
			on = true;
		}
	}
	
	return on;
}

bool OLE() // checks if the openList is empty
{
	if (oLN == 0)
	{
		return true;
	}
	else
	return false;
}


void triggerSonar(){
	PORTD|=(1<<trigger);
	_delay_us(15);
	PORTD &=~(1<<trigger);
}

int main(void) {
	//DDRA |= (1<<led) | (1<<PORTA1) | (1<<LR);
	DDRA = 0xFF;
	DDRA |= (1<<LR);
	DDRA &= 0x0F;
	
	
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
	
	// source and destination
	//DDRC &= ~((1<<source1)|(1<<source0)|(1<<dest0)|(1<<dest1)); // take input
	DDRC = 0x00;
	//PORTC = 0xFF;
	
	// A star setup
	setSource();
	buildMap();
	setDest();
	setGoal(destination);
	long count;
	
	sei();
	
	 //timer starts for PWM
	 TCCR0 |= (1<<CS00);
	 TCCR2 |= (1<<CS20);
	
	// set trigger signal once
	//_delay_ms(1000);
	//Forward();
	//Forward();
	//Right_Turn();
	//Right_Turn();
	//Left_Turn();
	//Left_Turn();
	triggerSonar();
	while (1) {
		if (!isGoal(curBotPos) && OLE)
		{
			_loop();                                      // the actual performance of the A* algorithm
		}
		else if (isGoal(curBotPos))
		{
			
			PathList();                                   // List the optimal path
			
			_delay_ms(5000);
			while (1){
				
				movement(curBotPos,curBotDir);
				curBotPos = curBotPos2;
				curBotDir = curBotDir2;
				
				if (!isGoal(curBotPos)){
					break;
				}
				
				_delay_ms(5000);
			}
		}
	}
}

void _loop(){                 // performs the A* algorithm, "main" program
	
	possMov(curBotPos);
	AddClosedList();
}

void buildMap() // builds the 6x6 map grid
{
	byte gridIn = 0;
	for (byte i = 0; i < row; i++)
	{
		for (byte j = 0; j < col; j++)
		{
			PF.Map[i][j].gridNom = gridIn;
			PF.Map[i][j].index = 0;
			PF.Map[i][j].parent = 0;
			PF.Map[i][j].h = 0;
			PF.Map[i][j].g = 0;
			PF.Map[i][j].f = 0;
			
			gridIn++;
		}
	}
}

void setGoal(byte goal) // asks user for input to set the goal state/tile
{
	for (byte i = 0; i < row; i++)
	{
		for (byte k = 0; k < col; k++)
		{
			if (PF.Map[i][k].gridNom == goal)
			{
				
				PF.Map[i][k].index = 3;
				goalN = PF.Map[i][k].gridNom;
			}
			else if (PF.Map[i][k].gridNom == curBotPos)  /// initial start point
			{
				PF.Map[i][k].index = 1;
				//curBotPos = PF.Map[i][k].gridNom;
			}
			//else if (PF.Map[i][k].gridNom == 4 || PF.Map[i][k].gridNom == 10 || PF.Map[i][k].gridNom == 16 || PF.Map[i][k].gridNom == 22 || PF.Map[i][k].gridNom == 28 || PF.Map[i][k].gridNom == 34 || PF.Map[i][k].gridNom == 33 || PF.Map[i][k].gridNom == 32 || PF.Map[i][k].gridNom == 31 || PF.Map[i][k].gridNom == 30)
			//{
				//PF.Map[i][k].index = 2;        // initial wall
			//}
			else if (PF.Map[i][k].gridNom == 4 || PF.Map[i][k].gridNom == 10 || PF.Map[i][k].gridNom == 16 || PF.Map[i][k].gridNom == 22 || PF.Map[i][k].gridNom == 28 || PF.Map[i][k].gridNom == 34)
			{
				PF.Map[i][k].index = 2;        // initial wall
			}
			else
			PF.Map[i][k].index = 0;          // initial free space
		}
	}
}

void possMov(byte gridNom) // checks the possible moves depending on the location of the current tile the bot is on
{
	byte rowp = (byte) gridNom / 6;
	byte colp = gridNom % 6;
	if (gridNom == 0) // checks the corner tiles | 2 possible moves
	{
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}

		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
	else if (gridNom == 5)
	{
		if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 1);
		}

		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
	else if (gridNom == 30)
	{
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}

		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 6);
		}
	}
	else if (gridNom == 35)
	{
		if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 6);
		}

		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 1);
		}
	}
	else if (gridNom > 0 && gridNom < 5) // checks the tiles on the outermost edges of the map | 3 possible moves
	{
		if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 1);
		}
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}
		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
	else if (gridNom%6==0)
	{
		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 6);
		}
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}
		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
	else if (gridNom%6==5)
	{
		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 6);
		}
		if (PF.Map[rowp][colp- 1].index != 1 && PF.Map[rowp][colp- 1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 1);
		}
		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
	else if (gridNom > 30 && gridNom < 35)
	{
		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 6);
		}
		if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 1);
		}
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}
	}
	else { // checks the remaining tiles | 4 possible moves
		if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
		{
			PF.Map[rowp-1][colp].parent = gridNom;
			AddOpenList(gridNom - 6);
		}
		if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
		{
			PF.Map[rowp][colp-1].parent = gridNom;
			AddOpenList(gridNom - 1);
		}
		if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
		{
			PF.Map[rowp][colp+1].parent = gridNom;
			AddOpenList(gridNom + 1);
		}
		if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
		{
			PF.Map[rowp+1][colp].parent = gridNom;
			AddOpenList(gridNom + 6);
		}
	}
}

void AddOpenList(byte aol) // adds the potential possible moves to the openList
{
	
	openList[oLN++] = aol;
	heuristics(aol);
}

void heuristics(byte curIn) // calculates the "cost" of the tile
{
	byte hH, gH, fH;
	byte rowh = (byte) curIn / 6;
	byte colh = curIn % 6;

	hH = H(rowh, colh, goalN);
	PF.Map[rowh][colh].h = hH;
	gH = G(rowh, colh);
	PF.Map[rowh][colh].g = gH;
	fH = FV(hH,gH);
	PF.Map[rowh][colh].f = fH;
}

byte getNextFI() // returns the best heuristics value restricted by the current path the bot is taking
{
	byte rowf;
	byte colf;
	byte lowestF;
	byte lowest = openList[0];
	rowf = (byte) lowest / 6;
	colf = lowest % 6;
	lowestF = PF.Map[rowf][colf].f;
	
	for (byte i = 0; i < oLN; i++)
	{
		rowf = (byte) openList[i] / 6;
		colf = openList[i] % 6;
		
		if (PF.Map[rowf][colf].f <= lowestF)
		{
			lowestF = PF.Map[rowf][colf].f;
			lowest = rowf*6 + colf;
		}
	}
	
	return lowest;
}

void AddClosedList() // adds the "best" tile to the closedList
{
	byte low = getNextFI();
	byte rowa, cola;

	closedList[cLN++] = low;
	rowa = (byte)low/6;
	cola = low%6;
	PF.Map[rowa][cola].index = 1;
	curBotPos = low;
	removeFOL(low);
}

void PathList()  // List the optimal path
{
	for(byte i=1;i<PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g+1;i++){
		for(byte j=0;j<cLN;j++){
			if(PF.Map[closedList[j]/6][closedList[j]%6].g == i){
				Path[i-1]=closedList[j];
			}
		}
	}
}

void removeFOL(byte rfol) // removes previous potential paths from the openList, in order to get the "best" current path
{

	for (byte i = 0; i < oLN-30; i++)
	{
		if (openList[i] == rfol)
		{
			openList[i] = openList[i+1];
		}
		else
		openList[i] = openList[i+1];
	}
	oLN=oLN-1;
}




void movement(byte curBotPos,byte curBotr) {
	
	curBotPos = PF.Map[Path[0]/6][Path[0]%6].parent;
	
	byte rowm, colm, parm;
	byte i = 0;

	while(!isGoal(curBotPos)){
		
		rowm = Path[i]/6;
		colm = Path[i]%6;
		_delay_ms(500);
		if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 1){
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 1){
			Left_Turn();
			curBotDir = 3;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 1){
			Right_Turn();
			curBotDir = 4;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 1){
			Right_Turn();
			Right_Turn();
			curBotDir = 2;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;

		}

		else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 2){
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 2){
			Right_Turn();
			curBotDir = 3;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;

		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 2){
			Left_Turn();
			curBotDir = 4;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 2){
			Right_Turn();
			Right_Turn();
			curBotDir = 1;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 3){
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 3){
			Right_Turn();
			curBotDir = 1;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 3){
			Left_Turn();
			curBotDir = 2;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 3){
			Right_Turn();
			Right_Turn();
			curBotDir = 4;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 4){
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 4){
			Right_Turn();
			curBotDir = 2;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 4){
			Left_Turn();
			curBotDir = 1;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
		else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 4){
			Right_Turn();
			Right_Turn();
			curBotDir = 3;
			if (obstacle == 1) {
				rePathPlan(curBotPos,curBotDir);
				break;
			}
			Forward();
			curBotPos = Path[i];
			i++;
		}
	}
	//PORTA |= (1<<led2);
	curBotPos2 = curBotPos;
	curBotDir2 = curBotDir;
	//return curBotPos2,curBotDir2;
	
}

void rePathPlan(byte curBotPos,byte curBotDir) // re-design the path if encounter obstacles
{
	
	for (byte i = 0; i < 36; i++){

		if(PF.Map[i/6][i%6].index == 1){
			PF.Map[i/6][i%6].index = 0;
		}
		PF.Map[i/6][i%6].g = 0;
		PF.Map[i/6][i%6].h = 0;
		PF.Map[i/6][i%6].f = 0;
		PF.Map[i/6][i%6].parent = 0;
	}
	PF.Map[curBotPos/6][curBotPos%6].index = 1;
	PF.Map[goalN/6][goalN%6].index = 3;
	if(curBotDir == 1){
		PF.Map[(curBotPos + 6)/6][(curBotPos + 6)%6].index = 2;
	}
	else if(curBotDir == 2){
		PF.Map[(curBotPos - 6)/6][(curBotPos - 6)%6].index = 2;
	}
	else if(curBotDir == 3){
		PF.Map[(curBotPos + 1)/6][(curBotPos + 1)%6].index = 2;
	}
	else if(curBotDir == 4){
		PF.Map[(curBotPos - 1)/6][(curBotPos - 1)%6].index = 2;
	}
	
	oLN=0;
	cLN=0;

	for (byte i = 0; i<50; i++){
		openList[i] = 0; // contains all the possible paths
		closedList[i] = 0; // contains the path taken
		Path[i] = 0 ;
	}
}




ISR(INT0_vect) {
	// left wheel
	PORTA ^= (1<<leftLed);
	leftCount++;
}


ISR(TIMER0_OVF_vect){
	// 255 ticks have passed
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
		if(distance < threshold){
			obstacle = 1;
			PORTA |= (1<<led);
		}else{
			obstacle = 0;
			PORTA &= ~(1<<led);
		}
	}
}