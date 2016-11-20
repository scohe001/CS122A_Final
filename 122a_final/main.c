/*
 * final_proj.c
 *
 * Created: 11/7/2016 12:24:11 PM
 *  Author: student
 */ 

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/portpins.h>
#include <avr/pgmspace.h>

#include <avr/io.h>
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "joystick.h"
#include "queue.h"
unsigned char receivedData = 0xFF;

void SPI_ServantInit(void) {
	// set DDRB to have MISO line as output and MOSI, SCK, and SS as input
	DDRB = 0x40;
	// set SPCR register to enable SPI and enable SPI interrupt (pg. 168)
	SPCR = SPCR|0xC0;
	// make sure global interrupts are enabled on SREG register (pg. 9)
	SREG = SREG|0x80;
	
}

ISR(SPI_STC_vect) { // this is enabled in with the SPCR register’s “SPI
	// Interrupt Enable”
	// SPDR contains the received data, e.g. unsigned char receivedData =
	// SPDR;
	receivedData = SPDR;
}


//For four shift registers sharing RCLK, SRCLK and SRCLR
//Means only one more pin is needed for the second register
//However they both must be updated together
#define SHIFT_BUS PORTC
#define SER1	0
#define SER2	4
#define SER3	5
#define SER4	6
#define RCLK	1
#define SRCLK	2
#define SRCLR	3

unsigned char setBit(unsigned char vec, unsigned char bit_num, unsigned char val) {
	if(val) return vec | (1 << bit_num);
	return vec & ~(1 << bit_num);
}

void transmit_data(unsigned char data1, unsigned char data2, unsigned char data3, unsigned char data4) {
	//Set SRCLR high and RCLK low
	SHIFT_BUS |= (1 << SRCLR);
	SHIFT_BUS &= ~(1 << RCLK);
	
	for(unsigned char x=0; x < 8; x++) {
		SHIFT_BUS |= (1 << SRCLR);
		SHIFT_BUS &= ~(1 << SRCLK);
		SHIFT_BUS = setBit(SHIFT_BUS, SER1, data1 & 0x80);
		SHIFT_BUS = setBit(SHIFT_BUS, SER2, data2 & 0x80);
		SHIFT_BUS = setBit(SHIFT_BUS, SER3, data3 & 0x80);
		SHIFT_BUS = setBit(SHIFT_BUS, SER4, data4 & 0x80);
		data1 <<= 1, data2 <<= 1; data3 <<= 1; data4 <<= 1;
		SHIFT_BUS |= (1 << SRCLK);
	}
	SHIFT_BUS |= (1 << RCLK);
}

//Control the motors every tick
enum ShiftState {SINIT} shift_state;
unsigned char shift1 = 0;
unsigned char shift2 = 0;
unsigned char debug1 = 0; //Not using the last two shift registers, so hook them up to
unsigned char debug2 = 0; //An LED Strip and use them for debugging

void Shift_Init() {
	shift_state = SINIT;
}

void Shift_Tick() {
	switch(shift_state) {
		case SINIT:
			transmit_data(shift1, shift2, debug1, debug2);
			break;
	}
	
	switch(shift_state) {
		case SINIT:
			shift_state = SINIT;
			break;
		default:
			shift_state = SINIT;
			break;
	}
}

void Shift_Task() {
	Shift_Init();
	for(;;) {
		Shift_Tick();
		vTaskDelay(2);
	}
}

//LEFT
#define MOTOR1PORT shift1
#define MOTOR1A 7
#define MOTOR1B 6
#define MOTOR1C 5
#define MOTOR1D 4

//RIGHT
#define MOTOR2PORT shift1
#define MOTOR2A 3
#define MOTOR2B 2
#define MOTOR2C 1
#define MOTOR2D 0

//FRONT
#define MOTOR3PORT shift2
#define MOTOR3A 3
#define MOTOR3B 2
#define MOTOR3C 1
#define MOTOR3D 0

//BACK
#define MOTOR4PORT shift2
#define MOTOR4A 7
#define MOTOR4B 6
#define MOTOR4C 5
#define MOTOR4D 4

#define NUM_MOTORS 4

enum MotorState {MINIT, MTURN} motor_state;
unsigned char *ports[NUM_MOTORS] = {&MOTOR1PORT, &MOTOR2PORT, &MOTOR3PORT, &MOTOR4PORT};
unsigned char dir[NUM_MOTORS] = {1, 1, 1, 1}; //1 for forward, 0 for backward
double target_angle[NUM_MOTORS] = {0};
double curr_angle[NUM_MOTORS] = {0};
unsigned char cnt[NUM_MOTORS] = {1, 1, 1, 1};

#define PHASE .703125

unsigned char pins[][4] = {{MOTOR1A, MOTOR1B, MOTOR1C, MOTOR1D},
									{MOTOR2A, MOTOR2B, MOTOR2C, MOTOR2D},
									{MOTOR3A, MOTOR3B, MOTOR3C, MOTOR3D},
									{MOTOR4A, MOTOR4B, MOTOR4C, MOTOR4D},};
unsigned char pin_order[8][4] = {{1, 0, 0, 0},
								{1, 1, 0, 0},
								{0, 1, 0, 0},
								{0, 1, 1, 0},
								{0, 0, 1, 0},
								{0, 0, 1, 1},
								{0, 0, 0, 1},
								{1, 0, 0, 1}};

void Motor_Init() {
	motor_state = MINIT;
}

void Motor_Tick() {
	switch(motor_state) {
		case MINIT:
			break;
		case MTURN:
			for(unsigned char x = 0; x < NUM_MOTORS; x++) {
				target_angle[x] = fmod(target_angle[x], 720), curr_angle[x] = fmod(curr_angle[x], 720);
				if(target_angle[x] < 0) target_angle[x] += 720;
				if(curr_angle[x] < 0) curr_angle[x] += 720;
				
				if(target_angle[x] == curr_angle[x]) continue;
				
				for(unsigned y = 0; y < 4; y++) {
					if(pin_order[cnt[x]][y]) *ports[x] |= 1 << pins[x][y];
					else *ports[x] &= ~(1 << pins[x][y]);
				}
				
				if(dir[x] && ++cnt[x] > 7) { cnt[x] = 0; curr_angle[x] += PHASE; }
				else if(!dir[x] && !((cnt[x])--)) { cnt[x] = 7; curr_angle[x] -= PHASE; }
			}
			break;
	}
	
	switch(motor_state) {
		case MINIT:
			motor_state = MTURN;
			break;
		case MTURN:
			motor_state = MTURN;
			break;
		default:
			break;
	}
}

void Motor_Task() {
	Motor_Init();
	for(;;) {
		Motor_Tick();
		vTaskDelay(2);
	}
}

#define SOL_BUS PORTD
#define L_SOL 7
#define R_SOL 6
#define F_SOL 5
#define B_SOL 4

enum Moves {R, Rp, R2, L, Lp, L2, F, Fp, F2, B, Bp, B2}; //Everything but up and down
enum MoveState {MOINIT, MWAIT, MMOVE, MPULLBACK, MCORRECT} move_state;
Queue moves;
double *curr, *target;
unsigned char mcount = 0, sol = R_SOL, last_move = 0;
unsigned char num_to_execute = 0;

void Move_Init() {
	move_state = MOINIT;
}

void Move_Tick() {
	switch(move_state) {
		case MOINIT:
			break;
		case MWAIT:
			SOL_BUS &= ~(1 << sol);
			break;
		case MMOVE:
			break;
		case MPULLBACK:
			SOL_BUS |= (1 << sol);
			break;
		case MCORRECT:
			break;
	}
	
	switch(move_state) {
		case MOINIT:
			move_state = MWAIT;
			break;
		case MWAIT:
			if(!num_to_execute) {move_state = MWAIT; break;}
			move_state = MMOVE;
			last_move = QueueDequeue(moves); num_to_execute--;
			if(last_move == R) { 
				target_angle[1] += 90; dir[1] = 1; 
				target = &target_angle[1], curr = &curr_angle[1], sol = R_SOL;
			} else if(last_move == Rp){
				target_angle[1] -= 90; dir[1] = 0;
				target = &target_angle[1], curr = &curr_angle[1], sol = R_SOL;
			} else if(last_move == R2){
				target_angle[1] += 180; dir[1] = 1;
				target = &target_angle[1], curr = &curr_angle[1], sol = R_SOL;
			} else if(last_move == L) {
				target_angle[0] += 90; dir[0] = 1;
				target = &target_angle[0], curr = &curr_angle[0], sol = L_SOL;
			} else if(last_move == Lp){
				target_angle[0] -= 90; dir[0] = 0;
				target = &target_angle[0], curr = &curr_angle[0], sol = L_SOL;
			}  else if(last_move == L2){
				target_angle[0] += 180; dir[0] = 1;
				target = &target_angle[0], curr = &curr_angle[0], sol = L_SOL;
			} else if(last_move == F) {
				target_angle[2] += 90; dir[2] = 1;
				target = &target_angle[2], curr = &curr_angle[2], sol = F_SOL;
			} else if(last_move == Fp){
				target_angle[2] -= 90; dir[2] = 0;
				target = &target_angle[2], curr = &curr_angle[2], sol = F_SOL;
			}  else if(last_move == F2){
				target_angle[2] += 180; dir[2] = 1;
				target = &target_angle[2], curr = &curr_angle[2], sol = F_SOL;
			}  else if(last_move == B) {
				target_angle[3] += 90; dir[3] = 1;
				target = &target_angle[3], curr = &curr_angle[3], sol = B_SOL;
			} else if(last_move == Bp){
				target_angle[3] -= 90; dir[3] = 0;
				target = &target_angle[3], curr = &curr_angle[3], sol = B_SOL;
			}  else if(last_move == B2){
				target_angle[3] += 180; dir[3] = 1;
				target = &target_angle[3], curr = &curr_angle[3], sol = B_SOL;
			}
			
			break;
		case MMOVE:
			if(*target == *curr) { 
				//If we do a double turn, no need to engage the solenoid to correct the stepper
				if(last_move == R2 || last_move == L2 || last_move == F2 || last_move == B2) move_state = MWAIT;
				else { move_state = MPULLBACK; mcount = 0; }
			}
			break;
		case MPULLBACK:
			if(mcount++ > 50) {
				move_state = MCORRECT;
				//Do the reverse to put things back in order
				if(last_move == R) {
					target_angle[1] -= 90; dir[1] = 0;
					target = &target_angle[1], curr = &curr_angle[1], sol = R_SOL;
				} else if(last_move == Rp){
					target_angle[1] += 90; dir[1] = 1;
					target = &target_angle[1], curr = &curr_angle[1], sol = R_SOL;
				} else if(last_move == L) {
					target_angle[0] -= 90; dir[0] = 0;
					target = &target_angle[0], curr = &curr_angle[0], sol = L_SOL;
				} else if(last_move == Lp){
					target_angle[0] += 90; dir[0] = 1;
					target = &target_angle[0], curr = &curr_angle[0], sol = L_SOL;
				} else if(last_move == F) {
					target_angle[2] -= 90; dir[2] = 0;
					target= &target_angle[2], curr = &curr_angle[2], sol = F_SOL;
				} else if(last_move == Fp){
					target_angle[2] += 90; dir[2] = 1;
					target = &target_angle[2], curr = &curr_angle[2], sol = F_SOL;
				} else if(last_move == B) {
					target_angle[3] -= 90; dir[3] = 0;
					target =&target_angle[3], curr = &curr_angle[3], sol = B_SOL;
				} else if(last_move == Bp){
					target_angle[3] += 90; dir[3] = 1;
					target = &target_angle[3], curr = &curr_angle[3], sol = B_SOL;
			}		
			}
			break;
		case MCORRECT:
			if(*target == *curr) move_state = MWAIT;
			break;
		default:
			move_state = MOINIT;
	}
}

void Move_Task() {
	Move_Init();
	for(;;) {
		Move_Tick();
		vTaskDelay(10);
	}
}

enum JoyState {JINIT, JWAIT, JPUSHED} joy_state;
unsigned char x = 0;
unsigned char y = 0;

void Joy_Init() {
	joy_state = JINIT;
}

void Joy_Tick() {
	switch(joy_state) {
		case JINIT:
			break;
		case JWAIT:
			break;
		case JPUSHED:
			break;
	}
	
	switch(joy_state) {
		case JINIT:
			joy_state = JWAIT;
			break;
		case JWAIT:
			joystickSample();
			//PORTD = joyPos;
			if(joyPos == None) joy_state = JWAIT;
			else {
				if(joyPos == Right) { PORTD |= 0x00; QueueEnqueue(moves, B); } //target_angle[1] += 90, dir[1] = 1;
				else if(joyPos == Left) QueueEnqueue(moves, Bp); //target_angle[1] -= 90, dir[1] = 0;
				else if(joyPos == Down) QueueEnqueue(moves, B2);
				else if(joyPos == Up) QueueEnqueue(moves, B2);
				joy_state = JPUSHED;
			}
			break;
		case JPUSHED:
			joystickSample();
			if(joyPos == None) joy_state = JWAIT;
			else joy_state = JPUSHED;
			break;
			default:
			joy_state = JINIT;
			break;
	}
}

void Joy_Task() {
	Joy_Init();
	for(;;) {
		Joy_Tick();
		vTaskDelay(20);
	}
}

enum ReveiveState {RINIT, RWAIT} receive_state;

void Receive_Init() {
	SPI_ServantInit();
	receive_state = RINIT;
}

void Reveiced_Tick() {
	switch(receive_state) {
		case RINIT:
			break;
		case RWAIT:
			if(~receivedData) {
				if(receivedData < 12) { //Make sure it's a valid move
					QueueEnqueue(moves, receivedData);
				}
				receivedData = 0xFF;
			} 
			break;
	}
	
	switch(receive_state) {
		case RINIT:
			receive_state = RWAIT;
			break;
		case RWAIT:
			break;
		default:
			receive_state = RINIT;
			break;
	}
}

void Receive_Task() {
	Receive_Init();
	for(;;) {
		Reveiced_Tick();
		vTaskDelay(10);
	}
}

#define BUTBUS PINA
#define BUTPIN 5
enum ButState {BINIT, BWAIT, BPRESSED, BPWAIT} but_state;

void But_Init() {
	but_state = BINIT;
}

//If the button is pressed, execute all moves currently in the queue
void But_Tick() {
	switch(but_state) {
		case BINIT:
			break;
		case BWAIT:
			break;
		case BPRESSED:
			num_to_execute = moves->num_objects;
			break;
		case BPWAIT:
			break;
	}
	
	switch(but_state) {
		case BINIT:
			but_state = BWAIT;
			break;
		case BWAIT:
			if(~BUTBUS & (1 << BUTPIN)) but_state = BPRESSED;
			break;
		case BPRESSED:
			but_state = BPWAIT;
			break;
		case BPWAIT:
			if(!(~BUTBUS & (1 << BUTPIN))) but_state = BWAIT;
			break;
	}
}

void But_Task() {
	But_Init();
	for(;;) {
		But_Tick();
		vTaskDelay(50);
	}
}

void StartSecPulse(unsigned portBASE_TYPE Priority) {
	xTaskCreate(Motor_Task, (signed portCHAR *)"Motorer", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
	xTaskCreate(Move_Task, (signed portCHAR *)"Mover", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
	xTaskCreate(Shift_Task, (signed portCHAR *)"Shifter", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
	xTaskCreate(Joy_Task, (signed portCHAR *)"JoySticker", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
	xTaskCreate(Receive_Task, (signed portCHAR *)"Receiver", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
	xTaskCreate(But_Task, (signed portCHAR *)"Buttoner", configMINIMAL_STACK_SIZE, NULL, Priority, NULL);
}



int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	
	
	moves = QueueInit(30);
	
	joystickInit();
	StartSecPulse(1);
	vTaskStartScheduler();
}