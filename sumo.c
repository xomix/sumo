/*
 * Sumo program:
 * 		Drives a Sumo Bot
 */

/********************************************/
/* pseudo code
 * 	initialise sensors
 * 	wait 60 secs
 * 	begin operation
 ******************************************/

/* test de modif*/

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include "sonar.h"
#include "reflectance.h"
#include "sharpdistance.h"
#include "timer.h"

#ifdef DEBUG
	#warning "Compiling with DEBUG"
	#include "serial.h"
char dbg_msg[50];
#endif

#ifndef F_CPU
#warning "Default F_CPU for arduino uno"
#define F_CPU 16000000UL
#endif
#ifndef BAUD
#warning "Default F_CPU for arduino uno"
#define BAUD 9600// define BAUD in makefile
#endif

uint8_t read = 0;
/* state structure to hold state machine */
struct state;
/* state_dt: state data to be used in state machine functions */
struct state_dt;
/* state_fn: type definition to hold state functions of state machine */
typedef void state_fn(struct state *);

/* Sensor struct: contains value of sensor reading */
struct sensor{
	uint16_t value;
};

/* struct state_dt contains:
 *	 array of 3 sonar range sensors
 *	 array of 2 ir range sensors
 *	 array of 4 ir line sensors
 */
struct state_dt{
	struct sensor sonars[3];
	struct sensor ir[2];
	struct sensor line[4];
};

/* struct state contains:
 *	next state function to execute
 *	state data
 */
struct state{
	state_fn * next;
	struct state_dt state_data;
};

/* search state function
 *	wanders over the arena
 *	looking for opponent
 *
 function state_sonar
 sonar left = 1 and sonar right = 1 -> function go
 sonar left = 0 and sonar right = 1 -> function move_little_right
 sonar left = 1 and sonar right = 0 -> function move_little_left
 sonar left = 0  and sonar right = 0 -> function search_activ
 
 search_activ:
 for 1 to 2
 (motor right : 20% / motor left : 10% during 0,5 sec
 state_sonar->
 motor right : 10% / motor left : 20% during 0,5 sec
 )
 state_sonar->
 motor left : 50% during 0,5 sec
 return 84
 
 
 move_litle_right:
 motor left 10% more than motor right during 0,5 sec
 state_sonar->
 
 move_litle_left
 motor right 10% more than motor left during 0,5 sec
 state_sonar->
 
 go:
 motor left and right : 50%
 state_sonar->
 
 
 */
void search(struct state * state)
{
	state->next=search;
}

/* query_sensors:
 *	Triggers next sonar measurement
 *	Updates sensors mesurements
 */
void query_sensors(struct state * state)
{
	/* trigger sonar measure */
	sonar_query();
	/* Copy sonars distances to state*/
	for(int i=0 ; i <3; i++){
		state->state_data.sonars[i].value = sonar_get_distance(i);
	}
	/* Copy range ir distances to state */
	for(int i=0; i<2; i++){
		state->state_data.ir[i].value = sharp_distance(i);
	}
	/* Copy line ir detection to state */
	for(int i=0; i<4; i++){
		state->state_data.line[i].value = reflectance_is_line(i);
		/* TODO(Jaume): if detection, set new state function here */
	}
}

/*analyse situation:
 *Savoir si en position 1, 2 ou 3
 *postion 1 : en face -> il faut foncer !
 *position 3 : tête bèche -> quart de tour puis fonce !
 *position 2 : oposés -> quart de tour puis quart de tour puis fonce!
 *
 *Deux options :
 * - soit essayer de faire ça très vite : pas besoin de chercher l'autre
 * - soit se dire que les autres vont faire ça : vite bouger pour après le chercher
 * - soit essayer de faire vite et quand même coder de la recherche
 */




/* search_wait:
 *	Initial state function to search oponnent without
 *	moving motors.
 *	When "starting" flag is unset, it will move to a
 *	real search state.
 *	In this state we want to trigger sonar range sensors
 *	IR range sensors are auto triggered.
 */
void search_wait(struct state * state)
{
	/* check PB4 for start button press */
	/* TODO(Jaume) */
	if (bit_is_set(PORTB,PB4) && ! read){
		start_wait();
		read=!read;
	}

	query_sensors(state);

	if(starting){
		state->next=search_wait;
	}
	else{
		state->next=search;
	}
}

/* init:
 *	Initialises sensors:
 *		sonar range
 *		ir range
 *		ir line
 *	Sets up start button timing
 *	Sets initial state machine function:
 *		search_wait
 *	Activates interrupts
 */
void init(struct state * state)
{
	/* Disable interrupts */
	cli();

#ifdef DEBUG
	/* Init serial port */
	serial_init();
#endif
	/* Init sonar */
	sonar_init();
	/* add sonar sensor */
	sonar_add_sensor(&DDRD, &PORTD, PD5); /* Sonar left */
	sonar_add_sensor(&DDRB, &PORTB, PB1); /* Sonar center */
	sonar_add_sensor(&DDRB, &PORTB, PB2); /* Sonar right */
	state->state_data.sonars[0].value=0;
	state->state_data.sonars[1].value=0;
	state->state_data.sonars[2].value=0;
	/* Init reflectance sensors */
	reflectance_init();
	/* add line sensors */
	reflectance_add_sensor(PC0); /* Front Left line sensor */
	reflectance_add_sensor(PC1); /* Front Right line sensor */
	reflectance_add_sensor(PC2); /* Back Left line sensor */
	reflectance_add_sensor(PC3); /* Back Right line sensor */
	state->state_data.line[0].value=0;
	state->state_data.line[1].value=0;
	state->state_data.line[2].value=0;
	state->state_data.line[3].value=0;
	/* Init IR range sensors */
	sharp_init();
	/* add IR range sensors */
	sharp_add_sensor(PC4); /* Left IR rang sensor */
	sharp_add_sensor(PC5); /* Right IR rang sensor */
	state->state_data.ir[0].value=0;
	state->state_data.ir[1].value=0;

	/* Add initial delay */
	init_wait(5U, &DDRB, &PORTB, PB4);
	/* Enable interrupts */
	sei();
	/* Set up initial state */
	state->next = search_wait;
}

/* Main program:
 *	Declares a state struct variable
 *	Executes state function forever
 *		(state function updates next state function)
 */
int main(void)
{
	/* State of state machine */
	struct state state;
	/* Initialize everything */
	init(&state);

	/* Main (infinite) loop */
	while (1) {
		/* Execute state function */
		state.next(&state);
	}
	return 0;
}
