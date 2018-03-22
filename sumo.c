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
#include <string.h>
#include "reflectance.h"
#include "sharpdistance.h"
#include "timer-j.h"
#include "driver.h"
#include "ultrasonic.h"

#ifdef DEBUG
	#warning "Compiling with DEBUG"
	#include <stdio.h>
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

#define GO_COUNT 1000U // cycle pour avancer
#define ESCAPE_COUNT 8000U // cycle pour s'echaper
#define ESCAPE_COUNT_2 2000U // cycle pour echaper, deuxieme phase
#define QUART_TOUR_COUNT 8000U // cycle pour déplacement 45°
#define DEMI_TOUR_COUNT 16000U // cycle pour déplacement 90°
#define MOTOR_ON_AVANT 255 // valeur motor marche avant
#define MOTOR_ON_ARR -255 // valeur motor marche arr
#define MOTOR_OFF 0 // valeur motor en arrêt

uint8_t read = 0;
/* state structure to hold state machine */
struct state;
/* state_dt: state data to be used in state machine functions */
struct state_dt;
/* state_fn: type definition to hold state functions of state machine */
typedef void state_fn(struct state *);

/* Sensor struct: contains value of sensor reading */
struct sensor{
	uint8_t index;
	uint16_t value;
};

/* Direction struct: contains value of speed motors */
struct direction {
	int8_t speed1;
	int8_t speed2;
};

/* Enum pour signaler vers ou il faut aller lors de la maneuvre de escape 2
 * phase */
enum second_phase_escape {AVG, AVD, ARG, ARD, AVANT, ARRIERE};

/* struct state_dt contains state data:
 *	 array of 3 sonar range sensors
 *	 array of 2 ir range sensors
 *	 array of 4 ir line sensors
 *	 array of 4 ir line sensors
 *	 next direction [ motor1_speed, motor2_speed]
 *	 escape direction [ motor1_speed, motor2_speed]
 *	 sonar sensor values encoded in a enum
 *	 counter: variable to stick to a state for counter cycles
 */
struct state_dt{
	struct sensor sonars[3];
	struct sensor ir[2]; /* left and right IR range sensors */
	struct sensor line[4]; /* left front, right front, left back an right back IR line sensors */
	enum second_phase_escape escape; /* second phase escape state */
	uint16_t counter; /* cycles in state */
};

/* struct state contains:
 *	next state function to execute
 *	state->name
 *	state data
 */
struct state{
	state_fn * next;
	char name[20];
	struct state_dt state_data;
};

// State functions definitions

void go(struct state *state);
void search(struct state *state);
void search_wait(struct state *state);
void escape(struct state *state);
void escape_phase2(struct state *state);
void quart_tour_droite(struct state *state);
void quart_tour_gauche(struct state *state);
void demi_tour(struct state *state);

// Helper functions
void query_sensors(struct state * state);

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
	// Set name for debugging
	strcpy(state->name,"search");

	// capteurs de ligne stimulés
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}

	if(state->state_data.sonars[0].value) {
		state->state_data.counter=0;
		state->next=go;
		return;
		}

	// pas de sonar et opposant à gauche
	if (state->state_data.ir[0].value) {
		state->state_data.counter=0;
		state->next=quart_tour_gauche;
		return;
	}
	// pas de sonar et opposant à droite
	if (state->state_data.ir[1].value) {
		state->state_data.counter=0;
		state->next=quart_tour_droite;
		return;
	}

	/* Pas de détection de l'opposant.
	 * TODO: On avance? */
	driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
	// No counter reset, if we get here, we stick to this state
	state->next=search;
}

void demi_tour(struct state *state)
{
	// Set name for debugging
	strcpy(state->name,"demi_tour");
	driver_move(MOTOR_ON_AVANT,MOTOR_ON_ARR);
	state->state_data.counter++;
	state->next=demi_tour;

	// capteurs de ligne stimulés
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}

	// Opposant supposé devant, on avance
	if(state->state_data.counter > DEMI_TOUR_COUNT){
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
		state->state_data.counter=0;
		state->next=go;
		return;
	}
}

void quart_tour_droite(struct state *state)
{
	// Set name for debugging
	strcpy(state->name,"quart_tour_droite");
	driver_move(MOTOR_ON_AVANT,MOTOR_ON_ARR);
	state->state_data.counter++;
	state->next=quart_tour_droite;

	// capteurs de ligne stimulés
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}

	// Opposant supposé devant, on avance
	if(state->state_data.counter > QUART_TOUR_COUNT){
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
		state->state_data.counter=0;
		state->next=go;
		return;
	}
}

void quart_tour_gauche(struct state *state)
{
	// Set name for debugging
	strcpy(state->name,"quart_tour_gauche");
	driver_move(MOTOR_ON_ARR,MOTOR_ON_AVANT);
	state->state_data.counter++;
	state->next=quart_tour_gauche;

	// capteurs de ligne stimulés
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}

	// Opposant devant, on avance
	if(state->state_data.counter > QUART_TOUR_COUNT){
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
		state->state_data.counter=0;
		state->next=go;
		return;
	}
}

/* définition de l'état go
 *	On fait avancer le robot vers l'avant pendant xxx
 *	Si l'opposant est devant et on ne marche pas sur une ligne.
 *	Si l'opposant n'est pas nettement localiser, on bascule vers affiner.
 *	Si on marche sur une ligne, on bascule vers escape.
 */
void go(struct state * state)
{
	// Set name for debugging
	strcpy(state->name,"go");
	// Opposant devant, on avance
	driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);

	// Si l'on marche sur une ligne --> escape
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}

	// Pas de détection du sonar --> search
	if (!state->state_data.sonars[0].value &&
	    state->state_data.counter > GO_COUNT) {
			state->state_data.counter=0;
			state->next=search;
			return;
	}

	// Si on est arrivé ici, oposant devant et pas de ligne,
	// on continue dans cet état
	state->next=go;
}

/* query_sensors:
 *	Triggers next sonar measurement
 *	Updates sensors mesurements
 */
void query_sensors(struct state * state)
{
	/* trigger sonar measure */
	ultrasonic_trigger_next();
	/* Copy sonars distances to state*/
#ifdef DEBUG2
		if (state->state_data.sonars[i].value != ultrasonic_get(0)){
			sprintf(dbg_msg,"Sonar %d mesure: %d.\n",0,ultrasonic_get(0));
			serial_send_str(dbg_msg);
		}
#endif
	state->state_data.sonars[0].value = ultrasonic_get(0);
	/* Copy range ir distances to state */
	for(int i=0; i<2; i++){
#ifdef DEBUG
		if (state->state_data.ir[i].value != sharp_distance(state->state_data.ir[i].index)){
			sprintf(dbg_msg,"SHARP IR %d: old: %d, new: %d.\n",
				i, state->state_data.ir[i].value,
				sharp_distance(state->state_data.ir[i].index));
			serial_send_str(dbg_msg);
		}
#endif
		state->state_data.ir[i].value = sharp_distance(state->state_data.ir[i].index);
	}
	/* Copy line ir detection to state */
	for(int i=0; i<4; i++){
#ifdef DEBUG
		if (state->state_data.line[i].value != reflectance_is_line(state->state_data.line[i].index)){
			sprintf(dbg_msg,"LINE IR %d mesure: %d.\n",
				i,reflectance_is_line(state->state_data.line[i].index));
			serial_send_str(dbg_msg);
		}
#endif
		state->state_data.line[i].value = reflectance_is_line(state->state_data.line[i].index);
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
 */
void search_wait(struct state * state)
{
	strcpy(state->name,"search_wait");
	/* check PB4 for start button press */
	/* when button is pressed it is at low level */
	if (bit_is_clear(PINB,PB4) && ! read){
		start_wait();
		read=!read;
	}

	if(starting){
		state->next=search_wait;
	}
	else{ // Le combat commence !
		// Si pas de sonar et pas
		// à IR, demi tour.
		if(!state->state_data.sonars[0].value &&
		   !state->state_data.ir[0].value &&
		   !state->state_data.ir[1].value){
			state->next=demi_tour;
		} else { // Autrement il est devant ou à coté, search prends en charge.
			state->next=search;
		}
	}
}

/* escape:
 *	escape strategie is in two phases:
 *		First phase (this state) is to go in opposite direction of the
 *		line detected. This will be done for some state machine cycles.
 *		At the end of this phase, we switch to second phase:
 *		escape_phase2.
 *
 *		A harness at the end of the state is coded.
 *
 *		At the end of the whole escape manoeuvre we switch to search state.
 */
void escape(struct state * state)
{
	strcpy(state->name,"escape");
	/* increment counter */
	state->state_data.counter++;
	/* Ligne devant
	 * Premier phase, on recule.
	 */
	if (state->state_data.line[0].value || state->state_data.line[1].value){
		state->state_data.counter=0; // Tant qu'on est sur la ligne, on demarre pas la tempo
		driver_move(MOTOR_ON_ARR,MOTOR_ON_ARR);
		/* Prepare phase two direction */
		if (state->state_data.line[0].value && state->state_data.line[1].value) {
			state->state_data.escape=ARRIERE; // On va partir en reculant
		} else if (state->state_data.line[0].value){
			state->state_data.escape=ARG;
		} else {
			state->state_data.escape=ARD;
		}
		// Stay here until timeout
		state->next=escape;
	}
	/* Ligne derriere
	 * Premier phase, on avance.
	 */
	if (state->state_data.line[2].value || state->state_data.line[3].value){
		state->state_data.counter=0; // Tant qu'on est sur la ligne, on demarre pas la tempo
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
		/* Prepare phase two direction */
		if (state->state_data.line[2].value && state->state_data.line[3].value) {
			state->state_data.escape=AVANT; // On va partir en avançant
		} else if (state->state_data.line[2].value){
			state->state_data.escape=AVD;
		} else {
			state->state_data.escape=AVG;
		}
		// Stay here until timeout
		state->next=escape;
	}

	/* Stay in this state until first manoeuver ends */
	/* TODO(Jaume): set up a lookup table to limit counter */
	if ((state->state_data.counter) > ESCAPE_COUNT){
		state->state_data.counter=0;
		state->next=escape_phase2;
		return;
	}
}

void escape_phase2(struct state * state)
{
	strcpy(state->name,"escape_ph2");
	/* increment counter */
	state->state_data.counter++;
	// capteurs de ligne stimulés
	for(int i=0 ; i <4; i++){
		if(state->state_data.line[i].value) {
			state->state_data.counter=0;
			state->next=escape;
			return;
		}
	}
	/* Move motors with information set up by phase1 */
	switch(state->state_data.escape){
	case AVANT:
		// Il faut aller vers l'avant
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_AVANT);
		break;
	case ARRIERE:
		driver_move(MOTOR_ON_ARR,MOTOR_ON_ARR);
		break;
	case AVG:
		// Un peu vers la gauche
		driver_move(MOTOR_ON_ARR+50,MOTOR_ON_AVANT);
		break;
	case AVD:
		// Un peu vers la droite
		driver_move(MOTOR_ON_AVANT,MOTOR_ON_ARR+50);
		break;
	case ARG:
		// Un peu vers la gauche
		driver_move(MOTOR_ON_AVANT-150,MOTOR_ON_ARR);
		break;
	case ARD:
		// Un peu vers la droite
		driver_move(MOTOR_ON_ARR,MOTOR_ON_AVANT-150);
		break;
		/*
	default: // on sort d'ici, on ne sais pas qu'est-ce qu'on y fait d'ailleurs
		state->state_data.escape=NONE;
		state->state_data.counter=0;
		state->next=search;
		return;
		break;
		*/
	}

	/* set new state selon counter */
	/* TODO(Jaume): Set up a lookup table to use as counter limit */
	if (state->state_data.counter < ESCAPE_COUNT_2){
		state->next=escape_phase2;
	}
	else {
		/* End of 2phase manoeuvre*/
		state->state_data.counter=0;
		driver_move(MOTOR_OFF,MOTOR_OFF);
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
	ultrasonic_init();
	/* add sonar sensor */
	pin_t sonar_center = {&PORTB,&DDRB,PB2}; /* Sonar center */
	ultrasonic_add(&sonar_center);
	state->state_data.sonars[0].value=0;
	/* Init reflectance sensors */
	reflectance_init();
	/* add line sensors */
	state->state_data.line[0].index=reflectance_add_sensor(PC0); /* Front Left line sensor */
	state->state_data.line[1].index=reflectance_add_sensor(PC1); /* Front Right line sensor */
	state->state_data.line[2].index=reflectance_add_sensor(PC2); /* Back Left line sensor */
	state->state_data.line[3].index=reflectance_add_sensor(PC3); /* Back Right line sensor */
	state->state_data.line[0].value=0;
	state->state_data.line[1].value=0;
	state->state_data.line[2].value=0;
	state->state_data.line[3].value=0;
	/* Init IR range sensors */
	sharp_init();
	/* add IR range sensors */
	state->state_data.ir[0].index=sharp_add_sensor(PC4); /* Left IR rang sensor */
	state->state_data.ir[1].index=sharp_add_sensor(PC5); /* Right IR rang sensor */
	state->state_data.ir[0].value=0;
	state->state_data.ir[1].value=0;

	/* Init motor driver shield */
	driver_init();

	/* Init state counter */
	state->state_data.counter=0;

	/* Add initial delay */
	init_wait(5, &DDRB, &PORTB, PB4);
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
		/* Query sensors and store values in state variable) */
		query_sensors(&state);
#ifdef DEBUG2
		sprintf(dbg_msg,"State: %s, Counter: %u, Escape: %u.\n",
			state.name, state.state_data.counter, state.state_data.escape);
		serial_send_str(dbg_msg);
#endif
		/* Execute state function */
		state.next(&state);
	}
	return 0;
}
