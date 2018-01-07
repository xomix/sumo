/*
 * sonar.h
 */
#ifndef SONAR_H_
#define SONAR_H_
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

void sonar_init(void);
int sonar_add_sensor(volatile uint8_t *ddr, volatile uint8_t *port, int pin);
void sonar_query(void);
int sonar_get_distance(int Tpin);

/*      Speed of sound in m/s */
#define SONAR_SOUND_SPEED 343UL

/*      MAX RANGE in meters is 1 meter. */
#define MAX_RANGE_METERS 1UL

/*      MIN RANGE in cm is 2 cms. */
#define MIN_RANGE_CM 2UL

/* Timer1 prescaler */
#define PRESCALER 64UL

/*
 * SONAR_MAX_RANGE_TICKS is time in ticks for 2 * MAX_RANGE_METERS meters distance
 *      Go and come back means 2 * MAX_RANGE_METERS in meters.
 *      Time elapsed for an object in range limit is 2 * MAX_RANGE_METERS / SONAR_SOUND_SPEED seconds
 *              ~ 5.831 ms
 *      We will use 64 as prescaler, which means 1 tick is 64 / F_CPU seconds
 *              = 0.004 ms (F_CPU = 16Mhz)
 *      Number of ticks max is then 2 * MAX_RANGE_METERS * F_CPU / SONAR_SOUND_SPEED / PRESCALER
 *              ~ 1458 ticks
 *      It helps filtering out objects further than 1 meter
 * #define SONAR_MAX_RANGE 1458
 */
#define SONAR_MAX_RANGE_TICKS ((2UL * (MAX_RANGE_METERS) * (F_CPU) / (SONAR_SOUND_SPEED) / (PRESCALER)) + 1 )
/*
 * SONAR_MIN_RANGE is time in ticks for 2 * MIN_RANGE_CM cm distance
 *      2 * MIN_RANGE_CM (go - come back)
 *      Time elapsed for an object in range limit is ( 2 * MIN_RANGE_CM / 100 ) / SONAR_SOUND_SPEED seconds
 *              ~ 5.831 ms
 *      We will use 64 as prescaler, which means 1 tick is 64 / F_CPU seconds
 *              = 0.004 ms (F_CPU = 16Mhz)
 *      Number of ticks min is then 2 * MIN_RANGE_CM * F_CPU / 100 / SONAR_SOUND_SPEED / 64
 *              ~ 29
 *      It helps filtering out objects nearer than 2 cm
 * #define SONAR_MIN_RANGE 29
 */
#define SONAR_MIN_RANGE_TICKS ((2UL * (MIN_RANGE_CM) * (F_CPU) / 100UL / (SONAR_SOUND_SPEED) / (PRESCALER)) - 1 )
/*
 * SONAR_TICKS_TO_CM
 *      1 tick ~ 0.004 ms
 *      343 m/s
 *      One tick is 64 / F_CPU seconds
 *      Speed of sound is 343 m/s
 *      Sound does twice the distance
 *      SONAR_TICKS_TO_CM is 343 * 100 * 64 / ( F_CPU * 2 )
 *              ~ 14.58 or 0.0686
 * #define SONAR_TICKS_TO_CM 14
 */
#define SONAR_TICKS_TO_CM ((F_CPU * 2UL) / ((SONAR_SOUND_SPEED) * 100UL * (PRESCALER)))
/* We use SONAR_TICKS_TO_CM in a division, so protect division by 0 error */
#if SONAR_TICKS_TO_CM == 0
#error SONAR_TICKS_TO_CM cannot be 0
#endif

#define MAX_SONAR_SENSORS 3

struct sonar_sensor {
	volatile uint8_t *port;
	int pin;
	volatile uint8_t pwidth;
};
#endif /* SONAR_H_ Include Guard */
