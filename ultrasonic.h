
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#include "registers.h"

// Max number of ultrasonic sensors
#define ULTRASONIC_MAX 16

typedef uint16_t ultrasonic_val_t;

// Setup the ultrasonic sensors.
void
ultrasonic_init();

// Add an ultrasonic sensor
void
ultrasonic_add(pin_t *pin);

// Trigger the next ultrasonic sensor.
void
ultrasonic_trigger_next();

// Get the value of an ultrasonic sensor
ultrasonic_val_t
ultrasonic_get(uint8_t index);

// Stop the ultrasonic sensors subsystem
void
ultrasonic_close();

#endif

