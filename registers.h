
#ifndef REGISTERS_H
#define REGISTERS_H

#include <avr/io.h>

// Macros that all the registers given a port
#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port

#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)

// Data type that represents a pin in some port
typedef struct {
    volatile uint8_t *port;
    volatile uint8_t *dir;
    uint8_t pin;
} pin_t;

#endif

