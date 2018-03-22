
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "config.h"
#include "timer.h"
#include "ultrasonic.h"

// Prescaler for the timer1 Input Capture (measure echo): 64 (3)
#define ULTRASONIC_ECHO_PRESCALER       TIMER1_PRESCALER_64
#define ULTRASONIC_ECHO_PRESCALER_VAL   \
    TIMER1_PRESCALER_VAL(ULTRASONIC_ECHO_PRESCALER)

// Length, in us, for the trigger pulse
#define ULTRASONIC_TRIGGER_PULSE_LENGTH 10

// Frequency of the ultrasonic triggering
#define ULTRASONIC_F 20
#define ULTRASONIC_CYCLE (F_CPU / ULTRASONIC_ECHO_PRESCALER_VAL / ULTRASONIC_F)

// Max number of ultrasonic sensors
#define ULTRASONIC_MAX_NUM 16

// Max distance to measure, discard echoes farther than this (in cm)
#define SOUND_SPEED_CM          34300
#define ULTRASONIC_MAX_TICKS    (2 * ULTRASONIC_MAX_DISTANCE * F_CPU \
    /(SOUND_SPEED_CM * ULTRASONIC_ECHO_PRESCALER_VAL))

// The pins used to trigger the ultrasonic sensors
static pin_t ultrasonic_trigger_pins[ULTRASONIC_MAX_NUM];

// Number of used pins
static uint8_t ultrasonic_num = 0;

// Range value given by the ultrasonic sensors
static volatile ultrasonic_val_t ultrasonic_range[ULTRASONIC_MAX_NUM];

// Variable to memorize the instant of the start of the pulse
static uint16_t pulse_start;

// Next ultrasonic sensor to trigger
static int8_t ultrasonic_current = -1;

/* Setup the ultrasonic sensors.
   Configure the IO pins to use and the timer1 to measure the echo pulse.
   */
void
ultrasonic_init()
{
    // Configure the echo pin as input
    DDRB &= ~(_BV(DDB0));

    // Activate Timer1 to measure the echo pulse (input capture mode)
    // TCCR1A
    // COM1 | COM1 | COM1 | COM1 | X | X | WGM11 | WGM10
    // 0      0      0      0      0   0   0       0
    // WGM1<1:0> = 00 (CTC/ICR1 mode)
    TCCR1A = 0x00;

    // TCCR1B
    // ICNC1 | ICES1 | X | WGM13 | WGM12 | CS12 | CS11 | CS10
    // 0       1       0   0       0       prescaler
    // ICNC1 = 0: Disable noise canceler.
    // ICES1 = 1: First detect rising edge (start of the echo pulse)
    // WGM1<3:2> = 00: Normal mode
    TCCR1B = 0x40 | ULTRASONIC_ECHO_PRESCALER;

    // Clear interrupts after setting the detecting edge (ICES1)
    TIFR1 |= _BV(ICF1);

    // TIMSK1
    // X | X | ICIE | X | X | OCIEB | OCIEA | TOIE
    // 0   0   1      0   0   0       0       0
    // Interrupt on input capture
    TIMSK1 = _BV(ICIE1);
}

// Add an ultrasonic sensor
void
ultrasonic_add(pin_t *pin)
{
    // Store the registers and pin where the ultrasonic sensor is
    ultrasonic_trigger_pins[ultrasonic_num] = *pin;
    // Configure the trigger pin as output, and write a 0
    *(pin->dir) |= _BV(pin->pin);
    *(pin->port) &= ~(_BV(pin->pin));
    // Initialize the range
    ultrasonic_range[ultrasonic_num] = 0;
    // Increment the number of sensors
    ultrasonic_num++;
}

// Trigger the next ultrasonic sensor.
void
ultrasonic_trigger_next()
{
    uint8_t pin;

    // Check that the minimum amount of time has elapsed
    if (TCNT1 > ULTRASONIC_CYCLE) {
        // Select the next sensor to trigger
        ultrasonic_current++;
        if (ultrasonic_current == ultrasonic_num)
            ultrasonic_current = 0;
        // Reset the timer1 counter to avoid overflow situations
        TCNT1 = 0;
        // Send a pulse of 10us
        pin = ultrasonic_trigger_pins[ultrasonic_current].pin;
        *ultrasonic_trigger_pins[ultrasonic_current].port |= _BV(pin);
        _delay_us(ULTRASONIC_TRIGGER_PULSE_LENGTH);
        *ultrasonic_trigger_pins[ultrasonic_current].port &= ~(_BV(pin));
    }
}

// Get the value of an ultrasonic sensor
ultrasonic_val_t
ultrasonic_get(uint8_t index)
{
    ultrasonic_val_t range;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        range = ultrasonic_range[index];
    }
    return range;
}

// Stop the ultrasonic sensors subsystem
void
ultrasonic_close()
{
    TCCR1B &= 0xf8;
    TIMSK1 = 0;
}

// Interrupt service routine
ISR(TIMER1_CAPT_vect)
{
    uint16_t pulse_length;

    // Identify if the pulse is starting or ending
    if (TCCR1B & _BV(ICES1)) {
        // Rising edge: start of the pulse
        // Store the moment of the rising edge
        pulse_start = ICR1;
        // Change the event to detect (falling edge)
        TCCR1B &= ~(_BV(ICES1));
        // Clear interrupt flag
        TIFR1 |= _BV(ICF1);
    } else {
        // Falling edge: end of the pulse
        pulse_length = ICR1 - pulse_start;
        // Value only valid if less than ULTRASONIC_MAX_TICKS
        ultrasonic_range[ultrasonic_current] = (
            pulse_length <= ULTRASONIC_MAX_TICKS) ? pulse_length : 0;
        // Change the input capture event to raising edge
        TCCR1B |= _BV(ICES1);
        // Clear interrupt flag
        TIFR1 |= _BV(ICF1);
    }
}

