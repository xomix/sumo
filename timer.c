
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "timer.h"

// With this constants, the timer resets every 8 ms
#define TIMER_PRESCALER_SECONDS TIMER2_PRESCALER_1024
#define TIMER_COMPARE_SECONDS   124
#define TIMER_SECONDS_TICKS     (F_CPU \
    / TIMER2_PRESCALER_VAL(TIMER_PRESCALER_SECONDS) \
    / (TIMER_COMPARE_SECONDS + 1))

// Counts the number of interrupts
static volatile uint16_t ticks;

// Wait flag
static volatile uint8_t wait;

void
timer_wait_seconds(uint8_t seconds)
{
    ticks = (uint16_t)seconds * TIMER_SECONDS_TICKS;
    wait = 1;

    // Reset the counter
    TCNT2 = 0;

    // Compare value
    OCR2A = TIMER_COMPARE_SECONDS;

    // Configure Timer2 to count cycles of 8ms (compare mode)
    // TCCR2A
    // COM2A1 | COM2A0 | COM2B1 | COM2B0 | X | X | WGM21 | WGM20
    // 0        0        0        0        0   0   1       0
    // COM2A<1:0>, COM2B<1:0> = 0 (not used in this mode)
    // WGM2<1:0> = 10 (CTC mode)
    TCCR2A = 0x02;

    // TCCR2B
    // FOC2A | FOC2B | X | X | WGM22 | CS2<2:0>
    // 0       0       0   0   0       prescaler
    // FOC2A = 0, FOC2B = 0: not used in this mode
    // WGM22 = 0: CTC mode
    // CS2<2:0> = TIMER_PRESCALER: the timer prescaler constant
    TCCR2B = 0x07 & TIMER_PRESCALER_SECONDS;

    // TIMSK2
    // X | X | X | X | X | OCIEB | OCIEA | TOIE
    // 0   0   0   0   0   0       1       0
    // OCIEA = 1: Enable interrupt on compare with channel A
    TIMSK2 = 0x02;
    
    while (wait);

    // Stop the timer and disable interrupts
    TCCR2B = 0;
    TIMSK2 = 0;
}

ISR(TIMER2_COMPA_vect)
{
    ticks--;
    if (!ticks) {
        wait = 0;
    }
}

