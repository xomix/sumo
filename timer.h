
#include <stdint.h>

// Possible values of the timer prescaler
#define TIMER0_PRESCALER_1       1
#define TIMER0_PRESCALER_8       2
#define TIMER0_PRESCALER_64      3
#define TIMER0_PRESCALER_256     4
#define TIMER0_PRESCALER_1024    5

#define TIMER1_PRESCALER_1       1
#define TIMER1_PRESCALER_8       2
#define TIMER1_PRESCALER_64      3
#define TIMER1_PRESCALER_256     4
#define TIMER1_PRESCALER_1024    5

#define TIMER2_PRESCALER_1       1
#define TIMER2_PRESCALER_8       2
#define TIMER2_PRESCALER_32      3
#define TIMER2_PRESCALER_64      4
#define TIMER2_PRESCALER_128     5
#define TIMER2_PRESCALER_256     6
#define TIMER2_PRESCALER_1024    7

#define TIMER1_PRESCALER_VAL(x)  \
    ((x == TIMER1_PRESCALER_1) ? 1 : \
        ((x == TIMER1_PRESCALER_8) ? 8 : \
            ((x == TIMER1_PRESCALER_64) ? 64 : \
                ((x == TIMER1_PRESCALER_256) ? 256 : 1024) \
            ) \
        ) \
    )

#define TIMER2_PRESCALER_VAL(x)  \
    ((x == TIMER2_PRESCALER_1) ? 1 : \
        ((x == TIMER2_PRESCALER_8) ? 8 : \
            ((x == TIMER2_PRESCALER_32) ? 32 : \
                ((x == TIMER2_PRESCALER_64) ? 64 : \
                    ((x == TIMER2_PRESCALER_128) ? 128 : \
                        ((x == TIMER2_PRESCALER_256) ? 256 : 1024) \
                    ) \
                ) \
            ) \
        ) \
    )

/* Wait a given number of seconds.
   Parameters:
     * seconds: the seconds to wait.
*/
void
timer_wait_seconds(uint8_t seconds);

