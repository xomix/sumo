#ifndef TIMER_H_
extern volatile uint8_t starting;
void init_wait(uint8_t duration, volatile uint8_t *ddr, volatile uint8_t *port, int pin);
void start_wait(void);
#endif /* TIMER_H_ guard ends */
