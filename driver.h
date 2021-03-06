#ifndef DRIVER_H_ /* DRIVER_H_ header guard */
#define DRIVER_H_

void driver_init(void);
void driver_move_motor1(int16_t speed);
void driver_move_motor2(int16_t speed);
/* move both motors:
 *	speed sign means forward or backward
 *	speed value means dutty cycle (speed)
 */
void driver_move(int16_t speed1, int16_t speed2);

#endif /* DRIVER_H_ header guard */
