#ifndef DRIVER_H_ /* DRIVER_H_ header guard */
#define DRIVER_H_

void motor_init(void);
void motor_move_motor1(int8_t speed);
void motor_move_motor2(int8_t speed);
/* move both motors:
 *	speed sign means forward or backward
 *	speed value means dutty cycle (speed)
 */
void motor_move(int8_t speed1, int8_t speed2);

#endif /* DRIVER_H_ header guard */
