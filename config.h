
#include "timer.h"

#ifndef CONFIG_H
#define CONFIG_H

// BATTERY MONITOR SUBSYSTEM

// ADC channel where the battery level voltage is connected
#define BATTERY_ADC_CHANNEL     5

// Port where the battery led is connected
#define BATTERY_LED_PORT        C

// Pin where the battery led is connected in BATTERY_LED_PORT
#define BATTERY_LED_PIN         0

// Threshold (in volts) to trigger a low battery alarm
#define BATTERY_LEVEL_THRESHOLD 9.6

// First resistor of the voltage divider
#define BATTERY_DIVIDER_R1      3300

// Second resistor of the voltage divider, where the ADC channel is connected
#define BATTERY_DIVIDER_R2      2000

// Reference voltage of the ADC
#define BATTERY_ADC_VREF        5


// BATTERY PROTECTION FLAG

// 1 if battery low protection on, meaning that everything will STOP if the
// battery is below the threshold.
// 0 to ignore low level battery.
#ifndef BATTERY_LOW_PROTECTION
    #define BATTERY_LOW_PROTECTION 0
#endif


// EDGE SENSORS SUBSYSTEM

// ADC channels used by the edge sensors
#define EDGE_CHANNEL0   3   // front-right
#define EDGE_CHANNEL1   2   // front-left
#define EDGE_CHANNEL2   1   // back-left
#define EDGE_CHANNEL3   4   // back-right

// Threshold for each sensor to separate black and white.
// If the sensor value is below the threshold, then the sensor is over WHITE
#define EDGE_THRESHOLD0 256
#define EDGE_THRESHOLD1 256
#define EDGE_THRESHOLD2 256
#define EDGE_THRESHOLD3 256


// OPERATION MODE SELECTION

// Define the ports and pins where the switches that gives the operation mode
// are connected.
// This configuration allows for two switches, so 4 operation modes.
#define MODES_PORT0    B
#define MODES_PIN0     5
#define MODES_PORT1    B
#define MODES_PIN1     4


// MOTOR SUBSYSTEM

// To configure the frequency of the PWM signal
#define MOTOR_PWM_PRESCALER TIMER0_PRESCALER_1

// Motor direction pins
#define MOTOR_00_PORT       D
#define MOTOR_00_PIN        3
#define MOTOR_01_PORT       MOTOR_00_PORT
#define MOTOR_01_PIN        2
#define MOTOR_10_PORT       MOTOR_00_PORT
#define MOTOR_10_PIN        1
#define MOTOR_11_PORT       MOTOR_00_PORT
#define MOTOR_11_PIN        0


// ULTRASONIC SUBSYSTEM

// Maximum distance to detect, in cm
#define ULTRASONIC_MAX_DISTANCE 90

// Pins used for ultrasonic trigger
#define ULTRASONIC_PORT0    D
#define ULTRASONIC_PIN0     4
#define ULTRASONIC_PORT1    D
#define ULTRASONIC_PIN1     7
#define ULTRASONIC_PORT2    B
#define ULTRASONIC_PIN2     1
#define ULTRASONIC_PORT3    B
#define ULTRASONIC_PIN3     2


// STARTING SWITCH

#define START_SWITCH_PORT   B
#define START_SWITCH_PIN    3

#endif

