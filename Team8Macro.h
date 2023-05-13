#ifndef TEAM8_MACRO_H
#define TEAM8_MACRO_H

#include <stdint.h>

// GPIO pins for window control
#define WINDOW_UP_DRIVER_PIN       GPIO_PIN_2   // GPIO pin for upward movement of driver-side window
#define WINDOW_DOWN_DRIVER_PIN     GPIO_PIN_3   // GPIO pin for downward movement of driver-side window
#define WINDOW_UP_PASSENGER_PIN    GPIO_PIN_6   // GPIO pin for upward movement of passenger-side window
#define WINDOW_DOWN_PASSENGER_PIN  GPIO_PIN_7   // GPIO pin for downward movement of passenger-side window

// GPIO pin for obstacle detection and On/Off control
#define OBSTACLE_PIN               GPIO_PIN_5   // GPIO pin for obstacle detection
#define ON_OFF_PIN                 GPIO_PIN_4   // GPIO pin for On/Off control

// GPIO ports for various functions
#define WINDOW_MOTOR_PORT         GPIO_PORTE_BASE  // GPIO port for window motor control
#define OBSTACLE_PORT             GPIO_PORTC_BASE  // GPIO port for obstacle detection
#define ON_OFF_PORT               GPIO_PORTF_BASE  // GPIO port for On/Off control
#define BUTTONS_PORT              GPIO_PORTA_BASE  // GPIO port for buttons
#define LIMIT_SWITCHES_PORT       GPIO_PORTB_BASE  // GPIO port for limit switches

// Macros for motor control
#define Stop_Motor                GPIOPinWrite(WINDOW_MOTOR_PORT, GPIO_PIN_1 | GPIO_PIN_2, 0x00)
#define Move_Up                   do { \
                                     GPIOPinWrite(WINDOW_MOTOR_PORT, GPIO_PIN_1, GPIO_PIN_1); \
                                     GPIOPinWrite(WINDOW_MOTOR_PORT, GPIO_PIN_2, 0x0); \
                                  } while(0)
#define Move_Down                 do { \
                                     GPIOPinWrite(WINDOW_MOTOR_PORT, GPIO_PIN_2, GPIO_PIN_2); \
                                     GPIOPinWrite(WINDOW_MOTOR_PORT, GPIO_PIN_1, 0x0); \
                                  } while(0)

#endif /* TEAM8_MACRO_H */
