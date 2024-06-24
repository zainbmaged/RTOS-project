#ifndef PASSENGER_H
#define PASSENGER_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

// Semaphores
extern SemaphoreHandle_t passengerUpSemaphore;
extern SemaphoreHandle_t passengerDownSemaphore;
extern SemaphoreHandle_t motorMutex;

// Queues
extern QueueHandle_t driverQueue;
extern QueueHandle_t jamQueue;

// GPIO Macros 
#define Red_ON()        // Macro to turn on red LED
#define White_OFF()     // Macro to turn off white LED
#define Blue_ON()        // Macro to turn on blue LED (optional for jam indication)
#define MOTOR_ROTATE(dir) // Macro to control motor direction (FORWARD/BACKWARD)
#define MOTOR_STOP()     // Macro to stop the motor
#define Delay_Ms(ms)     // Macro to delay for milliseconds

// Limits and Flags
extern bool passengerUp;
extern bool passengerDown;
extern bool uLimit;  // Upper floor limit
extern bool dLimit;  // Lower floor limit
extern bool Lock;    // Safety lock flag

#endif /* PASSENGER_H */
