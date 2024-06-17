// passenger.h

#ifndef PASSENGER_H
#define PASSENGER_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

// Define Pins for Limit Switches and Push Buttons
#define TOP_LIMIT_SWITCH_PIN GPIO_PIN_0
#define BOTTOM_LIMIT_SWITCH_PIN GPIO_PIN_1
#define PASSENGER_UP_PIN GPIO_PIN_5
#define PASSENGER_DOWN_PIN GPIO_PIN_6
#define JAM_DETECTION_PIN GPIO_PIN_3
#define WINDOW_LOCK_PIN GPIO_PIN_7

extern xSemaphoreHandle xMutex;
extern xSemaphoreHandle xBinarySemaphoreUp;    // Passenger up semaphore
extern xSemaphoreHandle xBinarySemaphoreDown;  // Passenger down semaphore

void PassengerUp(void *pvParameters);
void PassengerDown(void *pvParameters);

void Motor_Control(int direction);
uint8_t ReadPin(uint32_t *port, uint8_t pin);

#endif // PASSENGER_H
