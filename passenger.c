
#include "passenger.h"

static TickType_t start, end; // for timing calculations

void PassengerUp(void *pvParameters) {
    xSemaphoreTake(passengerUpSemaphore, portMAX_DELAY);

    while (1) {
        xSemaphoreTake(passengerUpSemaphore, portMAX_DELAY);

        if (!passengerUp && uLimit && Lock) {
            // Flag for jam and driver communication
            char jamFlag = 1;
            char driverFlag = 1;

            start = xTaskGetTickCount();
            xSemaphoreTake(motorMutex, portMAX_DELAY);

            while (!passengerUp && uLimit && Lock) {
                Red_ON();
                MOTOR_ROTATE(FORWARD);

                if (!uLimit) {
                    MOTOR_STOP();
                    xSemaphoreGive(motorMutex);
                    break;
                }
            }

            xSemaphoreGive(motorMutex);
            end = xTaskGetTickCount();

            // Check for short travel time and possible jam
            if ((end - start) < 100) {
                while (uLimit && jamFlag && driverFlag && Lock) {
                    MOTOR_ROTATE(FORWARD);
                    Blue_ON(); // Optional for jam indication
                    xQueuePeek(driverQueue, &driverFlag, 0);

                    if (!uLimit) {
                        MOTOR_STOP();
                        xSemaphoreGive(motorMutex);
                        break;
                    }
                }
            }

            // Signal passenger arrival and potential jam
            char sendValue = 1;
            xQueueOverwrite(driverQueue, &sendValue);
            xQueueOverwrite(jamQueue, &sendValue);

            MOTOR_STOP();
            White_OFF();
            Delay_Ms(1);
        }
    }
}

void PassengerDown(void *pvParameters) {
    xSemaphoreTake(passengerDownSemaphore, portMAX_DELAY);

    while (1) {
        xSemaphoreTake(passengerDownSemaphore, portMAX_DELAY);

        if (!passengerDown && dLimit && Lock) {
            char jamFlag = 1;
            char driverFlag = 1;

            start = xTaskGetTickCount();
            xSemaphoreTake(motorMutex, portMAX_DELAY);

            while (!passengerDown && dLimit && Lock) {
                Red_ON();
                MOTOR_ROTATE(BACKWARD);

                if (!dLimit) {
                    MOTOR_STOP();
                    xSemaphoreGive(motorMutex);
                    break;
                }
            }

            xSemaphoreGive(motorMutex);
            end = xTaskGetTickCount();

            if ((end - start) < 100) {
                while (dLimit && driverFlag && Lock) {
                    MOTOR_ROTATE(BACKWARD);
                    Blue_ON(); // Optional for jam indication
                    xQueuePeek(driverQueue, &driverFlag, 0);

                    if (!dLimit) {
                        MOTOR_STOP();
                        xSemaphoreGive(motorMutex);
                        break;
                    }
                }
            }

                        char sendValue = 1;
            xQueueOverwrite(driverQueue, &sendValue);
            xQueueOverwrite(jamQueue, &sendValue);

            MOTOR_STOP();
            White_OFF();
            Delay_Ms(1);
        }
    }
}
