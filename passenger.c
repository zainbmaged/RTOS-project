// passenger.c

#include "passenger.h"
#include "tm4c123gh6pm.h"

// Define the semaphore handles
xSemaphoreHandle xMutex;
xSemaphoreHandle xBinarySemaphoreUp;    // Passenger up semaphore
xSemaphoreHandle xBinarySemaphoreDown;  // Passenger down semaphore

void Motor_Control(int direction) {
    // Placeholder function to control motor
    // direction 1: up, direction 2: down, direction 0: stop
}

uint8_t ReadPin(uint32_t *port, uint8_t pin) {
    return (*port & pin) ? 1 : 0;
}

void PassengerUp(void *pvParameters) {
    xSemaphoreTake(xBinarySemaphoreUp, 0);
    for (;;) {
        xSemaphoreTake(xBinarySemaphoreUp, portMAX_DELAY);
        xSemaphoreTake(xMutex, portMAX_DELAY);

        // Check if the window lock is enabled
        if (ReadPin(&GPIO_PORTA_DATA_R, WINDOW_LOCK_PIN) == 0x01) {
            xSemaphoreGive(xMutex);
            continue;  // Skip if window lock is enabled
        }

        // One touch auto open/close detection
        bool autoMode = false;
        if (ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_UP_PIN) == 0x01) {
            vTaskDelay(200 / portTICK_RATE_MS);
            if (ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_UP_PIN) == 0x00) {
                autoMode = true;
            }
        }

        Motor_Control(1);  // Move window up

        NVIC_EN0_R &= ~(1 << 0);  /* Disable PORTA Interrupt IRQ0 */

        if (autoMode) {
            while (ReadPin(&GPIO_PORTF_DATA_R, TOP_LIMIT_SWITCH_PIN) == 0x00 &&
                   ReadPin(&GPIO_PORTA_DATA_R, JAM_DETECTION_PIN) == 0x00) {
                // Check for jam
                if (ReadPin(&GPIO_PORTA_DATA_R, JAM_DETECTION_PIN) == 0x01) {
                    Motor_Control(0);
                    vTaskDelay(500 / portTICK_RATE_MS);
                    Motor_Control(2);  // Move down to release jam
                    vTaskDelay(500 / portTICK_RATE_MS);
                    Motor_Control(0);
                    break;
                }
            }
        } else {
            while (ReadPin(&GPIO_PORTF_DATA_R, TOP_LIMIT_SWITCH_PIN) == 0x00 &&
                   ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_UP_PIN) == 0x01) {
            }
        }

        NVIC_EN0_R |= (1 << 0);  /* Enable PORTA Interrupt IRQ0 */

        Motor_Control(0);  // Stop motor

        xSemaphoreGive(xMutex);
    }
}

void PassengerDown(void *pvParameters) {
    xSemaphoreTake(xBinarySemaphoreDown, 0);
    for (;;) {
        xSemaphoreTake(xBinarySemaphoreDown, portMAX_DELAY);
        xSemaphoreTake(xMutex, portMAX_DELAY);

        // Check if the window lock is enabled
        if (ReadPin(&GPIO_PORTA_DATA_R, WINDOW_LOCK_PIN) == 0x01) {
            xSemaphoreGive(xMutex);
            continue;  // Skip if window lock is enabled
        }

        // One touch auto open/close detection
        bool autoMode = false;
        if (ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_DOWN_PIN) == 0x01) {
            vTaskDelay(200 / portTICK_RATE_MS);
            if (ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_DOWN_PIN) == 0x00) {
                autoMode = true;
            }
        }

        Motor_Control(2);  // Move window down

        NVIC_EN0_R &= ~(1 << 0);  /* Disable PORTA Interrupt IRQ0 */

        if (autoMode) {
            while (ReadPin(&GPIO_PORTF_DATA_R, BOTTOM_LIMIT_SWITCH_PIN) == 0x00) {
            }
        } else {
            while (ReadPin(&GPIO_PORTF_DATA_R, BOTTOM_LIMIT_SWITCH_PIN) == 0x00 &&
                   ReadPin(&GPIO_PORTA_DATA_R, PASSENGER_DOWN_PIN) == 0x01) {
            }
        }

        NVIC_EN0_R |= (1 << 0);  /* Enable PORTA Interrupt IRQ0 */

        Motor_Control(0);  // Stop motor

        xSemaphoreGive(xMutex);
    }
}

int main(void) {
    // Hardware Initialization
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, TOP_LIMIT_SWITCH_PIN | BOTTOM_LIMIT_SWITCH_PIN | JAM_DETECTION_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, PASSENGER_UP_PIN | PASSENGER_DOWN_PIN | WINDOW_LOCK_PIN);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);  // Example motor control pin

    // Create Mutex
    xMutex = xSemaphoreCreateMutex();

    // Create Binary Semaphores
    vSemaphoreCreateBinary(xBinarySemaphoreUp);
    vSemaphoreCreateBinary(xBinarySemaphoreDown);

    // Create Tasks
    xTaskCreate(PassengerUp, "PassengerUp", 128, NULL, 1, NULL);
    xTaskCreate(PassengerDown, "PassengerDown", 128, NULL, 1, NULL);

    // Start Scheduler
    vTaskStartScheduler();

    // Infinite Loop 
    for (;;) {
    }
}
