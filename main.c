#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "MOTOR.h"

#include "delay.h"

#define PortD_IRQn 3
#define PortF_IRQn 30
//driver
#define driverUp (GPIO_readPin(PORTD, PIN2))
#define driverDown (GPIO_readPin(PORTD, PIN3))
//passenger
#define passengerUp (GPIO_readPin(PORTD, PIN6))
#define passengerDown (GPIO_readPin(PORTD, PIN7))

//leds

#define RED_LED PIN1
#define BLUE_LED PIN2
#define GREEN_LED PIN3
//motor
#define FORWARD 1
#define BACKWARD 0
//lock
#define Lock (GPIO_readPin(PORTA, PIN4))
//limit
#define uLimit (GPIO_readPin(PORTA, PIN6))
#define dLimit (GPIO_readPin(PORTA, PIN7))
//handlers
QueueHandle_t driverQueue;
QueueHandle_t jamQueue;

xSemaphoreHandle driverUpSemaphore;
xSemaphoreHandle driverDownSemaphore;
xSemaphoreHandle passengerUpSemaphore;
xSemaphoreHandle passengerDownSemaphore;
xSemaphoreHandle jamSemaphore;

xSemaphoreHandle motorMutex;

volatile long start;
volatile long end;

//-------------------------------------------------------------------Led functions-----------------------------------------


void LED_INIT(void){
    GPIO_init(PORTF, PIN1, DIGITAL, OUTPUT);
    GPIO_init(PORTF, PIN2, DIGITAL, OUTPUT);
    GPIO_init(PORTF, PIN3, DIGITAL, OUTPUT);
}



void Red_ON(void){
    GPIO_setPin(PORTF, RED_LED);
}

void Blue_ON(void){
    GPIO_setPin(PORTF, BLUE_LED);
}

void Green_ON(void){
    GPIO_setPin(PORTF, GREEN_LED);
}

void White_ON(void){
    Red_ON();
    Blue_ON();
    Green_ON();
}


void Red_OFF(void){
    GPIO_clearPin(PORTF, RED_LED);
}

void Blue_OFF(void){
    GPIO_clearPin(PORTF, BLUE_LED);
}

void Green_OFF(void){
    GPIO_clearPin(PORTF, GREEN_LED);
}

void White_OFF(void){
    Red_OFF();
    Blue_OFF();
    Green_OFF();
}
//---------------------------------------------------------------------GPIO handler------------------------------------------------------------
void GPIOF_Handler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	// Give semaphore 
	xSemaphoreGiveFromISR( jamSemaphore, &xHigherPriorityTaskWoken );
	
	char jamFlag = 0;
	xQueueOverwriteFromISR(jamQueue, &jamFlag, &xHigherPriorityTaskWoken);
	
	// Clear the interrupt 
  GPIOF->ICR = 0x11;
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

//--------------------------------------------------------------------motor--------------------------------------------------------------------


//---------------------------------------------------------------jam------------------------------------------

//-------------------------------------------------------------------Driver Tasks--------------------------------------------------------


//-----------------------------------------------------------------------------Passenger Tasks---------------------------------------------------

//------------------------------------------------------------------------------Ports handlers----------------------------------------------------


int main(void) {
	
	PortA_Init();
	PortD_Init();
	PortF_Init();
	LED_INIT();
	
	char initValue = 1;
	driverQueue = xQueueCreate(1, sizeof(int));
	xQueueSendToBack(driverQueue, &initValue, 0);
	jamQueue = xQueueCreate(1, sizeof(int));
	xQueueSendToBack(jamQueue, &initValue, 0);
	
	
	driverUpSemaphore = xSemaphoreCreateBinary();
	driverDownSemaphore = xSemaphoreCreateBinary();
	passengerUpSemaphore = xSemaphoreCreateBinary();
	passengerDownSemaphore = xSemaphoreCreateBinary();
	jamSemaphore = xSemaphoreCreateBinary();
	
	motorMutex = xSemaphoreCreateMutex();

	xTaskCreate(DriverUp, "DriverUp", 40, NULL, 3, NULL);
	xTaskCreate(DriverDown, "DriverDown", 40, NULL, 3, NULL);
	xTaskCreate(PassengerUp, "PassengerUp", 40, NULL, 2, NULL);
	xTaskCreate(PassengerDown, "passengerDown", 40, NULL, 2, NULL);
	xTaskCreate(jam, "jam", 40, NULL, 4, NULL);

	vTaskStartScheduler();

	for(;;){
		;
	}
}
