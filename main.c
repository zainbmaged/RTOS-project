#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "GPIO.h"

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


void MOTOR_INIT(void){
	GPIO_init(PORTA, PIN2, DIGITAL, OUTPUT);
	GPIO_init(PORTA, PIN3, DIGITAL, OUTPUT);
}

void MOTOR_ROTATE(uint8_t direction){
	if (direction){
		GPIO_setPin(PORTA, PIN2);
		GPIO_clearPin(PORTA, PIN3);
	}
	else{
		GPIO_clearPin(PORTA, PIN2);
		GPIO_setPin(PORTA, PIN3);
	}
}
		
void MOTOR_STOP(void){
	GPIO_clearPin(PORTA, PIN2);
	GPIO_clearPin(PORTA, PIN3);
}
//---------------------------------------------------------------jam------------------------------------------
void jam(void *pvParameters){
	xSemaphoreTake(driverUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( jamSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		White_OFF();
		Green_ON();
		MOTOR_ROTATE(BACKWARD);
		Delay_Ms(200);
		
		MOTOR_STOP();
		White_OFF();
		xSemaphoreGive(motorMutex);
	}
}
//-------------------------------------------------------------------Driver Tasks--------------------------------------------------------
void DriverUp(void *pvParameters){
	xSemaphoreTake(driverUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( driverUpSemaphore, portMAX_DELAY );
		
		if(!driverUp && uLimit){
			char jamFlag = 1;
			
			start = xTaskGetTickCount();
			xSemaphoreTake( motorMutex, portMAX_DELAY); 
			while((!driverUp )&& uLimit){
				Red_ON();
				MOTOR_ROTATE(FORWARD);
				if(!uLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
			}
			
			xSemaphoreGive(motorMutex); 
			end = xTaskGetTickCount();
			if((end - start < 100)){
				while(uLimit && jamFlag){
					MOTOR_ROTATE(FORWARD);
					Blue_ON();
					xQueuePeek(jamQueue, &jamFlag, 0);
					if(!uLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
				}
			}
			char sendValue = 1;
			xQueueOverwrite(jamQueue, &sendValue);
			MOTOR_STOP();
			White_OFF();
			Delay_Ms(1); 
		}
	}
}


void DriverDown(void *pvParameters){
	xSemaphoreTake(driverDownSemaphore, 0);
	for(;;){
		xSemaphoreTake( driverDownSemaphore, portMAX_DELAY );
		
		if(!driverDown && dLimit){			
			start = xTaskGetTickCount();
			xSemaphoreTake( motorMutex, portMAX_DELAY); 
			while(!driverDown && dLimit){
				Red_ON();
				MOTOR_ROTATE(BACKWARD);
				if(!dLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
			}
			
			xSemaphoreGive(motorMutex); 
			end = xTaskGetTickCount();
			if((end - start < 100)){
				while(dLimit){
					MOTOR_ROTATE(BACKWARD);
					Blue_ON();
					if(!dLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
				}
			}
			MOTOR_STOP();
			White_OFF();
			Delay_Ms(1); 
		}
	}
}



//-----------------------------------------------------------------------------Passenger Tasks---------------------------------------------------
void PassengerUp(void *pvParameters){
	xSemaphoreTake(passengerUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( passengerUpSemaphore, portMAX_DELAY );
		
		if(!passengerUp && uLimit && Lock){
			char jamFlag = 1;
			char driverFlag = 1;
			
			start = xTaskGetTickCount();
			xSemaphoreTake( motorMutex, portMAX_DELAY);
			while(!passengerUp && uLimit && Lock){
				Red_ON();
				MOTOR_ROTATE(FORWARD);
					if(!uLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
			}
			
			xSemaphoreGive(motorMutex); 
			end = xTaskGetTickCount();
			if((end - start < 100)){
				while(uLimit && jamFlag && driverFlag && Lock){
					MOTOR_ROTATE(FORWARD);
					Blue_ON();
					xQueuePeek(driverQueue, &driverFlag, 0);
					if(!uLimit){
					
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


void PassengerDown(void *pvParameters){
	xSemaphoreTake(passengerDownSemaphore, 0);
	for(;;){
		xSemaphoreTake( passengerDownSemaphore, portMAX_DELAY );
		
		if(!passengerDown && dLimit && Lock){
			char jamFlag = 1;
			char driverFlag = 1;
			
			start = xTaskGetTickCount();
			xSemaphoreTake( motorMutex, portMAX_DELAY); 
			while(!passengerDown && dLimit && Lock){
				Red_ON();
				MOTOR_ROTATE(BACKWARD);
				if(!dLimit){
					
				  MOTOR_STOP();
					xSemaphoreGive(motorMutex); 
						break;
				}
			}
			
			xSemaphoreGive(motorMutex); 
			
			end = xTaskGetTickCount();
			if((end - start < 100)){
				while(dLimit && driverFlag && Lock){
					MOTOR_ROTATE(BACKWARD);
					Blue_ON();
					xQueuePeek(driverQueue, &driverFlag, 0);
					if(!dLimit){
					
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

//------------------------------------------------------------------------------Ports handlers----------------------------------------------------
void PortA_Init(void){
		// Motor on pins A2, A3
		MOTOR_INIT();
	
		// Window Lock on pin A4 - Limit Switches on A6, A7
		SYSCTL->RCGCGPIO |= 0x00000001;
		GPIOA->CR |= 0x10;
		GPIOA->DIR &= ~0x10;
		GPIOA->PUR |= 0x10;
		GPIOA->DEN |= 0x10;
	
		GPIOA->CR |= 0xC0;
		GPIOA->DIR &= ~0xC0;
		GPIOA->PUR |= 0xC0;
		GPIOA->DEN |= 0xC0;
	
		GPIOA->DATA = 0x00;
}



// Initialize the interrupt for Port D 
void PortD_Init(void) {
    SYSCTL->RCGCGPIO |= 0x00000008; 
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY; 
    GPIO_PORTD_CR_R |= 0xCC; 
    GPIO_PORTD_AMSEL_R = 0x00; 
    GPIO_PORTD_PCTL_R = 0x00000000; 
    GPIO_PORTD_DIR_R &= ~0xCC; 
    GPIO_PORTD_AFSEL_R = 0x00; 
    GPIO_PORTD_PUR_R |= 0xCC; 
    GPIO_PORTD_DEN_R |= 0xCC; 
    
    // Setup interrupts for Port D pins
    GPIO_PORTD_ICR_R = 0xCC; 
    GPIO_PORTD_IM_R |= 0xCC; 
    GPIO_PORTD_IS_R &= ~0xCC; 
    GPIO_PORTD_IBE_R &= ~0xCC; 
    GPIO_PORTD_IEV_R &= ~0xCC; 
    NVIC_EnableIRQ(PortD_IRQn);
    NVIC_PRI0_R = (NVIC_PRI0_R & 0x00FFFFFF) | 0xA0000000; 
}


void GPIOD_Handler(void) {
	
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    
    if (GPIO_PORTD_RIS_R & (1 << 2)) {
				xSemaphoreGiveFromISR( driverUpSemaphore, &xHigherPriorityTaskWoken );
				char driverFlag = 0;
				xQueueOverwriteFromISR(driverQueue, &driverFlag, &xHigherPriorityTaskWoken);
        GPIO_PORTD_ICR_R |= (1 << 2); 
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
   
    else if (GPIO_PORTD_RIS_R & (1 << 3)) {
				xSemaphoreGiveFromISR( driverDownSemaphore, &xHigherPriorityTaskWoken );
				char driverFlag = 0;
				xQueueOverwriteFromISR(driverQueue, &driverFlag, &xHigherPriorityTaskWoken);
        GPIO_PORTD_ICR_R |= (1 << 3); 
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
    
    else if (GPIO_PORTD_RIS_R & (1 << 6)) {
				xSemaphoreGiveFromISR( passengerUpSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 6); 
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
    
    else if (GPIO_PORTD_RIS_R & (1 << 7)) {
				xSemaphoreGiveFromISR( passengerDownSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 7); 
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
}


//Initialize the interrupt for Port-F
void PortF_Init(void){
		SYSCTL->RCGCGPIO |= 0x00000020; 
		GPIOF->LOCK = 0x4C4F434B; 
		GPIOF->CR = 0x11; 
		GPIOF->AMSEL= 0x00;
		GPIOF->PCTL = 0x00000000; 
		GPIOF->DIR = 0x0E; 
		GPIOF->AFSEL = 0x00; 
		GPIOF->PUR = 0x11; 
		GPIOF->DEN = 0x11; 
		GPIOF->DATA = 0x00;
	
		
		GPIOF->ICR = 0x11; 
		GPIOF->IM |=0x11; 
		GPIOF->IS |= 0x11; 
		GPIOF->IEV &= ~0x11; 
		NVIC_EnableIRQ(PortF_IRQn); 
	  NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00600000;  
}





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
