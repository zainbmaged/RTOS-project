#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "TM4C123GH6PM.h"
#include "macros.h"

#define OFF  1
#define UP   2
#define DOWN 3



//Declaration of Semaphores, Mutex, Queue, & Task Handler
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xLockSemaphore;
xSemaphoreHandle xMutex;
xQueueHandle xQueue;
TaskHandle_t  DriverHandle;

//Declaration of Functions
void INTERRUPTSInit(void);
void timer0Init(void);
void timer0_Delay(int time);
void motorInit(void);
void limitInit(void);
void buttonsInit(void);

/*
Description:
- This task waits for semaphore from ISR
- Then, it checks whether the lock button is pressed or not
- Depending on the value, the priority of the driver task is changed
*/

void lock(void* pvParameters) {

   xSemaphoreTake(xLockSemaphore, 0);
    while (1) {
			//Take semaphore
        xSemaphoreTake(xLockSemaphore, portMAX_DELAY);
			
			//Check on Button State
				if (GET_BIT(GPIO_PORTA_DATA_R,3)==0){
					
					
					GPIO_PORTF_DATA_R |= (1<<1); 				// Turn RED LED ON for indication
					vTaskPrioritySet(DriverHandle,2);  	// Change Driver Task Priority to 2
				}
				else if(GET_BIT(GPIO_PORTA_DATA_R,3)==1)
				{
					GPIO_PORTF_DATA_R &= ~(1<<1); 		// Turn RED LED OFF for indication
					vTaskPrioritySet(DriverHandle,1);	// CHange Driver Task Priority to 1
				}
		}
	}

/*
Description:
- This task waits for semaphore from ISR
- Then, it turns the motor DOWN for 1sec
*/
void jamTask(void* pvParameters) {
    xSemaphoreTake(xBinarySemaphore, 0);
    while (1) {
        //Take semaphore
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

        //motor reverse
        GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
       
        timer0_Delay(1000); //Timer Delay Used to Unblock Task

				// Stop motor
			  GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
	
   }  
}


/*
Description:
- This task waits for queue from lower priority tasks
- When recieving Value, State of motor is determined
*/
void recieveQueue(void* pvParameters) {
		int Val;
		portBASE_TYPE xStatus;
    const portTickType xTicks=100/portTICK_RATE_MS;
		while(1)
		{
      xStatus=xQueueReceive(xQueue,&Val,xTicks);
		
		if(Val==OFF) //STOP
		{
				GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
		}
		else if(Val==UP) //MOVE UP
		{
				GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
		}
		else if(Val==DOWNN) //MOVE DOWN
		{
				GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R |= (1 << 2);
		}		
		
		}
}


/*
Description:
- This task takes mutex upon entry
- Polls on UP & DOWN PB
- If pressed less than 1sec, AUTOMATIC mode is used
- If pressed more than 1sec, MANUAL mode is used
*/
void driver(void* pvParameters){
		int Val;
	  portBASE_TYPE xStatus;
		while(1)
		{
      xSemaphoreTake(xMutex,portMAX_DELAY );
			// driver's up button is pressed
			if (GET_BIT(GPIO_PORTD_DATA_R,0)==0){ 
				Val=UP;
				xStatus = xQueueSendToBack(xQueue,&Val,0); //Send up in Queue
				vTaskDelay(400); 
				if (GET_BIT(GPIO_PORTD_DATA_R,0)==0) //still pressing then it is manual
					{
						while(GET_BIT(GPIO_PORTD_DATA_R,0)==0);//keep moving up 
					}
     
				else if (GET_BIT(GPIO_PORTD_DATA_R,0)==1) // button is released after a short time it will be automatic
				{   				

					//keep moving up until limit switch at portA pin 7 is pressed or up button is pressed again or down button is pressed
					while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,0)==0 | GET_BIT(GPIO_PORTD_DATA_R,1)==0)); 
				}
				Val=OFF;
				xStatus = xQueueSendToBack(xQueue,&Val,0);
				
			}
				// driver's DOWN button is pressed
				if (GET_BIT(GPIO_PORTD_DATA_R,1)==0){
					Val=DOWN;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
					vTaskDelay(400); 
					if (GET_BIT(GPIO_PORTD_DATA_R,1)==0) //still pressing then it is manual
					{

					while(GET_BIT(GPIO_PORTD_DATA_R,1)==0);//keep moving down 
					}
     
					else if (GET_BIT(GPIO_PORTD_DATA_R,1)==1) // button is released after a short time it will be automatic
					{   
						//keep moving up until limit switch at portD pin 6 is pressed or up button is pressed  or down button is pressed again
								while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,1)==0 | GET_BIT(GPIO_PORTD_DATA_R,0)==0)); 
					}
					Val=OFF;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
			
				}
			

				xSemaphoreGive(xMutex);
				timer0_Delay(200);
			
			
			
		}
	}

	
	/*
Description:
- This task takes mutex upon entry
- Polls on UP & DOWN PB
- If pressed less than 1sec, AUTOMATIC mode is used
- If pressed more than 1sec, MANUAL mode is used
*/
void passenger(void* pvParameters){
		int Val;
		portBASE_TYPE xStatus;
		while(1)
		{
			
					
			
		}
}


int main( void )
{
	//Initialization of Semaphores, Queues, Mutex, GPIO, & Tasks
	  xQueue = xQueueCreate(2,sizeof(int));
	  xMutex = xSemaphoreCreateMutex(); 
    INTERRUPTSInit();
		buttonsInit();
		limitInit();
		motorInit();
	  timer0Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(xBinarySemaphore);
		vSemaphoreCreateBinary(xLockSemaphore);
	if( xBinarySemaphore != NULL )
		{
			xTaskCreate( jamTask, "jamTask", 200, NULL, 5, NULL );
			xTaskCreate( recieveQueue, "recieveQueue", 200, NULL, 3, NULL );
			xTaskCreate( passenger, "passenger", 270, NULL, 1, NULL );
			xTaskCreate( driver, "driver", 270, NULL, 1, &DriverHandle );
			xTaskCreate( lock, "lock", 270, NULL, 4, NULL );
			vTaskStartScheduler();
		}
}


void GPIOA_Handler(void)
{
    //Pin A2 --> Jam Interrupt
    //Pin A3 --> Lock Interrupt

    //jamTask --> Waits on xBinarySemaphore
    //lockTask --> Waits on xLockSemaphore

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;     //Initialize the flag to FALSE.

    unsigned int InterruptFlags;
    InterruptFlags = GPIO_PORTA_RIS_R;      //The flag to check if either A2 or A3 is triggered.

    //Check whether the interrupt on A2 (Jam Interrupt) is triggered.
    //If triggered, give the binary semaphore from ISR, and set the "xHigherPriorityTaskWoken" flag TRUE iff a higher priority task than the currently executing one is woken.
    if ( InterruptFlags & 0x04 ) {
        xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
    }

    //Else, check whether the interrupt on A3 (Lock Interrupt) is triggered.
    //If triggered, give the lock semaphore from ISR, and set the "xHigherPriorityTaskWoken" flag TRUE iff a higher priority task than the currently executing one is woken.
    else if ( InterruptFlags & 0x08 ) {
        xSemaphoreGiveFromISR( xLockSemaphore, &xHigherPriorityTaskWoken );
    }

    GPIO_PORTA_ICR_R = InterruptFlags;    //Acknowledge the serviced flag (Clear Interrupt Flag).

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );     //Forcing a context switch within an ISR only if the "xHigherPriorityTaskWoken" flag is TRUE.
                                                           //IF a higher priority task than the currently executing one is woken, a context switch is forced.
    
}


/*
Description:
- Configure Timer0 

*/
void timer0Init(void)
{
	SYSCTL_RCGCTIMER_R |= 0x01;
	TIMER0_CTL_R=0x00;
	TIMER0_CFG_R=0x00;
	TIMER0_TAMR_R=0x02;
	TIMER0_CTL_R=0x03;
}


/*
Description:
- Use timer0 for delays

*/
void timer0_Delay(int time)
{
	TIMER0_CTL_R=0x00;
	TIMER0_TAILR_R=16000*time-1;
	TIMER0_ICR_R=0x01;
	TIMER0_CTL_R |=0x03;
	while((TIMER0_RIS_R & 0x01)==0);
}


/*
Description:
- Configure Lock & Jam button Pins at pins A3 & A2 respectively
- These pins will be used as interrupts

*/
void INTERRUPTSInit(void)
{
	
	
}


/*
Description:
- Configure Up & Down Pushbutton Pins at pins D0,D1,D2,D3

*/
void buttonsInit(void)
{
	
}


/*
Description:
- Configure Limit Switch Pins at pins D6 & A7

*/
void limitInit(void)
{
	
	
}



/*
Description:
- Configure Motor Pins IN1 & IN2 at pins F2 & F3

*/
void motorInit(void)
{
	
}
