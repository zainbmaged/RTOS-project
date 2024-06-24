#ifndef GPIO_H
#define GPIO_H

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


#define PORTA 'A'
#define PORTB 'B'
#define PORTC 'C'
#define PORTD 'D'
#define PORTE 'E'
#define PORTF 'F'

#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7

#define DIGITAL 1
#define ANALOG 0

#define UP 1
#define DOWN 0

#define INPUT 0
#define OUTPUT 1

void GPIO_init(uint8_t port, uint32_t pin, uint32_t dtype, uint32_t io);
void GPIO_setPin(uint8_t port, uint32_t pin);
void GPIO_clearPin(uint8_t port, uint32_t pin);
void GPIO_togglePin(uint8_t port, uint32_t pin);
uint8_t GPIO_readPin(uint8_t port, uint32_t pin);

void GPIO_writePort(uint8_t port, uint32_t value);
uint32_t GPIO_readPort(uint8_t port);


#endif //GPIO_H