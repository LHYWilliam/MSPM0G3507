#ifndef __LED_H
#define __LED_H

#include "ti_msp_dl_config.h"

#define LOW  0
#define HIGH 1

typedef struct {
	GPIO_Regs* gpio;
	uint32_t pins;
	
	uint8_t mode;
} LED;

void LED_on(LED* led);
void LED_off(LED* led);
void LED_turn(LED* led);

#endif