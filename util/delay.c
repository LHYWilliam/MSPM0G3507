#include "delay.h"

volatile uint32_t globalms = 0;

void delay_ms(uint32_t ms) {
	globalms = ms;
	while(globalms);
}

void SysTick_Handler(void) {
	if (globalms) {
		globalms--;
	}
}