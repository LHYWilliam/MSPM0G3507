#include "delay.h"

volatile uint32_t globalms = 0;

void Delay_ms(uint32_t ms) {
	globalms = ms;
	while(globalms);
}

void SysTick_Handler(void) {
	if (globalms) {
		globalms--;
	}
}