#include "delay.h"

volatile uint32_t globalms = 0;

void Delay_ms(uint32_t ms) {
	SYSCFG_DL_SYSTICK_init();
	
	globalms = ms;
	while(globalms);
	
	SysTick->CTRL = 0;
}

void SysTick_Handler(void) {
	if (globalms) {
		globalms--;
	}
}