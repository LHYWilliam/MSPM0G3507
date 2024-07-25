#include "delay.h"

void Delay_us(uint32_t us) {
	SysTick->LOAD = us * (32000000 / 1000000);
	SysTick->VAL = 0;
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
	
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
}

void Delay_ms(uint32_t ms) {
	while (ms--) {
		Delay_us(1000);
	}
}

void Delay_s(uint32_t s) {
	while (s--) {
		Delay_ms(1000);
	}
}
