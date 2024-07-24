#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

#define DISABLE 0
#define ENABLE  1

typedef struct {
	GPIO_Regs* IN1_gpio;
	uint32_t IN1_pins;
	
	GPIO_Regs* IN2_gpio;
	uint32_t IN2_pins;
	
	GPTIMER_Regs *PWM;
	DL_TIMER_CC_INDEX Index;
	
	uint8_t invert;
} Motor;

void Motor_set(Motor* motor, int16_t speed);

#endif