#include "motor.h"

void Motor_set(Motor* motor, int16_t speed) {
	uint32_t setMode = motor->invert ? 0 : 0xFFFFFFFF;
	DL_GPIO_writePinsVal(motor->IN1_gpio, motor->IN1_pins, speed > 0 ? setMode : ~setMode);
	DL_GPIO_writePinsVal(motor->IN2_gpio, motor->IN2_pins, speed > 0 ? ~setMode : setMode);
	
	DL_TimerA_setCaptureCompareValue(motor->PWM, speed > 0 ? speed : -speed, motor->Index);
}