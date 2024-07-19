#include "motor.h"

void Motor_set(Motor* motor, int16_t speed) {
	DL_TimerG_setCaptureCompareValue(motor->PWM, speed > 0 ? speed : -speed, motor->Index);
	
	if (motor->invert) {
		if (speed >= 0) {
			DL_GPIO_setPins(motor->IN1_gpio, motor->IN1_pins);
			DL_GPIO_clearPins(motor->IN2_gpio, motor->IN2_pins);
		} else {
			DL_GPIO_clearPins(motor->IN1_gpio, motor->IN1_pins);
			DL_GPIO_setPins(motor->IN2_gpio, motor->IN2_pins);
		}
	} else {
		if (speed >= 0) {
			DL_GPIO_clearPins(motor->IN1_gpio, motor->IN1_pins);
			DL_GPIO_setPins(motor->IN2_gpio, motor->IN2_pins);
		} else {
			DL_GPIO_setPins(motor->IN1_gpio, motor->IN1_pins);
			DL_GPIO_clearPins(motor->IN2_gpio, motor->IN2_pins);
		}
	}
}