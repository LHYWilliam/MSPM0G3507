#include "LED.h"

void LED_on(LED* led) {
	if (led->mode == HIGH){
		DL_GPIO_setPins(led->gpio, led->pins);
	} else {
		DL_GPIO_clearPins(led->gpio, led->pins);
	}
}

void LED_off(LED* led) {
	if (led->mode == HIGH){
		DL_GPIO_clearPins(led->gpio, led->pins);
	} else {
		DL_GPIO_setPins(led->gpio, led->pins);
	}
}

void LED_turn(LED* led) {
	DL_GPIO_togglePins(led->gpio, led->pins);
}