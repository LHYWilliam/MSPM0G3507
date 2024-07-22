#include "LED.h"

void LED_on(LED* led) {
	DL_GPIO_writePinsVal(led->gpio, led->pins, led->mode ? 0xFFFFFFFF : 0);
}

void LED_off(LED* led) {
	DL_GPIO_writePinsVal(led->gpio, led->pins, led->mode ? 0 : 0xFFFFFFFF);
}

void LED_turn(LED* led) {
	DL_GPIO_writePinsVal(led->gpio, led->pins, 
						 ~DL_GPIO_readPins(led->gpio, led->pins));
}