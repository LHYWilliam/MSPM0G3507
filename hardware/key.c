#include "key.h"
#include "delay.h"

uint8_t Key_read(Key* key) {
	if (key->mode ? 
		DL_GPIO_readPins(key->gpio, key->pins) == key->pins 
		: DL_GPIO_readPins(key->gpio, key->pins) == 0) {
		Delay_ms(10);
		if (key->mode ? 
			DL_GPIO_readPins(key->gpio, key->pins) == key->pins 
			: DL_GPIO_readPins(key->gpio, key->pins) == 0) {
			
			return KEYDOWN;
		}
	}
	
	return NOKEYDOWN;
}