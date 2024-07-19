#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"

#define LOW  0
#define HIGH 1

#define NOKEYDOWN  0
#define KEYDOWN 1

typedef struct {
	GPIO_Regs* gpio;
	uint32_t pins;
	
	uint8_t mode;
} Key;

uint8_t Key_read(Key* key);

#endif