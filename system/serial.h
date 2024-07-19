#ifndef __SERIAL_H
#define __SERIAL_H

#include "ti_msp_dl_config.h"

typedef struct {
	UART_Regs* uart;
} Serial;

void Serial_writeByte(Serial* serial, uint8_t byte);
uint8_t Serial_readByte(Serial* serial);

void Searial_write(Serial* serial, uint8_t* bytes, uint16_t length);
void Searial_read(Serial* serial, uint8_t* bytes, uint16_t length);

void Serial_printf(Serial* serial, char *format, ...);


#endif