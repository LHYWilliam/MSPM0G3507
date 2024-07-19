#include <stdio.h>
#include <stdarg.h>

#include "serial.h"

void Serial_writeByte(Serial* serial, uint8_t byte) {
	while(DL_UART_Main_isBusy(serial->uart));
	
	DL_UART_Main_transmitData(serial->uart, byte);
}

uint8_t Serial_readByte(Serial* serial) {
	return DL_UART_Main_receiveData(serial->uart);
}

void Searial_write(Serial* serial, uint8_t* bytes, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		Serial_writeByte(serial, bytes[i]);
	}
}

void Searial_read(Serial* serial, uint8_t* bytes, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		bytes[i] = Serial_readByte(serial);
	}
}

void Serial_printf(Serial* serial, char *format, ...) {
	char string[64];
    va_list arg;
    va_start(arg, format);
    vsprintf(string, format, arg);
    va_end(arg);
    for (uint8_t i = 0; string[i] != '\0'; i++) {
        Serial_writeByte(serial, string[i]);
    }
}