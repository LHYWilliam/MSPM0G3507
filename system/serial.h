#ifndef __SERIAL_H
#define __SERIAL_H

#include "ti_msp_dl_config.h"

#define RESET 0
#define SET   1

typedef enum {
    None,
    Byte,
    HexPack,
    StringPack,
} PackType;

typedef struct {
	UART_Regs* uart;
	
	uint8_t count;
    uint8_t RecieveFlag;

    PackType type;

    uint8_t ByteData;
    uint8_t HexData[32];
    char StringData[32];
} Serial;

void Serial_init(Serial* serial);

void Serial_writeByte(Serial* serial, uint8_t byte);
uint8_t Serial_readByte(Serial* serial);

void Searial_write(Serial* serial, uint8_t* bytes, uint16_t length);
void Searial_read(Serial* serial, uint8_t* bytes, uint16_t length);

void Serial_printf(Serial* serial, char *format, ...);

void Serial_clear(Serial *serial);


#endif