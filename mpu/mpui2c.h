#ifndef __I2C_H
#define __I2C_H

#include <stm32f10x.h>

#include "gpio.h"

#define SDA_IN()                                                               \
    {                                                                          \
        GPIOB->CRH &= 0XFFFFFF0F;                                              \
        GPIOB->CRH |= 8 << 4;                                                  \
    }
#define SDA_OUT()                                                              \
    {                                                                          \
        GPIOB->CRH &= 0XFFFFFF0F;                                              \
        GPIOB->CRH |= 3 << 4;                                                  \
    }

#define I2C_SCL PBout(8)
#define I2C_SDA PBout(9)
#define I2C_READ_SDA PBin(9)

typedef struct {
    char SCL[4];
    char SDA[4];
} I2C;

void MPUI2C_Init();

void MPUI2C_Start();
void MPUI2C_Stop();

void MPUI2C_SendByte(uint8_t txd);
uint8_t MPUI2C_ReceiveByte(unsigned char ack);

uint8_t MPUI2C_WaitAck();
void MPUI2C_Ack();
void MPUI2C_NoAck();

void MPUI2C_Delay();

uint8_t MPUI2C_Send(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
uint8_t MPUI2C_Receive(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
#endif