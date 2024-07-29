#ifndef __MPUI2C_H
#define __MPUI2C_H

#include "ti_msp_dl_config.h"

#define SDA_OUT()   {                                                  \
                        DL_GPIO_initDigitalOutput(MPU_MPUSDA_IOMUX);     \
                        DL_GPIO_setPins(MPU_PORT, MPU_MPUSDA_PIN);      \
                        DL_GPIO_enableOutput(MPU_PORT, MPU_MPUSDA_PIN); \
                    }
#define SDA_IN()    { DL_GPIO_initDigitalInput(MPU_MPUSDA_IOMUX); }
#define SDA_GET()   ( ( ( DL_GPIO_readPins(MPU_PORT,MPU_MPUSDA_PIN) & MPU_MPUSDA_PIN ) > 0 ) ? 1 : 0 )

#define SDA(x)      ( (x) ? (DL_GPIO_setPins(MPU_PORT,MPU_MPUSDA_PIN)) : (DL_GPIO_clearPins(MPU_PORT,MPU_MPUSDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(MPU_PORT,MPU_MPUSCL_PIN)) : (DL_GPIO_clearPins(MPU_PORT,MPU_MPUSCL_PIN)) )

void MPUI2C_Init();

void MPUI2C_Start();
void MPUI2C_Stop();

void MPUI2C_SendByte(uint8_t dat);
uint8_t MPUI2C_ReceiveByte(void);

uint8_t MPUI2C_WaitAck();
void MPUI2C_Ack(unsigned char ack);

void MPUI2C_Delay();

uint8_t MPUI2C_Send(uint8_t addr,uint8_t regaddr,uint8_t num,uint8_t *regdata);
uint8_t MPUI2C_Receive(uint8_t addr, uint8_t regaddr,uint8_t num,uint8_t* Read);
#endif