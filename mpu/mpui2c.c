#include "mpui2c.h"
#include "delay.h"
#include "gpio.h"

void MPUI2C_Init() {
    GPIO SCL = {
        .GPIOxPiny = "B8",
        .Mode = GPIO_Mode_Out_PP,
    };
    GPIO_Init_(&SCL);

    GPIO SDA = {
        .GPIOxPiny = "B9",
        .Mode = GPIO_Mode_Out_PP,
    };
    GPIO_Init_(&SDA);

    GPIO_WriteBit(SCL.GPIOx, SCL.GPIO_Pin, (BitAction)1);
    GPIO_WriteBit(SDA.GPIOx, SDA.GPIO_Pin, (BitAction)1);
}

void MPUI2C_Start() {
    SDA_OUT();
    I2C_SDA = 1;
    I2C_SCL = 1;
    MPUI2C_Delay();
    I2C_SDA = 0;
    MPUI2C_Delay();
    I2C_SCL = 0;
}

void MPUI2C_Stop() {
    SDA_OUT();
    I2C_SCL = 0;
    I2C_SDA = 0;
    MPUI2C_Delay();
    I2C_SCL = 1;
    I2C_SDA = 1;
    MPUI2C_Delay();
}

uint8_t MPUI2C_WaitAck() {
    uint8_t ucErrTime = 0;
    SDA_IN();
    I2C_SDA = 1;
    MPUI2C_Delay();
    I2C_SCL = 1;
    MPUI2C_Delay();
    while (I2C_READ_SDA) {
        ucErrTime++;
        if (ucErrTime > 250) {
            MPUI2C_Stop();
            return 1;
        }
    }
    I2C_SCL = 0;

    return 0;
}

void MPUI2C_Ack() {
    I2C_SCL = 0;
    SDA_OUT();
    I2C_SDA = 0;
    MPUI2C_Delay();
    I2C_SCL = 1;
    MPUI2C_Delay();
    I2C_SCL = 0;
}

void MPUI2C_NoAck() {
    I2C_SCL = 0;
    SDA_OUT();
    I2C_SDA = 1;
    MPUI2C_Delay();
    I2C_SCL = 1;
    MPUI2C_Delay();
    I2C_SCL = 0;
}

void MPUI2C_SendByte(uint8_t txd) {
    uint8_t t;
    SDA_OUT();
    I2C_SCL = 0;
    for (t = 0; t < 8; t++) {
        I2C_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        I2C_SCL = 1;
        MPUI2C_Delay();
        I2C_SCL = 0;
        MPUI2C_Delay();
    }
}

uint8_t MPUI2C_ReceiveByte(unsigned char ack) {
    unsigned char i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++) {
        I2C_SCL = 0;
        MPUI2C_Delay();
        I2C_SCL = 1;
        receive <<= 1;
        if (I2C_READ_SDA)
            receive++;
        MPUI2C_Delay();
    }
    if (!ack)
        MPUI2C_NoAck();
    else
        MPUI2C_Ack();
    return receive;
}

void MPUI2C_Delay() { Delay_us(2); }

uint8_t MPUI2C_Send(uint8_t addr, uint8_t reg, const uint8_t *buf,
                    uint8_t len) {
    uint8_t i;
    MPUI2C_Start();
    MPUI2C_SendByte((addr << 1) | 0);
    if (MPUI2C_WaitAck()) {
        MPUI2C_Stop();
        return 1;
    }
    MPUI2C_SendByte(reg);
    MPUI2C_WaitAck();
    for (i = 0; i < len; i++) {
        MPUI2C_SendByte(buf[i]);
        if (MPUI2C_WaitAck()) {
            MPUI2C_Stop();
            return 1;
        }
    }
    MPUI2C_Stop();
    return 0;
}

uint8_t MPUI2C_Receive(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    MPUI2C_Start();
    MPUI2C_SendByte((addr << 1) | 0);
    if (MPUI2C_WaitAck()) {
        MPUI2C_Stop();
        return 1;
    }
    MPUI2C_SendByte(reg);
    MPUI2C_WaitAck();
    MPUI2C_Start();
    MPUI2C_SendByte((addr << 1) | 1);
    MPUI2C_WaitAck();
    while (len) {
        if (len == 1)
            *buf = MPUI2C_ReceiveByte(0);
        else
            *buf = MPUI2C_ReceiveByte(1);
        len--;
        buf++;
    }
    MPUI2C_Stop();
    return 0;
}