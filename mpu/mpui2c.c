#include "mpui2c.h"
#include "delay.h"

void MPUI2C_Init() {
   
}

void MPUI2C_Start() {
     SDA_OUT();
     SCL(1);
     SDA(0);

     SDA(1);
     Delay_us(5);
     SDA(0);
     Delay_us(5);

     SCL(0);
}

void MPUI2C_Stop() {
     SDA_OUT();
     SCL(0);
     SDA(0);

     SCL(1);
     Delay_us(5);
     SDA(1);
     Delay_us(5);
}

uint8_t MPUI2C_WaitAck() {
    char ack = 0;
	unsigned char ack_flag = 10;
	SCL(0);
	SDA(1);
	SDA_IN();

	SCL(1);
	while((SDA_GET()==1) && ( ack_flag )) {
		ack_flag--;
		Delay_us(5);
	}

	if( ack_flag <= 0 ) {
		MPUI2C_Stop();
		return 1;
	} else {
		SCL(0);
		SDA_OUT();
	}
	return ack;
}

void MPUI2C_Ack(unsigned char ack) {
	SDA_OUT();
	SCL(0);
	SDA(0);
	Delay_us(5);
	if(!ack) SDA(0);
	else     SDA(1);
	SCL(1);
	Delay_us(5);
	SCL(0);
	SDA(1);
}

void MPUI2C_SendByte(uint8_t dat) {
	int i = 0;
	SDA_OUT();
	SCL(0);

	for( i = 0; i < 8; i++ ) {
		SDA( (dat & 0x80) >> 7 );
		Delay_us(1);
		SCL(1);
		Delay_us(5);
		SCL(0);
		Delay_us(5);
		dat<<=1;
	}
}

uint8_t MPUI2C_ReceiveByte(void) {
    unsigned char i,receive=0;
    SDA_IN();
    for(i=0;i<8;i++ )
    {
        SCL(0);
        Delay_us(5);
        SCL(1);
        Delay_us(5);
        receive<<=1;
        if( SDA_GET() )
        {
            receive|=1;
        }
        Delay_us(5);
    }

    SCL(0);

    return receive;
}

uint8_t MPUI2C_Send(uint8_t addr,uint8_t regaddr,uint8_t num,uint8_t *regdata) {
    uint16_t i = 0;
    MPUI2C_Start();
    MPUI2C_SendByte((addr<<1)|0);
    if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return 1;}
    MPUI2C_SendByte(regaddr);
    if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return 2;}

    for(i=0;i<num;i++)
    {
        MPUI2C_SendByte(regdata[i]);
        if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return (3+i);}
    }

    MPUI2C_Stop();
    return 0;
}

uint8_t MPUI2C_Receive(uint8_t addr, uint8_t regaddr,uint8_t num,uint8_t* Read) {
  uint8_t i;
	MPUI2C_Start();
	MPUI2C_SendByte((addr<<1)|0);
	if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return 1;}
	MPUI2C_SendByte(regaddr);
	if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return 2;}

	MPUI2C_Start();
	MPUI2C_SendByte((addr<<1)|1);
	if( MPUI2C_WaitAck() == 1 ) {MPUI2C_Stop();return 3;}

	for(i=0;i<(num-1);i++){
			Read[i]=MPUI2C_ReceiveByte();
			MPUI2C_Ack(0);
	}
	Read[i]=MPUI2C_ReceiveByte();
	MPUI2C_Ack(1);
	MPUI2C_Stop();
	return 0;
}