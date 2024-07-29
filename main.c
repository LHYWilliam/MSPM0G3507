/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "OLED.h"
#include "serial.h"
#include "motor.h"
#include "delay.h"

Serial BluetoothSerial = {
	.uart = Bluetooth_INST,
};

Motor motorLeft = {
	.IN1_gpio = MotorIN_PORT,
	.IN1_pins = MotorIN_LeftIN1_PIN,
	.IN2_gpio = MotorIN_PORT,
	.IN2_pins = MotorIN_LeftIN2_PIN,
	.PWM = MotorPWM_INST,
	.Index = GPIO_MotorPWM_C0_IDX,
	.invert = ENABLE,
};

Motor motorRight = {
	.IN1_gpio = MotorIN_PORT,
	.IN1_pins = MotorIN_RightIN1_PIN,
	.IN2_gpio = MotorIN_PORT,
	.IN2_pins = MotorIN_RightIN2_PIN,
	.PWM = MotorPWM_INST,
	.Index = GPIO_MotorPWM_C1_IDX,
	.invert = DISABLE,
};

typedef enum {
    Stop,
    Advance,
    Turn,
    Round,
} ActionType;
ActionType action = Stop;
char *actionString[] = {"Stop", "Advance", "Turn", "Round"};

typedef enum {
    Forward,
    TurnLeft,
    TurnRight,
    TurnBack,
} DirectionType;
DirectionType direction = Forward;
char *directionString[] = {"Forward", "TurnLeft", "TurnRight", "TurnBack"};

uint16_t advanceBaseSpeed = 2048;
uint16_t turnBaseSpeed = 920;

int16_t AdvancediffSpeed = 0;
int16_t turnDiffSpeed = 0;

uint16_t turnTime = 0;
uint16_t turnBaseTime = 1000;
uint16_t turnTimer = DISABLE;

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

uint16_t ADCValue[3];
int16_t encoderLeft, encoderRight;

int16_t speedLeft, speedRight;
uint16_t infraredLeft, infraredCener, infraredRight;

int main(void)
{
    SYSCFG_DL_init();
	
	OLED_Init();
	Serial_init(&BluetoothSerial);
	
	NVIC_ClearPendingIRQ(Bluetooth_INST_INT_IRQN);
	NVIC_EnableIRQ(Bluetooth_INST_INT_IRQN);
	
	NVIC_ClearPendingIRQ(infraredADC_INST_INT_IRQN);
	NVIC_EnableIRQ(infraredADC_INST_INT_IRQN);
	
	NVIC_ClearPendingIRQ(Encoder_INT_IRQN);
	NVIC_EnableIRQ(Encoder_INT_IRQN);
	
	NVIC_ClearPendingIRQ(Timer_INST_INT_IRQN);
    NVIC_EnableIRQ(Timer_INST_INT_IRQN);
	
    while (1) {
		
    }
}

void Timer_INST_IRQHandler(void) {
    if(DL_TimerG_getPendingInterrupt(Timer_INST) == DL_TIMER_IIDX_ZERO) {
		DL_ADC12_startConversion(infraredADC_INST);
		
		speedLeft = encoderLeft;
		encoderLeft = 0;
		speedRight = encoderRight;
		encoderRight = 0;
		
    }
}

void Bluetooth_INST_IRQHandler(void) {
  if (DL_UART_Main_getPendingInterrupt(BluetoothSerial.uart) == DL_UART_IIDX_RX) {
        Serial_Praser(&BluetoothSerial);
        Serial_Handler(&BluetoothSerial);
  }
}

void infraredADC_INST_IRQHandler(void) {
	if (DL_ADC12_getPendingInterrupt(infraredADC_INST) == DL_ADC12_IIDX_MEM2_RESULT_LOADED) {
        infraredLeft = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredLeft);  
		infraredCener = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredCenter);
		infraredRight = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredRight);		
     }
}

void GROUP1_IRQHandler(void) {
    uint32_t INT_PIN = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN | Encoder_EncoderLeft2_PIN | Encoder_EncoderRight1_PIN | Encoder_EncoderRight2_PIN);
 
    if((INT_PIN & Encoder_EncoderLeft1_PIN) == Encoder_EncoderLeft1_PIN) {
        if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
				encoderLeft--;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) == 0) {
				encoderLeft++;
			}
		} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) == 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
				encoderLeft++;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) == 0) {
				encoderLeft--;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN);
    } else if((INT_PIN & Encoder_EncoderLeft2_PIN) == Encoder_EncoderLeft2_PIN) {
		if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
				encoderLeft++;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) == 0) {
				encoderLeft--;
			}
		} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) == 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
				encoderLeft--;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) == 0) {
				encoderLeft++;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft2_PIN);
    } else if((INT_PIN & Encoder_EncoderRight1_PIN) == Encoder_EncoderRight1_PIN) {
        if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
				encoderRight--;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) == 0) {
				encoderRight++;
			}
		} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) == 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
				encoderRight++;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) == 0) {
				encoderRight--;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight1_PIN);
    } else if((INT_PIN & Encoder_EncoderRight2_PIN) == Encoder_EncoderRight2_PIN) {
		if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
				encoderRight++;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) == 0) {
				encoderRight--;
			}
		} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) == 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
				encoderRight--;
			} else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) == 0) {
				encoderRight++;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight2_PIN);
    }
}


void Serial_Praser(Serial *serial) {
    serial->ByteData = Serial_readByte(serial);

    switch (serial->type) {
    case None:
        if (serial->ByteData == 0xFF) {
            serial->type = HexPack;
        } else {
            Serial_clear(serial);
        }
        break;

    case HexPack:
        if (serial->ByteData == 0xFE && serial->count == 3) {
            serial->RecieveFlag = SET;
        } else {
            serial->HexData[serial->count++] = serial->ByteData;
        }

        if (serial->count > 3) {
            Serial_clear(serial);
        }
        break;

    default:
        Serial_clear(serial);
        break;
    }
}

void Serial_Handler(Serial *serial) {
    if (serial->RecieveFlag == SET) {
        action = serial->HexData[0];

        switch (action) {
        case Stop:
            break;

        case Advance:
            AdvancediffSpeed =
                (int16_t)(serial->HexData[1] << 8 | serial->HexData[2]);
            break;

        case Turn:
            direction = (int16_t)(serial->HexData[1] << 8 | serial->HexData[2]);

            switch (direction) {
            case Forward:
                break;

            case TurnLeft:
                turnTime = turnBaseTime;
                turnDiffSpeed = -turnBaseSpeed;
                break;

            case TurnRight:
                turnTime = turnBaseTime;
                turnDiffSpeed = turnBaseSpeed;
                break;

            case TurnBack:
                turnTime = turnBaseTime * 2;
                turnDiffSpeed = turnBaseSpeed;
                break;
            }

            turnTimer = ENABLE;
            break;

        case Round:
            break;

        default:
            action = Stop;
            break;
        }

        Serial_clear(serial);
    }
}
