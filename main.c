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
#include "pid.h"
#include "motor.h"
#include "delay.h"

uint32_t ms = 0;

Serial BluetoothSerial = {
	.uart = Bluetooth_INST,
};

Motor motorLeft = {
	.IN1_gpio = MotorIN_LeftIN1_PORT,
	.IN1_pins = MotorIN_LeftIN1_PIN,
	.IN2_gpio = MotorIN_LeftIN2_PORT,
	.IN2_pins = MotorIN_LeftIN2_PIN,
	.PWM = MotorPWM_INST,
	.Index = GPIO_MotorPWM_C0_IDX,
	.invert = ENABLE,
};

Motor motorRight = {
	.IN1_gpio = MotorIN_RightIN1_PORT,
	.IN1_pins = MotorIN_RightIN1_PIN,
	.IN2_gpio = MotorIN_RightIN2_PORT,
	.IN2_pins = MotorIN_RightIN2_PIN,
	.PWM = MotorPWM_INST,
	.Index = GPIO_MotorPWM_C1_IDX,
	.invert = DISABLE,
};

PID tracePID = {
    .Kp = -0,
    .Ki = 0,
    .Kd = -0,
    .imax = 1024,
};

PID motorLeftPID = {
    .Kp = -0,
    .Ki = -0,
    .Kd = 0,
    .imax = 2048,
};

PID motorRightPID = {
    .Kp = -0,
    .Ki = -0,
    .Kd = 0,
    .imax = 2048,
};

typedef enum {
    Left,
    Center,
    Right,
} InfraredType;

typedef enum {
    Stop,
    Advance,
    Turn,
    Round,
} ActionType;
ActionType action = Stop;
char *actionString[] = {"Stop", "Advance", "Turn", "Round"};

typedef enum {
    NoDirection = 0x0,
    Forward = 0x1,
    TurnLeft = 0x2,
    TurnRight = 0x4,
    TurnBack = 0x8,
} DirectionType;
DirectionType direction = TurnRight;
char *directionString[] = {"NoDirection", "Forward", "TurnLeft", "TurnRight",
                           "TurnBack"};

typedef enum {
    OffLine,
    OnLine,
    OnCross,
} LineType;
LineType lineState = OffLine;
char *lineString[] = {"OffLine", "OnLine", "OnCross"};

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

float encoderToPWM = 7200. / 55.;
uint16_t infraredMax = 3850, infraredMaxCenter = 2500;
uint16_t offLineInfrared = 1024, onLineInfrared = 3700, onCrossInfrared = 3800;
uint16_t advanceBaseSpeed = 1024, turnBaseSpeed = 390, turnBaseTime = 1000,
         turnAdvanceSpeed = 3072, roundSpeed = 390;

int16_t AdvancediffSpeed, turnDiffSpeed;
uint16_t turnTime, turnTimer = DISABLE;

int16_t speedLeft, speedRight;
int16_t leftPIDOut, rightPIDOut, tracePIDError;

uint16_t ADCValue[3];
int16_t encoderLeft, encoderRight;

uint16_t infraredLeft, infraredCener, infraredRight;

int main(void) {
    SYSCFG_DL_init();
	
	OLED_Init();
	Serial_init(&BluetoothSerial);
	
	NVIC_ClearPendingIRQ(msTimer_INST_INT_IRQN);
	NVIC_EnableIRQ(msTimer_INST_INT_IRQN);
	
	NVIC_ClearPendingIRQ(Bluetooth_INST_INT_IRQN);
	NVIC_EnableIRQ(Bluetooth_INST_INT_IRQN);
	
	Serial_init(&BluetoothSerial);
	
//	NVIC_ClearPendingIRQ(infraredADC_INST_INT_IRQN);
//	NVIC_EnableIRQ(infraredADC_INST_INT_IRQN);

	NVIC_ClearPendingIRQ(Encoder_INT_IRQN);
	NVIC_EnableIRQ(Encoder_INT_IRQN);

	NVIC_ClearPendingIRQ(taskTimer_INST_INT_IRQN);
    NVIC_EnableIRQ(taskTimer_INST_INT_IRQN);
	
    while (1) {
		Motor_set(&motorLeft, 7200);
		Motor_set(&motorRight, 7200);
//		Serial_printf(&BluetoothSerial, "%d,%d\r\n",speedLfet,speedRight);
//		Serial_printf(&BluetoothSerial,"%d,%d,%d\r\n", (int)ADCValue[0], (int)ADCValue[1], (int)ADCValue[2]);
    }
}

void msTimer_INST_IRQHandler(void) {
	if(DL_TimerG_getPendingInterrupt(msTimer_INST) == DL_TIMER_IIDX_ZERO) {
		ms++;
    }
}

void taskTimer_INST_IRQHandler(void) {
    if(DL_TimerG_getPendingInterrupt(taskTimer_INST) == DL_TIMER_IIDX_ZERO) {
		DL_ADC12_startConversion(infraredADC_INST);
		speedLeft = -encoderLeft;
		speedRight = -encoderRight;
		
		OLED_ShowSignedNum(1, 1,speedLeft,4);
		OLED_ShowSignedNum(2, 1,speedRight,4);
		
//		Serial_printf(&BluetoothSerial, "%d,%d\r\n",speedLeft,speedRight);
//		encoderLeft = 0;
//		encoderRight = 0;
		
	
//		
////		switch (lineState) {
////        case OffLine:
////            if (infraredLeft > onLineInfrared ||
////                infraredCenter > onLineInfrared ||
////                infraredRight > onLineInfrared) {
////                lineState = OnLine;
////                action = Advance;
////            }
////            break;

////        case OnLine:
////            if (infraredLeft < offLineInfrared &&
////                infraredCenter < offLineInfrared &&
////                infraredRight < offLineInfrared) {
////                lineState = OffLine;
////                action = Round;
////            }
////            if ((infraredLeft > onCrossInfrared &&
////                 infraredCenter > onCrossInfrared) ||
////                (infraredCenter > onCrossInfrared &&
////                 infraredRight > onCrossInfrared)) {
////                lineState = OnCross;
////                action = Advance;
////            }
////            break;

////        case OnCross:
////            if (direction == Forward) {
////                lineState = OnLine;
////                action = Advance;
////                break;
////            }

////            if (turnTimer == DISABLE) {
////                switch (direction) {
////                case TurnLeft:
////                    turnDiffSpeed = -turnBaseSpeed;
////                    turnTime = turnBaseTime;
////                    break;

////                case TurnRight:
////                    turnDiffSpeed = turnBaseSpeed;
////                    turnTime = turnBaseTime;
////                    break;

////                case TurnBack:
////                    turnDiffSpeed = turnBaseSpeed;
////                    turnTime = 2 * turnBaseTime;
////                    break;

////                default:
////                    break;
////                }
////                lineState = OnCross;
////                action = Turn;
////                turnTimer = ENABLE;
////            }
////            if (turnTimer > turnTime) {
////                lineState = OnLine;
////                action = Advance;
////                turnTimer = DISABLE;
////            }
////            break;
////        }

        switch (action) {
        case Stop:
            leftPIDOut =
                PID_Caculate(&motorLeftPID, speedLeft * encoderToPWM - 0);
            rightPIDOut =
                PID_Caculate(&motorRightPID, speedRight * encoderToPWM - 0);

            Motor_set(&motorLeft, leftPIDOut);
            Motor_set(&motorRight, rightPIDOut);
            break;

        case Advance:
//            if (infraredCenter > infraredMaxCenter) {
//                tracePIDError = infraredLeft - infraredRight;
//            } else if (infraredLeft > infraredRight) {
//                tracePIDError =
//                    (2 * infraredMax - infraredLeft) - infraredRight;
//            } else if (infraredLeft < infraredRight) {
//                tracePIDError =
//                    infraredLeft - (2 * infraredMax - infraredRight);
//            }
//            AdvancediffSpeed = PID_Caculate(&tracePID, tracePIDError);

            leftPIDOut = PID_Caculate(
                &motorLeftPID, speedLeft * encoderToPWM -
                                   (advanceBaseSpeed + AdvancediffSpeed));
            rightPIDOut = PID_Caculate(
                &motorRightPID, speedRight * encoderToPWM -
                                    (advanceBaseSpeed - AdvancediffSpeed));
            LIMIT(leftPIDOut, -7200, 7200);
            LIMIT(rightPIDOut, -7200, 7200);

            Motor_set(&motorLeft, leftPIDOut);
            Motor_set(&motorRight, rightPIDOut);
            break;

        case Turn:
            if (turnTimer) {
                leftPIDOut = PID_Caculate(
                    &motorLeftPID, speedLeft * encoderToPWM - (+turnDiffSpeed));
                rightPIDOut =
                    PID_Caculate(&motorRightPID,
                                 speedRight * encoderToPWM - (-turnDiffSpeed));
                LIMIT(leftPIDOut, -7200, 7200);
                LIMIT(rightPIDOut, -7200, 7200);

                Motor_set(&motorLeft, turnAdvanceSpeed + leftPIDOut);
                Motor_set(&motorRight, turnAdvanceSpeed + rightPIDOut);

                turnTimer += 10;
            }
            break;

        case Round:
            leftPIDOut = PID_Caculate(&motorLeftPID,
                                      speedLeft * encoderToPWM - (+roundSpeed));
            rightPIDOut = PID_Caculate(
                &motorRightPID, speedRight * encoderToPWM - (-roundSpeed));
            LIMIT(leftPIDOut, -7200, 7200);
            LIMIT(rightPIDOut, -7200, 7200);

            Motor_set(&motorLeft, leftPIDOut);
            Motor_set(&motorRight, rightPIDOut);
            break;
        }
    }
}

void infraredADC_INST_IRQHandler(void) {
	if (DL_ADC12_getPendingInterrupt(infraredADC_INST) == DL_ADC12_IIDX_MEM2_RESULT_LOADED) {
        ADCValue[0] = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredLeft);  
		ADCValue[1] = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredCenter);
		ADCValue[2] = DL_ADC12_getMemResult(infraredADC_INST, infraredADC_ADCMEM_infraredRight);		
     }
}

void GROUP1_IRQHandler(void) {
    uint32_t INT_PIN = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN | Encoder_EncoderLeft2_PIN | Encoder_EncoderRight1_PIN | Encoder_EncoderRight2_PIN);
 
    if((INT_PIN & Encoder_EncoderLeft1_PIN) == Encoder_EncoderLeft1_PIN) {
        if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
				encoderLeft--;
			} else  {
				encoderLeft++;
			}
		} else  {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
				encoderLeft++;
			} else {
				encoderLeft--;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN);
    } else if((INT_PIN & Encoder_EncoderLeft2_PIN) == Encoder_EncoderLeft2_PIN) {
		if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
				encoderLeft++;
			} else  {
				encoderLeft--;
			}
		} else  {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
				encoderLeft--;
			} else {
				encoderLeft++;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft2_PIN);
    } else if((INT_PIN & Encoder_EncoderRight1_PIN) == Encoder_EncoderRight1_PIN) {
        if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
				encoderRight--;
			} else  {
				encoderRight++;
			}
		} else  {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
				encoderRight++;
			} else  {
				encoderRight--;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight1_PIN);
    } else if((INT_PIN & Encoder_EncoderRight2_PIN) == Encoder_EncoderRight2_PIN) {
		if(DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
				encoderRight++;
			} else  {
				encoderRight--;
			}
		} else {
			if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
				encoderRight--;
			} else  {
				encoderRight++;
		       }
		}
		DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight2_PIN);
    }
}

void Bluetooth_INST_IRQHandler(void) {
  if (DL_UART_Main_getPendingInterrupt(Bluetooth_INST) == DL_UART_IIDX_RX) {
        Serial_Praser(&BluetoothSerial);
        Serial_Handler(&BluetoothSerial);
  }
}


void Serial_Praser(Serial *serial) {
    serial->ByteData = Serial_readByte(serial);
	
	Serial_writeByte(&BluetoothSerial, serial->ByteData);

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
			
			default:
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
