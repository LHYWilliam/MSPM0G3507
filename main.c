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
#include "delay.h"
#include "dmp.h"
#include "motor.h"
#include "pid.h"
#include "serial.h"
#include "mpu.h"

#include "MadgwickAHRS.h"

#define YAW (getYaw() - 180)

MPU6050Params mpu6050 = {
    .MPU6050dt = 10,
    .preMillis = 0,
    .MPU6050ERROE = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

SensorMsg msg = {
	.A = {0.0f, 0.0f, 0.0f},
	.G = {0.0f, 0.0f, 0.0f}
};

#define MAPPING(x) ((x) >= 0 ? (x) : (360 + (x)))

volatile uint32_t ms = 0;
uint8_t question = 2;

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
    .invert = DISABLE,
};

Motor motorRight = {
    .IN1_gpio = MotorIN_RightIN1_PORT,
    .IN1_pins = MotorIN_RightIN1_PIN,
    .IN2_gpio = MotorIN_RightIN2_PORT,
    .IN2_pins = MotorIN_RightIN2_PIN,
    .PWM = MotorPWM_INST,
    .Index = GPIO_MotorPWM_C1_IDX,
    .invert = ENABLE,
};

PID tracePID = {
    .Kp = -0.3,
    .Ki = 0,
    .Kd = -0.001,
    .imax = 1024,
};

PID advanceYawPID = {
	  .Kp = 20,
    .Ki = 0,
    .Kd = -0,
    .imax = 1024,
};

PID turnYawPID = {
	  .Kp = 20,
    .Ki = 0,
    .Kd = -0,
    .imax = 1024,
};

PID motorLeftPID = {
    .Kp = -2.25,
    .Ki = -8,
    .Kd = 0,
    .imax = 2048,
};

PID motorRightPID = {
    .Kp = -2.25,
    .Ki = -8,
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
  Trace,
  Advance,
  Turn,
	Adapt,
} ActionType;
ActionType action = Stop;
char *actionString[] = {"Stop", "Trace", "Advance", "Turn"};

typedef enum {
  OffLine,
  OnLine,
} LineType;
LineType lineState = OffLine;
char *lineString[] = {"OffLine", "OnLine"};

void Serial_Praser(Serial *serial);
void Serial_Handler(Serial *serial);

float encoderLeftToPWM = 7200. / 70.685, encoderRightToPWM = 7200 / 70.285;
uint16_t infraredMax = 3850, infraredMaxCenter = 2500;
uint16_t offLineInfrared = 1400, onLineInfrared = 2048;
uint16_t advanceBaseSpeed = 2048, turnBaseTime = 1000, adaptBaseSpeed = 512;

int16_t AdvancediffSpeed, turnDiffSpeed, adaptDiffSpeed;
uint16_t turnTime = 1000, turnTimer = DISABLE;
uint8_t infraredFilterTime = 1;

int16_t speedLeft, speedRight;
int16_t leftPIDOut, rightPIDOut, tracePIDError;

int16_t encoderLeft, encoderRight;
uint16_t infraredLeft, infraredCenter, infraredRight;

float AdvanceYaw, turnTimeYaw, turnTargetYaw;
float turnDiffYaw[] = {44.69, 44.00,44.69,44.00,44.69,44.00,44.69,44.00,};
int16_t yawPIDOut;

uint16_t ADCValue[3];
uint8_t infraredState[4];

uint8_t traceToAdvancceCount = 0, traceToTurnCount = 0;

int main(void) {
  SYSCFG_DL_init();

  OLED_Init();
	Serial_init(&BluetoothSerial);
	
	Delay_ms(1000);
	begin(1000.0f / (float)mpu6050.MPU6050dt);
	MPU_Init();
	MPU_getError();

  NVIC_ClearPendingIRQ(msTimer_INST_INT_IRQN);
  NVIC_EnableIRQ(msTimer_INST_INT_IRQN);

  NVIC_ClearPendingIRQ(Bluetooth_INST_INT_IRQN);
  NVIC_EnableIRQ(Bluetooth_INST_INT_IRQN);

  Serial_init(&BluetoothSerial);
  PID_Init(&tracePID);
	PID_Init(&advanceYawPID);
	PID_Init(&turnYawPID);
  PID_Init(&motorLeftPID);
  PID_Init(&motorRightPID);

  DL_ADC12_startConversion(infraredADC_INST);

  NVIC_ClearPendingIRQ(Encoder_INT_IRQN);
  NVIC_EnableIRQ(Encoder_INT_IRQN);

  NVIC_ClearPendingIRQ(infraredADC_INST_INT_IRQN);
  NVIC_EnableIRQ(infraredADC_INST_INT_IRQN);

  NVIC_ClearPendingIRQ(taskTimer_INST_INT_IRQN);
  NVIC_EnableIRQ(taskTimer_INST_INT_IRQN);

  while (1) {
//    OLED_ShowNum(1, 1, infraredLeft, 4);
//    OLED_ShowNum(2, 1, infraredCenter, 4);
//    OLED_ShowNum(3, 1, infraredRight, 4);
//		DMP_GetData(&pitch, &roll, &yaw);
//		OLED_ShowSignedNum(1, 1, pitch, 4);
//		OLED_ShowSignedNum(2, 1, roll, 4);
//		OLED_ShowSignedNum(3, 1, YAW, 3);
//		OLED_ShowSignedNum(1, 1, yaw, 4);
//		OLED_ShowNum(2, 1, traceToAdvancceCount, 6);
//		OLED_ShowString(3, 1, actionString[action]);
//		OLED_ShowNum(1, 1, infraredState[0], 1);
//		OLED_ShowNum(2, 1, infraredState[1], 1);
//		OLED_ShowNum(3, 1, infraredState[2], 1);
//		OLED_ShowNum(4, 1, infraredState[3], 1);
			if(ms - mpu6050.preMillis >= mpu6050.MPU6050dt) {
				mpu6050.preMillis = ms;
				dataGetAndFilter();		                     
				updateIMU(msg.G[0], msg.G[1], msg.G[2], msg.A[0], msg.A[1], msg.A[2]);
			}
		OLED_ShowString(2, 1, "Yaw:");
		OLED_ShowSignedNum(2, 8, getYaw(), 3);
		OLED_ShowNum(2, 13, (uint32_t)(getYaw() * 100) % 100, 2);			
		OLED_ShowString(3, 1, "Roll:");
		OLED_ShowSignedNum(3, 8, getRoll(), 3);
		OLED_ShowNum(3, 13, (uint32_t)(getRoll() * 100) % 100, 2);			
		OLED_ShowString(4, 1, "Pitch:");
		OLED_ShowSignedNum(4, 8, getPitch(), 3);
		OLED_ShowNum(4, 13, (uint32_t)(getPitch() * 100) % 100, 2);	
  }
}

void msTimer_INST_IRQHandler(void) {
  if (DL_TimerG_getPendingInterrupt(msTimer_INST) == DL_TIMER_IIDX_ZERO) {
    ms++;
  }
}

void taskTimer_INST_IRQHandler(void) {
  if (DL_TimerG_getPendingInterrupt(taskTimer_INST) == DL_TIMER_IIDX_ZERO) {
    speedLeft = encoderLeft;
    speedRight = encoderRight;
    encoderLeft = 0;
    encoderRight = 0;

    infraredLeft = ADCValue[0];
    infraredCenter = ADCValue[1];
    infraredRight = ADCValue[2];
		
		infraredState[0] += DL_GPIO_readPins(infrared_PORT, infrared_infrared1_PIN) ? 1 : 0;
		infraredState[3] += DL_GPIO_readPins(infrared_PORT, infrared_infrared4_PIN) ? 1 : 0;

    switch (lineState) {
		case OffLine:
								switch (action){
								case Advance:
									if (infraredLeft > onLineInfrared || infraredCenter > onLineInfrared ||
										infraredRight > onLineInfrared) {
										if (question == 1) {
											lineState = OffLine;
											action = Stop;
										} else {
											lineState = OnLine;
											action = Trace;
										}
										
										tracePID.integrator = 0;
										motorLeftPID.integrator = 0;
										motorRightPID.integrator = 0;
									}										
											
									if (infraredState[0] > infraredFilterTime || infraredState[3] > infraredFilterTime) {
										lineState = OffLine;
										action = Adapt;
									}
								break;
								
								case Turn:
									turnTimer += 10;
									
									if (turnTimer > turnTime){
										lineState = OffLine;
										action = Advance;
										
										infraredState[0] = 0;
										infraredState[1] = 0;
										infraredState[2] = 0;
										infraredState[3] = 0;
										
										AdvanceYaw = YAW;
										turnTimer = DISABLE;
									}
									break;
									
								case Adapt:
									if (infraredLeft > onLineInfrared || infraredCenter > onLineInfrared || infraredRight > onLineInfrared) {
										if (question == 1) {
											lineState = OffLine;
											action = Stop;
										} else {
											lineState = OnLine;
											action = Trace;
											
										tracePID.integrator = 0;
										motorLeftPID.integrator = 0;
										motorRightPID.integrator = 0;
										}
									}
									break;
									
								default:
									break;
							}
			break;

    case OnLine:
								if (infraredLeft < offLineInfrared && infraredCenter < offLineInfrared &&
										infraredRight < offLineInfrared) {
									if (question == 1 || question == 2) {
										lineState = OffLine;
										action = Advance;
										
										traceToAdvancceCount++;
										
										infraredState[0] = 0;
										infraredState[1] = 0;
										infraredState[2] = 0;
										infraredState[3] = 0;
										
										tracePID.integrator = 0;
										motorLeftPID.integrator = 0;
										motorRightPID.integrator = 0;
										
										if (traceToAdvancceCount >= 2){
											lineState = OffLine;
											action = Stop;
										}
									} else if (question == 3 || question == 4) {
										lineState = OffLine;
										action = Turn;
										
										infraredState[0] = 0;
										infraredState[1] = 0;
										infraredState[2] = 0;
										infraredState[3] = 0;
										
										traceToTurnCount++;
										turnTimer = ENABLE;
										turnTimeYaw = YAW;
										
										if ((question == 3 && traceToTurnCount >= 2) || (question == 4 && traceToTurnCount >= 8)) {
											lineState = OffLine;
											action = Stop;
										}
									}
								}
    }

    switch (action) {
    case Stop:
				leftPIDOut =
						PID_Caculate(&motorLeftPID, speedLeft * encoderLeftToPWM - 0);
				rightPIDOut =
						PID_Caculate(&motorRightPID, speedRight * encoderRightToPWM - 0);

				Motor_set(&motorLeft, leftPIDOut);
				Motor_set(&motorRight, rightPIDOut);
				break;

    case Trace:
				if (infraredCenter > infraredMaxCenter) {
					tracePIDError = infraredLeft - infraredRight;
				} else if (infraredLeft > infraredRight) {
					tracePIDError = (2 * infraredMax - infraredLeft) - infraredRight;
				} else if (infraredLeft < infraredRight) {
					tracePIDError = infraredLeft - (2 * infraredMax - infraredRight);
				}
				AdvancediffSpeed = PID_Caculate(&tracePID, tracePIDError);

				leftPIDOut = PID_Caculate(&motorLeftPID,
																	speedLeft * encoderLeftToPWM -
																			(advanceBaseSpeed + AdvancediffSpeed));
				rightPIDOut = PID_Caculate(&motorRightPID,
																	 speedRight * encoderRightToPWM -
																			 (advanceBaseSpeed - AdvancediffSpeed));
				LIMIT(leftPIDOut, -7200, 7200);
				LIMIT(rightPIDOut, -7200, 7200);

				Motor_set(&motorLeft, leftPIDOut);
				Motor_set(&motorRight, rightPIDOut);
				break;

    case Advance:
				if (question == 1 || question == 2) {
					float advanceTargetYaw = 0;
					if (traceToAdvancceCount == 0) {
						yawPIDOut = PID_Caculate(&advanceYawPID, YAW - 0);
					} else if (traceToAdvancceCount == 1){
//						advanceYawPID.Kp = 0.9 - 0.005;
						yawPIDOut = PID_Caculate(&advanceYawPID, YAW - YAW > 0 ? 180 : (-180));
					}
				} else if (question == 3 || question == 4) {
					if (traceToTurnCount % 2 == 0){
						yawPIDOut = PID_Caculate(&turnYawPID, YAW - AdvanceYaw);
					} else {
						yawPIDOut = PID_Caculate(&turnYawPID, MAPPING(YAW) - MAPPING(AdvanceYaw));
					}
				}
				
				leftPIDOut = PID_Caculate(&motorLeftPID, speedLeft * encoderLeftToPWM -
																										 (advanceBaseSpeed + yawPIDOut)); 
				rightPIDOut = PID_Caculate(
						&motorRightPID, speedRight * encoderRightToPWM - (advanceBaseSpeed -yawPIDOut));

				Motor_set(&motorLeft, leftPIDOut);
				Motor_set(&motorRight, rightPIDOut);
				break;

    case Turn:
				if (turnTimer) {
					if (traceToTurnCount % 2 == 1) {
						turnTargetYaw = MAPPING(turnTimeYaw + turnDiffYaw[traceToTurnCount - 1]);
					} else {
						turnTargetYaw = turnTimeYaw - turnDiffYaw[traceToTurnCount - 1];
					}
					yawPIDOut = PID_Caculate(&turnYawPID, MAPPING(YAW) - turnTargetYaw);
					
					leftPIDOut = PID_Caculate(&motorLeftPID, speedLeft * encoderLeftToPWM -
																											 (+yawPIDOut));	
					rightPIDOut = PID_Caculate(
							&motorRightPID, speedRight * encoderRightToPWM - (-yawPIDOut));
					LIMIT(leftPIDOut, -7200, 7200);
					LIMIT(rightPIDOut, -7200, 7200);

					Motor_set(&motorLeft,  leftPIDOut);
					Motor_set(&motorRight, rightPIDOut);
				}
				break;
		
		case Adapt:
			if (infraredState[0] > infraredFilterTime) {
				adaptDiffSpeed = -2 * adaptBaseSpeed;
			} else if (infraredState[3] > infraredFilterTime) {
				adaptDiffSpeed = 2 * adaptBaseSpeed;
			}
			
			leftPIDOut = PID_Caculate(&motorLeftPID, speedLeft * encoderLeftToPWM - (+adaptDiffSpeed));	
			rightPIDOut = PID_Caculate(&motorRightPID, speedRight * encoderRightToPWM - (-adaptDiffSpeed));
			LIMIT(leftPIDOut, -7200, 7200);
			LIMIT(rightPIDOut, -7200, 7200);

			Motor_set(&motorLeft,  leftPIDOut);
			Motor_set(&motorRight, rightPIDOut);
			break;
    }
  }
}

void infraredADC_INST_IRQHandler(void) {
  if (DL_ADC12_getPendingInterrupt(infraredADC_INST) ==
      DL_ADC12_IIDX_MEM2_RESULT_LOADED) {
    ADCValue[0] = DL_ADC12_getMemResult(infraredADC_INST,
                                        infraredADC_ADCMEM_infraredLeft);
    ADCValue[1] = DL_ADC12_getMemResult(infraredADC_INST,
                                        infraredADC_ADCMEM_infraredCenter);
    ADCValue[2] = DL_ADC12_getMemResult(infraredADC_INST,
                                        infraredADC_ADCMEM_infraredRight);
  }
}

void GROUP1_IRQHandler(void) {
  uint32_t INT_PIN = DL_GPIO_getEnabledInterruptStatus(
      Encoder_PORT, Encoder_EncoderLeft1_PIN | Encoder_EncoderLeft2_PIN |
                        Encoder_EncoderRight1_PIN | Encoder_EncoderRight2_PIN);

  if ((INT_PIN & Encoder_EncoderLeft1_PIN) == Encoder_EncoderLeft1_PIN) {
    if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
        encoderLeft--;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) ==
                 0) {
        encoderLeft++;
      }
    } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) == 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
        encoderLeft++;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) ==
                 0) {
        encoderLeft--;
      }
    }
    DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN);
  } else if ((INT_PIN & Encoder_EncoderLeft2_PIN) == Encoder_EncoderLeft2_PIN) {
    if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) > 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
        encoderLeft++;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) ==
                 0) {
        encoderLeft--;
      }
    } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft2_PIN) == 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) > 0) {
        encoderLeft--;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderLeft1_PIN) ==
                 0) {
        encoderLeft++;
      }
    }
    DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft2_PIN);
  } else if ((INT_PIN & Encoder_EncoderRight1_PIN) ==
             Encoder_EncoderRight1_PIN) {
    if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
        encoderRight--;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) ==
                 0) {
        encoderRight++;
      }
    } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) == 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
        encoderRight++;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) ==
                 0) {
        encoderRight--;
      }
    }
    DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight1_PIN);
  } else if ((INT_PIN & Encoder_EncoderRight2_PIN) ==
             Encoder_EncoderRight2_PIN) {
    if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) > 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
        encoderRight++;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) ==
                 0) {
        encoderRight--;
      }
    } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight2_PIN) == 0) {
      if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) > 0) {
        encoderRight--;
      } else if (DL_GPIO_readPins(Encoder_PORT, Encoder_EncoderRight1_PIN) ==
                 0) {
        encoderRight++;
      }
    }
    DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderRight2_PIN);
  }
}

// void Bluetooth_INST_IRQHandler(void) {
//   if (DL_UART_Main_getPendingInterrupt(Bluetooth_INST) == DL_UART_IIDX_RX) {
//         Serial_Praser(&BluetoothSerial);
//         Serial_Handler(&BluetoothSerial);
//   }
// }

// void Serial_Praser(Serial *serial) {
//     serial->ByteData = Serial_readByte(serial);

//    switch (serial->type) {
//    case None:
//        if (serial->ByteData == 0xFF) {
//            serial->type = HexPack;
//        } else {
//            Serial_clear(serial);
//        }
//        break;

//    case HexPack:
//        if (serial->ByteData == 0xFE && serial->count == 3) {
//            serial->RecieveFlag = SET;
//        } else {
//            serial->HexData[serial->count++] = serial->ByteData;
//        }

//        if (serial->count > 3) {
//            Serial_clear(serial);
//        }
//        break;

//    default:
//        Serial_clear(serial);
//        break;
//    }
//}

// void Serial_Handler(Serial *serial) {
//     if (serial->RecieveFlag == SET) {
//         action = serial->HexData[0];

//        switch (action) {
//        case Stop:
//            break;

//        case Advance:
//            AdvancediffSpeed =
//                (int16_t)(serial->HexData[1] << 8 | serial->HexData[2]);
//            break;

//        case Turn:
//            direction = (int16_t)(serial->HexData[1] << 8 |
//            serial->HexData[2]);

//            switch (direction) {
//            case Forward:
//                break;

//            case TurnLeft:
//                turnTime = turnBaseTime;
//                turnDiffSpeed = -turnBaseSpeed;
//                break;

//            case TurnRight:
//                turnTime = turnBaseTime;
//                turnDiffSpeed = turnBaseSpeed;
//                break;

//            case TurnBack:
//                turnTime = turnBaseTime * 2;
//                turnDiffSpeed = turnBaseSpeed;
//                break;
//
//			default:
//				break;
//            }

//            turnTimer = ENABLE;
//            break;

//        case Round:
//            break;

//        default:
//            action = Stop;
//            break;
//        }

//        Serial_clear(serial);
//    }
//}
