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

Serial OpenMVSerial = {
	.uart = OpenMVSerial_INST,
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

int main(void)
{
    SYSCFG_DL_init();
	
	Delay_ms(2500);
	
	OLED_Init();
	Serial_init(&OpenMVSerial);
	
	NVIC_ClearPendingIRQ(OpenMVSerial_INST_INT_IRQN);
	NVIC_EnableIRQ(OpenMVSerial_INST_INT_IRQN);
	
	NVIC_ClearPendingIRQ(Timer_INST_INT_IRQN);
    NVIC_EnableIRQ(Timer_INST_INT_IRQN);
	
    while (1) {
		OLED_ShowString(1, 1, "Action:         ");
        OLED_ShowString(1, 8, action == Turn ? directionString[direction]
                                             : actionString[action]);
        OLED_ShowSignedNum(2, 6, AdvancediffSpeed, 5);
    }
}

void Timer_INST_IRQHandler(void) {
    if(DL_TimerG_getPendingInterrupt(Timer_INST) == DL_TIMER_IIDX_ZERO) {
		switch (action) {
        case Stop:
            Motor_set(&motorLeft, 0);
            Motor_set(&motorRight, 0);
            break;

        case Advance:
            Motor_set(&motorLeft, advanceBaseSpeed - AdvancediffSpeed);
            Motor_set(&motorRight, advanceBaseSpeed + AdvancediffSpeed);
            break;

        case Turn:
            if (turnTimer) {
                Motor_set(&motorLeft, -turnDiffSpeed);
                Motor_set(&motorRight, +turnDiffSpeed);

                turnTimer += 10;
                if (turnTimer > turnTime) {
                    action = Stop;
                    turnTimer = DISABLE;
                }
            }
            break;

        case Round:
            Motor_set(&motorLeft, -turnDiffSpeed);
            Motor_set(&motorRight, +turnDiffSpeed);
            break;
        }
    }
}

void OpenMVSerial_INST_IRQHandler(void) {
  if (DL_UART_Main_getPendingInterrupt(OpenMVSerial.uart) == DL_UART_IIDX_RX) {
        Serial_Praser(&OpenMVSerial);
        Serial_Handler(&OpenMVSerial);
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
