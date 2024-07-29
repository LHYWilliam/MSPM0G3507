/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for MotorPWM */
#define MotorPWM_INST                                                      TIMA0
#define MotorPWM_INST_IRQHandler                                TIMA0_IRQHandler
#define MotorPWM_INST_INT_IRQN                                  (TIMA0_INT_IRQn)
#define MotorPWM_INST_CLK_FREQ                                           4000000
/* GPIO defines for channel 0 */
#define GPIO_MotorPWM_C0_PORT                                              GPIOA
#define GPIO_MotorPWM_C0_PIN                                       DL_GPIO_PIN_8
#define GPIO_MotorPWM_C0_IOMUX                                   (IOMUX_PINCM19)
#define GPIO_MotorPWM_C0_IOMUX_FUNC                  IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_MotorPWM_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MotorPWM_C1_PORT                                              GPIOA
#define GPIO_MotorPWM_C1_PIN                                       DL_GPIO_PIN_9
#define GPIO_MotorPWM_C1_IOMUX                                   (IOMUX_PINCM20)
#define GPIO_MotorPWM_C1_IOMUX_FUNC                  IOMUX_PINCM20_PF_TIMA0_CCP1
#define GPIO_MotorPWM_C1_IDX                                 DL_TIMER_CC_1_INDEX



/* Defines for Timer */
#define Timer_INST                                                       (TIMG0)
#define Timer_INST_IRQHandler                                   TIMG0_IRQHandler
#define Timer_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define Timer_INST_LOAD_VALUE                                             (399U)



/* Defines for OpenMVSerial */
#define OpenMVSerial_INST                                                  UART1
#define OpenMVSerial_INST_IRQHandler                            UART1_IRQHandler
#define OpenMVSerial_INST_INT_IRQN                                UART1_INT_IRQn
#define GPIO_OpenMVSerial_RX_PORT                                          GPIOB
#define GPIO_OpenMVSerial_TX_PORT                                          GPIOB
#define GPIO_OpenMVSerial_RX_PIN                                   DL_GPIO_PIN_7
#define GPIO_OpenMVSerial_TX_PIN                                   DL_GPIO_PIN_6
#define GPIO_OpenMVSerial_IOMUX_RX                               (IOMUX_PINCM24)
#define GPIO_OpenMVSerial_IOMUX_TX                               (IOMUX_PINCM23)
#define GPIO_OpenMVSerial_IOMUX_RX_FUNC                IOMUX_PINCM24_PF_UART1_RX
#define GPIO_OpenMVSerial_IOMUX_TX_FUNC                IOMUX_PINCM23_PF_UART1_TX
#define OpenMVSerial_BAUD_RATE                                            (9600)
#define OpenMVSerial_IBRD_4_MHZ_9600_BAUD                                   (26)
#define OpenMVSerial_FBRD_4_MHZ_9600_BAUD                                    (3)





/* Defines for infraredADC */
#define infraredADC_INST                                                    ADC1
#define infraredADC_INST_IRQHandler                              ADC1_IRQHandler
#define infraredADC_INST_INT_IRQN                                (ADC1_INT_IRQn)
#define infraredADC_ADCMEM_infraredLeft                       DL_ADC12_MEM_IDX_1
#define infraredADC_ADCMEM_infraredLeft_REF         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define infraredADC_ADCMEM_infraredLeft_REF_VOLTAGE_V                                     3.3
#define infraredADC_ADCMEM_infraredCenter                      DL_ADC12_MEM_IDX_2
#define infraredADC_ADCMEM_infraredCenter_REF         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define infraredADC_ADCMEM_infraredCenter_REF_VOLTAGE_V                                     3.3
#define infraredADC_ADCMEM_infraredRight                      DL_ADC12_MEM_IDX_3
#define infraredADC_ADCMEM_infraredRight_REF         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define infraredADC_ADCMEM_infraredRight_REF_VOLTAGE_V                                     3.3
#define GPIO_infraredADC_C1_PORT                                           GPIOA
#define GPIO_infraredADC_C1_PIN                                   DL_GPIO_PIN_16
#define GPIO_infraredADC_C2_PORT                                           GPIOA
#define GPIO_infraredADC_C2_PIN                                   DL_GPIO_PIN_17
#define GPIO_infraredADC_C3_PORT                                           GPIOA
#define GPIO_infraredADC_C3_PIN                                   DL_GPIO_PIN_18



/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOA)

/* Defines for OLEDSDA: GPIOA.31 with pinCMx 6 on package pin 39 */
#define OLED_OLEDSDA_PIN                                        (DL_GPIO_PIN_31)
#define OLED_OLEDSDA_IOMUX                                        (IOMUX_PINCM6)
/* Defines for OLEDSCL: GPIOA.28 with pinCMx 3 on package pin 35 */
#define OLED_OLEDSCL_PIN                                        (DL_GPIO_PIN_28)
#define OLED_OLEDSCL_IOMUX                                        (IOMUX_PINCM3)
/* Defines for LeftIN1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define MotorIN_LeftIN1_PORT                                             (GPIOA)
#define MotorIN_LeftIN1_PIN                                     (DL_GPIO_PIN_25)
#define MotorIN_LeftIN1_IOMUX                                    (IOMUX_PINCM55)
/* Defines for LeftIN2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define MotorIN_LeftIN2_PORT                                             (GPIOA)
#define MotorIN_LeftIN2_PIN                                     (DL_GPIO_PIN_26)
#define MotorIN_LeftIN2_IOMUX                                    (IOMUX_PINCM59)
/* Defines for RightIN1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define MotorIN_RightIN1_PORT                                            (GPIOB)
#define MotorIN_RightIN1_PIN                                     (DL_GPIO_PIN_8)
#define MotorIN_RightIN1_IOMUX                                   (IOMUX_PINCM25)
/* Defines for RightIN2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define MotorIN_RightIN2_PORT                                            (GPIOB)
#define MotorIN_RightIN2_PIN                                     (DL_GPIO_PIN_9)
#define MotorIN_RightIN2_IOMUX                                   (IOMUX_PINCM26)
/* Port definition for Pin Group Encoder */
#define Encoder_PORT                                                     (GPIOA)

/* Defines for EncoderLeft1: GPIOA.12 with pinCMx 34 on package pin 5 */
// pins affected by this interrupt request:["EncoderLeft1","EncoderLeft2","EncoderRight1","EncoderRight2"]
#define Encoder_INT_IRQN                                        (GPIOA_INT_IRQn)
#define Encoder_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Encoder_EncoderLeft1_IIDX                           (DL_GPIO_IIDX_DIO12)
#define Encoder_EncoderLeft1_PIN                                (DL_GPIO_PIN_12)
#define Encoder_EncoderLeft1_IOMUX                               (IOMUX_PINCM34)
/* Defines for EncoderLeft2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define Encoder_EncoderLeft2_IIDX                           (DL_GPIO_IIDX_DIO13)
#define Encoder_EncoderLeft2_PIN                                (DL_GPIO_PIN_13)
#define Encoder_EncoderLeft2_IOMUX                               (IOMUX_PINCM35)
/* Defines for EncoderRight1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Encoder_EncoderRight1_IIDX                          (DL_GPIO_IIDX_DIO14)
#define Encoder_EncoderRight1_PIN                               (DL_GPIO_PIN_14)
#define Encoder_EncoderRight1_IOMUX                              (IOMUX_PINCM36)
/* Defines for EncoderRight2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define Encoder_EncoderRight2_IIDX                          (DL_GPIO_IIDX_DIO15)
#define Encoder_EncoderRight2_PIN                               (DL_GPIO_PIN_15)
#define Encoder_EncoderRight2_IOMUX                              (IOMUX_PINCM37)
/* Port definition for Pin Group MPU */
#define MPU_PORT                                                         (GPIOA)

/* Defines for MPUSCL: GPIOA.0 with pinCMx 1 on package pin 33 */
#define MPU_MPUSCL_PIN                                           (DL_GPIO_PIN_0)
#define MPU_MPUSCL_IOMUX                                          (IOMUX_PINCM1)
/* Defines for MPUSDA: GPIOA.1 with pinCMx 2 on package pin 34 */
#define MPU_MPUSDA_PIN                                           (DL_GPIO_PIN_1)
#define MPU_MPUSDA_IOMUX                                          (IOMUX_PINCM2)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MotorPWM_init(void);
void SYSCFG_DL_Timer_init(void);
void SYSCFG_DL_OpenMVSerial_init(void);
void SYSCFG_DL_infraredADC_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
