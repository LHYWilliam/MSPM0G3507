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





/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOA)

/* Defines for SCL: GPIOA.31 with pinCMx 6 on package pin 39 */
#define OLED_SCL_PIN                                            (DL_GPIO_PIN_31)
#define OLED_SCL_IOMUX                                            (IOMUX_PINCM6)
/* Defines for SDA: GPIOA.28 with pinCMx 3 on package pin 35 */
#define OLED_SDA_PIN                                            (DL_GPIO_PIN_28)
#define OLED_SDA_IOMUX                                            (IOMUX_PINCM3)
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



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MotorPWM_init(void);
void SYSCFG_DL_Timer_init(void);
void SYSCFG_DL_OpenMVSerial_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
