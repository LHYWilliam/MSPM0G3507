/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gMotorPWMBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_MotorPWM_init();
    SYSCFG_DL_Timer_init();
    SYSCFG_DL_OpenMVSerial_init();
    /* Ensure backup structures have no valid state */
	gMotorPWMBackup.backupRdy 	= false;



}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(MotorPWM_INST, &gMotorPWMBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(MotorPWM_INST, &gMotorPWMBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(MotorPWM_INST);
    DL_TimerG_reset(Timer_INST);
    DL_UART_Main_reset(OpenMVSerial_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(MotorPWM_INST);
    DL_TimerG_enablePower(Timer_INST);
    DL_UART_Main_enablePower(OpenMVSerial_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_MotorPWM_C0_IOMUX,GPIO_MotorPWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MotorPWM_C0_PORT, GPIO_MotorPWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MotorPWM_C1_IOMUX,GPIO_MotorPWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MotorPWM_C1_PORT, GPIO_MotorPWM_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_OpenMVSerial_IOMUX_TX, GPIO_OpenMVSerial_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_OpenMVSerial_IOMUX_RX, GPIO_OpenMVSerial_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_LeftIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_LeftIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_RightIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_RightIN2_IOMUX);

    DL_GPIO_clearPins(GPIOA, OLED_SCL_PIN |
		OLED_SDA_PIN |
		MotorIN_LeftIN1_PIN |
		MotorIN_LeftIN2_PIN);
    DL_GPIO_enableOutput(GPIOA, OLED_SCL_PIN |
		OLED_SDA_PIN |
		MotorIN_LeftIN1_PIN |
		MotorIN_LeftIN2_PIN);
    DL_GPIO_clearPins(GPIOB, MotorIN_RightIN1_PIN |
		MotorIN_RightIN2_PIN);
    DL_GPIO_enableOutput(GPIOB, MotorIN_RightIN1_PIN |
		MotorIN_RightIN2_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_enableMFCLK();

}


/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMotorPWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMotorPWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 7200,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_MotorPWM_init(void) {

    DL_TimerA_setClockConfig(
        MotorPWM_INST, (DL_TimerA_ClockConfig *) &gMotorPWMClockConfig);

    DL_TimerA_initPWMMode(
        MotorPWM_INST, (DL_TimerA_PWMConfig *) &gMotorPWMConfig);

    DL_TimerA_setCaptureCompareOutCtl(MotorPWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MotorPWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MotorPWM_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MotorPWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MotorPWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(MotorPWM_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(MotorPWM_INST);


    
    DL_TimerA_setCCPDirection(MotorPWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000 Hz = 4000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gTimerClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Timer_INST_LOAD_VALUE = (10 ms * 40000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTimerTimerConfig = {
    .period     = Timer_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Timer_init(void) {

    DL_TimerG_setClockConfig(Timer_INST,
        (DL_TimerG_ClockConfig *) &gTimerClockConfig);

    DL_TimerG_initTimerMode(Timer_INST,
        (DL_TimerG_TimerConfig *) &gTimerTimerConfig);
    DL_TimerG_enableInterrupt(Timer_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(Timer_INST);





}



static const DL_UART_Main_ClockConfig gOpenMVSerialClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gOpenMVSerialConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_OpenMVSerial_init(void)
{
    DL_UART_Main_setClockConfig(OpenMVSerial_INST, (DL_UART_Main_ClockConfig *) &gOpenMVSerialClockConfig);

    DL_UART_Main_init(OpenMVSerial_INST, (DL_UART_Main_Config *) &gOpenMVSerialConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9598.08
     */
    DL_UART_Main_setOversampling(OpenMVSerial_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(OpenMVSerial_INST, OpenMVSerial_IBRD_4_MHZ_9600_BAUD, OpenMVSerial_FBRD_4_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(OpenMVSerial_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(OpenMVSerial_INST_INT_IRQN, 1);


    DL_UART_Main_enable(OpenMVSerial_INST);
}

