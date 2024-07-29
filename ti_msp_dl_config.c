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
DL_TimerG_backupConfig gmsTimerBackup;

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
    SYSCFG_DL_taskTimer_init();
    SYSCFG_DL_msTimer_init();
    SYSCFG_DL_Bluetooth_init();
    SYSCFG_DL_infraredADC_init();
    /* Ensure backup structures have no valid state */
	gMotorPWMBackup.backupRdy 	= false;
	gmsTimerBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(MotorPWM_INST, &gMotorPWMBackup);
	retStatus &= DL_TimerG_saveConfiguration(msTimer_INST, &gmsTimerBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(MotorPWM_INST, &gMotorPWMBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(msTimer_INST, &gmsTimerBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(MotorPWM_INST);
    DL_TimerG_reset(taskTimer_INST);
    DL_TimerG_reset(msTimer_INST);
    DL_UART_Main_reset(Bluetooth_INST);
    DL_ADC12_reset(infraredADC_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(MotorPWM_INST);
    DL_TimerG_enablePower(taskTimer_INST);
    DL_TimerG_enablePower(msTimer_INST);
    DL_UART_Main_enablePower(Bluetooth_INST);
    DL_ADC12_enablePower(infraredADC_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_MotorPWM_C0_IOMUX,GPIO_MotorPWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MotorPWM_C0_PORT, GPIO_MotorPWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MotorPWM_C1_IOMUX,GPIO_MotorPWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MotorPWM_C1_PORT, GPIO_MotorPWM_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Bluetooth_IOMUX_TX, GPIO_Bluetooth_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_Bluetooth_IOMUX_RX, GPIO_Bluetooth_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutputFeatures(LED_PIN_0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(Buzzer_Buzzer1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutput(OLED_OLEDSDA_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_OLEDSCL_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_LeftIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_LeftIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_RightIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MotorIN_RightIN2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Encoder_EncoderLeft1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Encoder_EncoderLeft2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Encoder_EncoderRight1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Encoder_EncoderRight2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MPU_MPUSCL_IOMUX);

    DL_GPIO_initDigitalOutput(MPU_MPUSDA_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Key_Key1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Key2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Key3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Key4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Key5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, LED_PIN_0_PIN |
		MotorIN_LeftIN1_PIN |
		MotorIN_LeftIN2_PIN |
		MotorIN_RightIN1_PIN |
		MotorIN_RightIN2_PIN);
    DL_GPIO_setPins(GPIOA, Buzzer_Buzzer1_PIN |
		OLED_OLEDSDA_PIN |
		OLED_OLEDSCL_PIN |
		MPU_MPUSCL_PIN |
		MPU_MPUSDA_PIN);
    DL_GPIO_enableOutput(GPIOA, LED_PIN_0_PIN |
		Buzzer_Buzzer1_PIN |
		OLED_OLEDSDA_PIN |
		OLED_OLEDSCL_PIN |
		MotorIN_LeftIN1_PIN |
		MotorIN_LeftIN2_PIN |
		MotorIN_RightIN1_PIN |
		MotorIN_RightIN2_PIN |
		MPU_MPUSCL_PIN |
		MPU_MPUSDA_PIN);
    DL_GPIO_setUpperPinsPolarity(Encoder_PORT, DL_GPIO_PIN_18_EDGE_RISE_FALL |
		DL_GPIO_PIN_19_EDGE_RISE_FALL |
		DL_GPIO_PIN_20_EDGE_RISE_FALL |
		DL_GPIO_PIN_24_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_EncoderLeft1_PIN |
		Encoder_EncoderLeft2_PIN |
		Encoder_EncoderRight1_PIN |
		Encoder_EncoderRight2_PIN);
    DL_GPIO_enableInterrupt(Encoder_PORT, Encoder_EncoderLeft1_PIN |
		Encoder_EncoderLeft2_PIN |
		Encoder_EncoderRight1_PIN |
		Encoder_EncoderRight2_PIN);

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
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 1);

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
static const DL_TimerG_ClockConfig gtaskTimerClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * taskTimer_INST_LOAD_VALUE = (10 ms * 40000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gtaskTimerTimerConfig = {
    .period     = taskTimer_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_taskTimer_init(void) {

    DL_TimerG_setClockConfig(taskTimer_INST,
        (DL_TimerG_ClockConfig *) &gtaskTimerClockConfig);

    DL_TimerG_initTimerMode(taskTimer_INST,
        (DL_TimerG_TimerConfig *) &gtaskTimerTimerConfig);
    DL_TimerG_enableInterrupt(taskTimer_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(taskTimer_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000 Hz = 4000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gmsTimerClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * msTimer_INST_LOAD_VALUE = (1 ms * 40000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gmsTimerTimerConfig = {
    .period     = msTimer_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_msTimer_init(void) {

    DL_TimerG_setClockConfig(msTimer_INST,
        (DL_TimerG_ClockConfig *) &gmsTimerClockConfig);

    DL_TimerG_initTimerMode(msTimer_INST,
        (DL_TimerG_TimerConfig *) &gmsTimerTimerConfig);
    DL_TimerG_enableInterrupt(msTimer_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(msTimer_INST_INT_IRQN, 0);
    DL_TimerG_enableClock(msTimer_INST);





}



static const DL_UART_Main_ClockConfig gBluetoothClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gBluetoothConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_Bluetooth_init(void)
{
    DL_UART_Main_setClockConfig(Bluetooth_INST, (DL_UART_Main_ClockConfig *) &gBluetoothClockConfig);

    DL_UART_Main_init(Bluetooth_INST, (DL_UART_Main_Config *) &gBluetoothConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9598.08
     */
    DL_UART_Main_setOversampling(Bluetooth_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(Bluetooth_INST, Bluetooth_IBRD_4_MHZ_9600_BAUD, Bluetooth_FBRD_4_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(Bluetooth_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(Bluetooth_INST_INT_IRQN, 1);


    DL_UART_Main_enable(Bluetooth_INST);
}

/* infraredADC Initialization */
static const DL_ADC12_ClockConfig ginfraredADCClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_infraredADC_init(void)
{
    DL_ADC12_setClockConfig(infraredADC_INST, (DL_ADC12_ClockConfig *) &ginfraredADCClockConfig);

    DL_ADC12_initSeqSample(infraredADC_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_02, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(infraredADC_INST, infraredADC_ADCMEM_infraredLeft,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(infraredADC_INST, infraredADC_ADCMEM_infraredCenter,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(infraredADC_INST, infraredADC_ADCMEM_infraredRight,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(infraredADC_INST,400);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(infraredADC_INST,(DL_ADC12_INTERRUPT_MEM2_RESULT_LOADED));
    DL_ADC12_enableInterrupt(infraredADC_INST,(DL_ADC12_INTERRUPT_MEM2_RESULT_LOADED));
    NVIC_SetPriority(infraredADC_INST_INT_IRQN, 1);
    DL_ADC12_enableConversions(infraredADC_INST);
}

