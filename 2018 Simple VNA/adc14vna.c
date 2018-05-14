/*
 * -------------------------------------------
 *    MSP432 DriverLib - v4_00_00_11
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 ADC14 - Multiple Channel Sample without Repeat
 *
 * Description: In this code example, the feature of being able to scan multiple
 * ADC channels is demonstrated by the user a the DriverLib APIs.  Conversion
 * memory registers ADC_MEM0 - ADC_MEM3 are configured to read conversion
 * results from A6, A12, A10, A8 respectively. Conversion is enabled and then sampling is
 * toggled using a software toggle. Repeat mode is not enabled and sampling only
 * occurs once (and it is expected that the user pauses the debugger to observe
 * the results). Once the final sample has been taken, the interrupt for
 * ADC_MEM3 is triggered and the result is stored in the resultsBuffer buffer.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P4.0  |<--- A13 (Analog Input, Measured)
 *            |            P6.1  |<--- A14 (Analog Input, Reference)
 *            |                  |
 *            |                  |
 *
 *
 *
 * Author: Timothy Logan
 * This was modified by Rob Frohne to do multiple ADC at 8 kHz sample rate,
 * and in concert with Energia.
 ******************************************************************************/
#include "adc14vna.h"
/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

void ADC14_IRQHandler(void);
/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_UpModeConfig upModeConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 48MHz
 (SMCLK_FREQUENCY/SAMPLE_FREQUENCY),  // Number of counts
 TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
 TIMER_A_DO_CLEAR                     // Clear Counter
};

/* Timer_A Compare Configuration Parameter */
const Timer_A_CompareModeConfig compareConfig =
{
 TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
 TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
 TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output bit
 (SMCLK_FREQUENCY/SAMPLE_FREQUENCY)
};

static volatile uint16_t resultsBuffer[8];
extern volatile uint16_t ref[SAMPLE_LENGTH];
extern volatile uint16_t meas[SAMPLE_LENGTH];
extern volatile bool doneADC = false;

int adc14_main(void)
{
    int i;

    /* This stuff added by Rob to make sure we are ok at 48 MHz.  It is probably not
     * needed, but might help.
     */
    /*
     * Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency
     */
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    /*
     * Revision C silicon supports wait states of 1 at 48Mhz
     */
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    /*
     * Setting up clocks
     * MCLK = MCLK = 48MHz
     * SMCLK = MCLK/2 = 24Mhz
     * ACLK = REFO = 32Khz
     */
    CS_setDCOFrequency(48000000);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* End: this stuff added by Rob to make sure we are ok at 48 MHz.  It is probably not
     * needed, but might help.
     */

    // Register interrupt (sets up IRQ vectors) using TI-RTOS which Energia uses.
    Hwi_Params params;
    Hwi_Params_init(&params);
    Hwi_create(INT_ADC14, ADC14_IRQHandler, &params, 0);
    Hwi_setPriority(INT_ADC14, 60);

    // Configuring debugging pins as output for debugging...
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    //Zero-filling buffer
    memset(resultsBuffer, 0x00, 8);

    // Setting reference voltage to 2.5  and enabling reference
    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    REF_A_enableReferenceVoltage();

    //Initializing ADC (MCLK/1/1)
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                     0);

    //Configuring GPIOs for Analog In
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
         GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
         GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    // Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A13, A14)  with no repeat)
    // with internal 2.5v reference
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    ADC14_configureConversionMemory(ADC_MEM0,
                                    ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM1,
                                    ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    //  Enabling the interrupt when a conversion on channel 1 (end of sequence)
    //  is complete and enabling conversions
    ADC14_enableInterrupt(ADC_INT1);

    // This may need to be increased if the sample capacitor doesn't charge in time.
    // This will be controlled by the reference channel, because it has higher resistance.
    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4);

    // Configuring Timer_A in continuous mode as setup in upModeConfig above
    Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    // Configuring Timer_A0 in CCR1 to trigger as set in compareConfig above
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    // Configuring the sample trigger to be sourced from Timer_A0  and setting it
    // to automatic iteration after it is triggered
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    // Setting up the sample timer to automatically step through the sequence
    // convert.
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // Enabling Interrupts
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    // Starting the Timer
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // Initialize results arrays and done flag.
    for (i=0; i<SAMPLE_LENGTH; i++)
    {
        ref[i] = 0;
        meas[i] = 0;
    }
    doneADC = false;
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    static int i = 0;
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    if(status & ADC_INT1)
    {
        doneADC = false;
        ADC14_getMultiSequenceResult(resultsBuffer);
        meas[i] = resultsBuffer[0];
        ref[i] = resultsBuffer[1];
        GPIO_toggleOutputOnPin(GPIO_PORT_P5, GPIO_PIN5);
        ADC14_disableConversion();
        if (i!=SAMPLE_LENGTH)
        {
            i++;
            ADC14_enableConversion();
        }
        else
        {
            i=0;
            GPIO_toggleOutputOnPin(GPIO_PORT_P5, GPIO_PIN4);
            doneADC = true;
        }
    }
}

