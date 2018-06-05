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
 * This was modified by Rob Frohne and Jacob Priddy to do multiple ADC at x kHz sample rate,
 * and in concert with Energia.
 ******************************************************************************/
#include "adc14vna.h"

/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[16];

const Timer_A_PWMConfig timerA_PWM =
{
    .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
    .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,
    .timerPeriod =  (SMCLK_FREQUENCY/SAMPLE_FREQUENCY),
    .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
    .compareOutputMode = TIMER_A_OUTPUTMODE_SET_RESET,
    .dutyCycle = 11
};

static volatile bool doneRef = false;
static volatile bool doneMeas = false;
extern volatile uint16_t ref[SAMPLE_LENGTH];
extern volatile uint16_t meas[SAMPLE_LENGTH];
extern volatile bool doneADC = false;

void startConversion(void)
{
    P1->OUT |= BIT0;
    P1IFG &= ~BIT1;
    MAP_DMA_enableChannel(7);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &timerA_PWM);
    P1->OUT &= ~BIT0;
}

int adc14_main(void)
{
    int i;

    // Register interrupt (sets up IRQ vectors) using TI-RTOS which Energia uses.
//    Hwi_Params params;
//    Hwi_Params_init(&params);
//    Hwi_create(INT_DMA_INT1, DMA_INT1_IRQHandler, &params, 0);
//    Hwi_setPriority(INT_DMA_INT1, 60);

//    Hwi_Handle myHwi;
//    Error_Block eb;
//    Error_init(&eb);
//
//    myHwi = Hwi_create(DMA_INT1, DMA_INT1_IRQHandler, NULL, &eb);
//
//    if (myHwi == NULL) {
//        return 1;
//    }

//    Hwi_Params params;
//    Hwi_Params_init(&params);
//    params.priority = 0;
//    Hwi_create(INT_DMA_INT1, DMA_INT1_IRQHandler, &params, NULL);

//    MAP_Interrupt_registerInterrupt(INT_DMA_INT1, DMA_INT1_IRQHandler);

    // Configuring debugging pins as output for debugging...
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    P1OUT &= ~BIT0;

    /*
     * Debug: set TA0.1 as output to see ADC trigger signal
     */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,GPIO_PRIMARY_MODULE_FUNCTION);

    // Setting reference voltage to 2.5  and enabling reference
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    //Initializing ADC (MCLK/1/1)
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                     0);

    //Configuring GPIOs for Analog In
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
         GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
         GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    // Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A13, A14)  with no repeat)
    // with internal 2.5v reference
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
                                    ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
    MAP_ADC14_configureConversionMemory(ADC_MEM1,
                                    ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    //  Enabling the interrupt when a conversion on channel 1 (end of sequence)
    //  is complete and enabling conversions
    MAP_ADC14_enableInterrupt(ADC_INT1);

    // This may need to be increased if the sample capacitor doesn't charge in time.
    // This will be controlled by the reference channel, because it has higher resistance.
    MAP_ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4);

    // Configuring the sample trigger to be sourced from Timer_A0  and setting it
    // to automatic iteration after it is triggered
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    // Setting up the sample timer to automatically step through the sequence
    // convert.
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    MAP_ADC14_enableConversion();



    /* Configuring DMA module */
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);

    /*
     * Setup the DMA + ADC14 interface
     */
    MAP_DMA_disableChannelAttribute(DMA_CH7_ADC14,
                                 UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                 UDMA_ATTR_HIGH_PRIORITY |
                                 UDMA_ATTR_REQMASK);

    /*
     * Setting Control Indexes. In this case we will set the source of the
     * DMA transfer to ADC14 Memory 0 and the destination to the destination
     * data array.
     */
    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
            (void*)meas, SAMPLE_LENGTH);

    MAP_DMA_setChannelControl(UDMA_ALT_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[1],
            (void*)ref, SAMPLE_LENGTH);

    /* Assigning/Enabling Interrupts */
    MAP_DMA_assignInterrupt(DMA_INT1, 7);
    MAP_DMA_assignChannel(DMA_CH7_ADC14);
    MAP_DMA_clearInterruptFlag(7);


    // Initialize results arrays and done flag.
    for (i=0; i<SAMPLE_LENGTH; i++)
    {
        ref[i] = 0;
        meas[i] = 0;
    }
    doneADC = false;

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_DMA_INT1);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    return 0;
}


/* Completion interrupt for ADC14 MEM0 */
/*__attribute__((ramfunc))*/  // Requires compiler TI v15.12.1.LTS
void DMA_INT1_IRQHandler(void)
{
    P1->OUT |= BIT0;
    /*
     * Switch between primary and alternate buffers with DMA's PingPong mode
     */
    if (DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT)
    {
        MSP_EXP432P401RLP_DMAControlTable[7].control =
                (MSP_EXP432P401RLP_DMAControlTable[7].control & 0xff000000 ) |
                (((SAMPLE_LENGTH)-1)<<4) | 0x03;
        doneMeas = true;
    }
    else
    {
        MSP_EXP432P401RLP_DMAControlTable[15].control =
                (MSP_EXP432P401RLP_DMAControlTable[15].control & 0xff000000 ) |
                (((SAMPLE_LENGTH)-1)<<4) | 0x03;
        doneRef = true;
    }

    if(doneRef && doneMeas)
    {
        MAP_Timer_A_stopTimer(TIMER_A0_BASE);
        MAP_DMA_disableChannel(7);
        doneADC = true;
    }

    P1->OUT &= ~BIT0;
}

