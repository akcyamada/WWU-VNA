/*
 * adc14.h
 *
 *  Created on: May 2, 2017
 *      Author: frohro
 */

#ifndef ADC14VNA_H_
#define ADC14VNA_H_
//#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/hal/Hwi.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


#define SMCLK_FREQUENCY     12000000

#define SAMPLE_FREQUENCY    7956
#define SAMPLE_LENGTH       3840/2  // 8192 is too big.
#define F_IF 285
#define OMEGA_IF F_IF*TWO_PI
#define SAMPLES_IN_ONE_CYCLE SAMPLE_FREQUENCY/F_IF
/* Make sure these above are such that SAMPLE_FREQUENCY/OMEGA_IF/4 is an integer
 * so that there are an integer number of points in a quarter cycles of the IF frequency.
 */
void startConversion(void);

int adc14_main(void);

void DMA_INT1_IRQHandler(void);

#endif /* ADC14VNA_H_ */
