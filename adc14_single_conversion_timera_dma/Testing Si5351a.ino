#include "si5351.h"
#include "Wire.h"
extern "C"{
#include "adc14vna.h"
};
#include "driverlib.h"

// Were uint16_t
volatile float ref[SAMPLE_LENGTH];
volatile float meas[SAMPLE_LENGTH];
volatile uint16_t ref_uint16_t[SAMPLE_LENGTH];
volatile uint16_t meas_uint16_t[SAMPLE_LENGTH];
volatile unsigned short ref_ushort[SAMPLE_LENGTH];
volatile unsigned short meas_ushort[SAMPLE_LENGTH];
extern volatile bool doneADC;

Si5351 si5351;

void setup() {
  // put your setup code here, to run once:
    si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); // 0,6,8,10pf
    si5351.set_correction(-25800); //-26550
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    si5351.set_freq(500000000ULL, SI5351_CLK0);
    si5351.set_freq(500000000ULL, SI5351_CLK1);
    //si5351.set_freq(1000000000ULL, SI5351_CLK2);

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
