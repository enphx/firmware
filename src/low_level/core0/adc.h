#ifndef ADC_H
#define ADC_H
#include "esp_adc/adc_continuous.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"


/*
  CONTINUOUS ADC CHANNEL SETUP! ALL 8 CHANNELS MUST BE USED,
  AS THIS IS AN INVARIANT USED IN OTHER SECTIONS OF CODE.

  The ADC will run at maximum sampling rate (about 2MHz, gotta calibrate it)
  so each channel should run at precisely 250kHz sample rate (2MHz/4).
*/

// Uses only ADC 1 (which is what we want. Would not recommend changing this ever.)
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1

// the master sampling frequency. The sampling frequency
// for each channel will be 8 times less, so at 2MHz master sample
// rate we would have 250kHz sample rate per channel.
// 800kHz would give us 100kHz per channel in theory.
#define ADC_SAMPLE_FREQ_HZ (1000 * 1000 * 2)

// adc read len represents the number of data points we want to read per
// channel in one go (per frame). Each datapoint has a size of 2 bytes,
// so both these values will be multiplied by 2 each time.
// #define ADC_VALS_PER_CONV (8 * 10)
#define ADC_VALS_PER_CONV 64
#define ADC_READ_LEN (ADC_VALS_PER_CONV * 8)

// number of conversions to store in the buffer. This times ADC_READ_LEN
// will be the actual buffer size.
#define ADC_BUF_SIZE 4

#define ADC_UNIT ADC_UNIT_1
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1

// ADC channel enum {write an enum which maps adc channels to pin functions}

// Default attenuation is maximum for close to full 3.3V range.
// Other attenuations are 0, 2_5 (2.5), 6.
// voltage that reads as "0" is about 100mV in all cases.
#define DEFAULT_ADC_ATTEN ADC_ATTEN_DB_12

// Per channel attenuation, in case we want different values for different
// channels.
const uint8_t adc_attenuations[8] = {
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
  DEFAULT_ADC_ATTEN,
};

// The following should only be called on core 0:
adc_continuous_handle_t adc_init(TaskHandle_t conversion_return_taskhandle);
uint16_t get_adc_conversion_count();
void reset_adc_conversion_count();

void handle_data(adc_digi_output_data_t * data, uint16_t verbose);
void print_channel_data(uint16_t channel);

void convolve_all_and_update();
#endif
