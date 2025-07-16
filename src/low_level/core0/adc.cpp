#include "adc.h"
#include "low_level/core0.h"
#include "esp32-hal-log.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include "soc/soc_caps.h"
#include <Arduino.h>
#include <cstdint>
#include <atomic>

static TaskHandle_t core0_task_handle;
static const char* TAG = "ADC";

#define NUM_CHANNELS 8

std::atomic<uint16_t> atomic_adc_vals_convolved[NUM_CHANNELS];

// Function to retrieve post convolution values in an inter-core safe way.
uint16_t get_convolved_value(uint16_t channel) {
    uint16_t value;
    if (channel < 8) {
        // give 'em the atomic value!
        value = atomic_adc_vals_convolved[channel].load(std::memory_order_relaxed);
        return value;
    } else {
        // Invalid channel...
        return 10;
    }
}

volatile uint16_t conversion_count = 0;

uint16_t get_adc_conversion_count() {return conversion_count;}
void reset_adc_conversion_count() {conversion_count = 0;}

static const uint32_t DATA_BUFFER_SIZES[NUM_CHANNELS] = {
    1,
    1,
    1,
    1,
    256,
    256,
    1,
    1,
};


struct channel_data_buffer_t {
    uint16_t *data;
    uint16_t i;
};


static channel_data_buffer_t data_buffer[NUM_CHANNELS];


inline uint16_t get_data_buf_val(uint16_t channel, uint16_t i) {
    if (channel < 8) {
        return data_buffer[channel].data[(data_buffer[channel].i + i) % DATA_BUFFER_SIZES[channel]];
    }

    return 1;
}


/*
    Notes on the convolutions!

    The ADC values and convolution values are 16 bits,
    which means that the maximum possible value is about 2 ^ (16 + 16) = 2^32.
    This means that a 64 bit integer is guaranteed to store the value successfully,
    and will be able to store summations of many values as long as they are both
    not the maximum value.

    So, the convolutions will be done as integers, 16 bit times 16 bit summed up and
    stored in a 32 bit value. The output will be converted back to a 16 bit value by
    keeping only the upper 16 bits of the 32 bit value.
*/

// Convolution values to convolve.
uint16_t * convolvers[NUM_CHANNELS];

// Initialize convolvers (write values).
// adc_init() must be called before this to initialize values!!
void convolver_init() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Set all values to 0 initially.
        for (int j = 0; j < DATA_BUFFER_SIZES[i]; j++) {
            convolvers[i][j] = 0;
        }

        // Set last value to 1 for simple last value convolution.
        convolvers[i][DATA_BUFFER_SIZES[i] - 1] = 2 * ((1 << 15) - 1); // set to 2 less than 2^16.
    }
}

// Convolution function
void convolve_all_and_update() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Convolve each buffer with its convolution.
        uint32_t accumulator = 0;
        for (int j = 0; j < DATA_BUFFER_SIZES[i]; j++) {
            accumulator += get_data_buf_val(i, j) * convolvers[i][j];
        }
        atomic_adc_vals_convolved[i].store((uint16_t)(accumulator >> 16), std::memory_order_relaxed);
    }
}

// Both of the below require that channel is between 0 and 7!
void add_buffer_element(uint16_t channel, uint16_t data) {
    if (channel < 8) {
        data_buffer[channel].data[data_buffer[channel].i] = data;
        data_buffer[channel].i = (data_buffer[channel].i + 1) % DATA_BUFFER_SIZES[channel];
    }
}

void print_channel_data(uint16_t channel) {
    if (channel < 7) {
        ESP_LOGI(TAG, "buffer index: %hu", data_buffer[channel].i);
        for (int i = 0; i < DATA_BUFFER_SIZES[channel]; i++) {
            ESP_LOGI(TAG, "buffer channel %hu value %hu: %hu", channel, i, data_buffer[channel].data[i]);
        }
    }
}

static bool IRAM_ATTR adc_conversion_callback(
                                              adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data
                                          ) {
    BaseType_t mustYield = pdFALSE;

    conversion_count += 1;

    vTaskNotifyGiveFromISR(core0_task_handle, &mustYield);
    //mustYield = pdTRUE;

    return (mustYield == pdTRUE);
}


// CALL ONLY ONCE! MEMORY WILL LEAK FOR EACH EXTRA CALL!
adc_continuous_handle_t adc_init(TaskHandle_t conversion_return_taskhandle) {
    core0_task_handle = conversion_return_taskhandle;

    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Allocate data buffers :D
        // Note that memory will leak if called more than once...
        data_buffer[i].data = (uint16_t *) malloc(DATA_BUFFER_SIZES[i] * sizeof(uint16_t));
        data_buffer[i].i = 0;

        convolvers[i] = (uint16_t *) malloc(DATA_BUFFER_SIZES[i] * sizeof(uint16_t));

        // Setup convolvers:
        for (int j = 0; j < DATA_BUFFER_SIZES[i]; j++) {
            convolvers[i][j] = 0;
        }
        convolvers[i][DATA_BUFFER_SIZES[i] - 1] = 1;
    }

    convolver_init();
    
    vTaskDelay(100);

    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {

        .max_store_buf_size = ADC_BUF_SIZE * ADC_READ_LEN * sizeof(adc_digi_output_data_t),

        .conv_frame_size = ADC_READ_LEN * sizeof(adc_digi_output_data_t),
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {

        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,

        .conv_mode = ADC_CONV_SINGLE_UNIT_1,

        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 8;
    for (int i = 0; i < 8; i++) {

        adc_pattern[i].atten = adc_attenuations[i];

        adc_pattern[i].channel = i;

        adc_pattern[i].unit = ADC_UNIT_1;

        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conversion_callback,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    ESP_LOGI(TAG, "Continous ADC now running!");

    return handle;
}

void handle_data(adc_digi_output_data_t *data, uint16_t verbose) {
    uint16_t channel = 0;
    uint16_t channel_count = 0;
    uint16_t prev_channel = data[0].type1.channel;
    size_t i = i;

    while (1) {
        channel = data[i].type1.channel;
        if (channel == prev_channel) {
            channel_count++;
        } else {
            if (channel_count > 1) {
                if (verbose < 8 && prev_channel == verbose) {
                    ESP_LOGI(TAG, "adding point %hu to channel %hu.", data[i - 1].type1.data, prev_channel);
                }
                add_buffer_element(prev_channel, data[i - 1].type1.data);
            }
            channel_count = 0;
        }

        i++;
        if (i >= ADC_READ_LEN) {
            break;
        }
    }
    
}
