#include "core0.h"
#include <stdio.h>
#include "core0/adc.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/idf_additions.h"
#include <Arduino.h>

static TaskHandle_t core0_task_handle;
static const char* TAG = "CORE0";
adc_continuous_handle_t adc_handle = NULL;

void core0_task(void *data) {
  vTaskDelay(100);
  ESP_LOGI(TAG, "init...");

  esp_err_t esp_err;
  uint32_t adc_data_len = 0;
  adc_digi_output_data_t adc_data[ADC_READ_LEN] = {0};
  memset(adc_data, 0xcc, ADC_READ_LEN * sizeof(adc_digi_output_data_t));

  bool reading = false;

  while (1) {

    // Wait for ADC conversion to occur...
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    reading = true;

    while (reading) {

      esp_err = adc_continuous_read(
        adc_handle,
        (uint8_t* )adc_data,
        ADC_READ_LEN * sizeof(adc_digi_output_data_t),
        &adc_data_len,
        0
      );

      if (esp_err == ESP_OK) {
        handle_data(adc_data, 9);

        if (get_adc_conversion_count() >= 50) {
          convolve_all_and_update();
          reset_adc_conversion_count();
        }
        // if (get_adc_conversion_count() >= 1000) {
        //   handle_data(adc_data, 7);
        //   print_channel_data(7);

        //   for (int i = 0; i < (ADC_READ_LEN - 1); i++) {
        //     ESP_LOGI(TAG, "i: %i, data: %hu, channel: %hu", i, adc_data[i].type1.data, adc_data[i].type1.channel);
        //   }
        //   reset_adc_conversion_count();

        // } else {
        //   handle_data(adc_data, 9);
        // }

      } else {
       reading = false;

      }
    }

  }
}



void core0_init() {
  xTaskCreatePinnedToCore(core0_task,
                          "core0_task",
                          32768,
                          NULL,
                          2,
                          &core0_task_handle,
                          0
                        );
  adc_handle = adc_init(core0_task_handle);
}
