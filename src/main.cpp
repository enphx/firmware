#include <Arduino.h>
#include "include/low_level.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_continuous.h"
#include "low_level/tapefollowingsensor.h"


TapeFollowingSensor tapeFollowingSensor;
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  delay(1000);

  low_level_init();

}

void loop() {
  tapeFollowingSensor.update();
  Serial.println(tapeFollowingSensor.getError());
  // print_adc_vals();
  //Serial.println(SOC_ADC_PATT_LEN_MAX);
  // adc_init(core0_task_handle);
  vTaskDelay(10);
  
}


