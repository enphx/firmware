#include <Arduino.h>
#include "esp_log.h"
#include "include/convfilter.h"

const char * TAG = "MAIN";


#define KERN_SIZE_1 3
int32_t kern1[KERN_SIZE_1] = {
  -1,
  0,
  1
};

ConvFilter<int32_t, KERN_SIZE_1> conv_filter1(kern1);


#define KERN_SIZE_2 5
int32_t kern2[KERN_SIZE_2] {
  -2,
  -1,
  0,
  1,
  2
};

ConvFilter<int32_t, KERN_SIZE_2> conv_filter2(kern2);


#define TEST_DATA_1_SIZE 21
int32_t test_data_1[TEST_DATA_1_SIZE] = {
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  2,
  3,
  4,
  5,
  10,
  11,
  12,
  13,
  14,
  4,
  5,
  6,
  7,
  8
};


#define TEST_DATA_2_SIZE 23
int32_t test_data_2[TEST_DATA_2_SIZE] = {
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1020,
  910,
  1004,
  850,
  600,
  200,
  210,
  240,
  234,
  500,
  550,
  600,
  1200,
  1210,
  1190,
  1100
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(100);
}



void loop() {
  Serial.println("\n\n\n\n\n\n==== START TESTS ====\n\nTesting dataset 1:");
  for (int i = 0; i < TEST_DATA_1_SIZE; i++) {
    Serial.print("data: ");
    Serial.print(test_data_1[i]);
    // Serial.print(", ");
    Serial.print(", kern1: ");
    Serial.print(conv_filter1.eval(test_data_1[i]));
    // Serial.print(", ");
    Serial.print(", kern2: ");
    Serial.println(conv_filter2.eval(test_data_1[i]));
  }

  Serial.println("\n\nTesting dataset 2:");
  for (int i = 0; i < TEST_DATA_2_SIZE; i++) {
    Serial.print("data: ");
    Serial.print(test_data_2[i]);
    // Serial.print(", ");
    Serial.print(", kern1: ");
    Serial.print(conv_filter1.eval(test_data_2[i]));
    // Serial.print(", ");
    Serial.print(", kern2: ");
    Serial.println(conv_filter2.eval(test_data_2[i]));
  }
  Serial.println("\n\n==== STOP TESTS ====");
  vTaskDelay(1000);
}
