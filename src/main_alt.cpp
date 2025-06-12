#include <Arduino.h>
#include <Wire.h>
#include <I2C.h>

#include "bmi270.h"
#include "bmm150.h"

void setup () {
  
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz

  struct bmi2_dev   bmi2_dev = {0};
  struct bmm150_dev bmm1     = {0};

  uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

  int8_t result = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
  bmi2_error_codes_print_result(result);

  result = bmi270_init(&bmi2_dev);
  bmi2_error_codes_print_result(result);
  
  if (result == BMI2_OK) {

    Serial.println("The accelerometer has been initialized successfully.");

  } else {

    Serial.print("The accelerometer initialization failed with error code: ");
    Serial.println(result);
    
    while (1);

  }
}

void loop () {

  ;;

}