#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bmi270.h"
#include "common.h"

#include "imu_calibration.h"

int main( void ) {

  /* Sensor Initialization with Zero Values */
  struct bmi2_dev bmi2_dev = { 0 }; 

  /* Save to NVM Flag */
  bool save_to_nvm = false;  

  /* Status Return of the API */
  int8_t result;

  /* I2C Interface Initialization */
  result = bmi2_interface_init( &bmi2_dev, BMI2_I2C_INTF );
  bmi2_error_codes_print_result( result );

  /* BMI270 Initialization */
  result = bmi270_init( &bmi2_dev );
  bmi2_error_codes_print_result( result );

  if ( result == BMI2_OK ) {
    
    /* Perform Comprehensive Calibration */
    result = perform_comprehensive_calibration( &bmi2_dev, save_to_nvm );

    if ( result == BMI2_OK ) {
      
      printf("\n ===== Calibration Sequence Completed Successfully ===== \n");

    } else {
      
      printf("\n ===== Calibration Sequence with Error: %d ===== \n",
             result);
    }
  }

  return result;

}

static int8_t perform_comprehensive_calibration( struct bmi2_dev *bmi2_dev, bool save_to_nvm ) {
  
  int8_t result;

  uint8_t sens_list[ 2 ] = { BMI2_ACCEL, BMI2_GYRO };
  uint8_t status;
  uint8_t internal_status;

  result = bmi2_sensor_enable( sens_list, 2, bmi2_dev );
  bmi2_error_codes_print_result( result );

  if ( result != BMI2_OK ) {
    
    printf("Failed to enable sensors\n");
    
    return result;

  }

  /* Delay Sensors for Stabilization Purposes */
  bmi2_dev->delay_us( 100000, bmi2_dev->intf_ptr );

  /* Check sensor status */
  result = bmi2_get_status( &status, bmi2_dev );

  if ( result == BMI2_OK ) {
  
    printf("\n Sensor Status: 0x%02X\n", status);
    
    printf("Accelerometer Ready: %s\n",  (status & BMI2_DRDY_ACC) ? "Yes" : "No");
    printf("Gyroscope Ready: %s\n",      (status & BMI2_DRDY_GYR) ? "Yes" : "No");
    printf("Command ready: %s\n",        (status & BMI2_CMD_RDY)  ? "Yes" : "No");

  } else {
    
    printf("Failed to get sensor status\n");
    return result;

  }

  /* Check internal status */
  result = bmi2_get_internal_status(&internal_status, bmi2_dev);
  if (result == BMI2_OK) {
    printf("\nInternal Status: 0x%02X\n", internal_status);

    if (internal_status == BMI2_INIT_OK) {
      printf("BMI270 initialization OK\n");
    } else {
      printf("BMI270 status: ");
      if (internal_status & BMI2_NOT_INIT) printf("NOT INITIALIZED ");
      if (internal_status & BMI2_INIT_ERR) printf("INIT ERROR ");
      if (internal_status & BMI2_DRV_ERR) printf("DRIVER ERROR ");
      if (internal_status & BMI2_SNS_STOP) printf("SENSOR STOPPED ");
      if (internal_status & BMI2_NVM_ERROR) printf("NVM ERROR ");
      if (internal_status & BMI2_START_UP_ERROR) printf("STARTUP ERROR ");
      if (internal_status & BMI2_COMPAT_ERROR) printf("COMPATIBILITY ERROR ");
      if (internal_status & BMI2_VFM_SKIPPED) printf("VFM SKIPPED ");
      if (internal_status & BMI2_AXES_MAP_ERROR) printf("AXES MAP ERROR ");
      if (internal_status & BMI2_ODR_50_HZ_ERROR) printf("ODR 50HZ ERROR ");
      if (internal_status & BMI2_ODR_HIGH_ERROR) printf("ODR HIGH ERROR ");
      printf("\n");
    }
  } else {

    printf("Failed to get internal status.\n");
    
    return result;
  }

  /***************** STEP 1: REGISTER STATE BEFORE CALIBRATION
   * *****************/
  printf("\n==== REGISTER VALUES BEFORE CALIBRATION ====\n");
  print_calibration_registers(bmi2_dev);

  /************************* STEP 2: PERFORM ACCEL FOC
   * *************************/
  printf("\n==== PERFORMING ACCELEROMETER FOC ====\n");
  printf(
      "Place the sensor flat with Z-axis pointing upward and keep it stable\n");
  printf("Press Enter to continue...\n");
  getchar();
  getchar();  // Consume extra newline

  result = perform_accel_foc(bmi2_dev);

  if (result == BMI2_OK) {
    printf("Accelerometer FOC completed successfully\n");

    /* Print register values after FOC */
    printf("\n==== REGISTER VALUES AFTER ACCEL FOC ====\n");
    print_calibration_registers(bmi2_dev);
  } else {
    printf("Accelerometer FOC failed with error code: %d\n", result);
    return result;
  }

  /************************* STEP 3: PERFORM GYRO FOC
   * **************************/
  printf("\n==== PERFORMING GYROSCOPE FOC ====\n");
  printf(
      "Keep the sensor completely steady. Do not move the sensor during "
      "FOC!\n");
  printf("Press Enter to continue...\n");
  getchar();

  result = perform_gyro_foc(bmi2_dev);

  if (result == BMI2_OK) {
    printf("Gyroscope FOC completed successfully\n");

    /* Print register values after FOC */
    printf("\n==== REGISTER VALUES AFTER GYRO FOC ====\n");
    print_calibration_registers(bmi2_dev);
  } else {
    printf("Gyroscope FOC failed with error code: %d\n", result);
    printf("Continuing with CRT despite FOC failure...\n");
  }

  /************************* STEP 4: PERFORM GYRO CRT
   * **************************/
  printf("\n==== PERFORMING GYROSCOPE CRT ====\n");
  printf(
      "Keep the sensor completely steady. Do not move the sensor during "
      "CRT!\n");
  printf("Press Enter to continue...\n");
  getchar();

  result = perform_gyro_crt(bmi2_dev);

  if (result == BMI2_OK) {
    printf("CRT successfully completed.\n");

    /* Print register values after CRT */
    printf("\n==== REGISTER VALUES AFTER GYRO CRT ====\n");
    print_calibration_registers(bmi2_dev);

    /***************** STEP 5: OPTIONALLY SAVE CALIBRATION TO NVM
     * *****************/
    if (save_to_nvm) {
      printf("\n==== SAVING CALIBRATION TO NVM ====\n");
      printf("WARNING: This operation writes to non-volatile memory\n");
      printf("Press Enter to continue or Ctrl+C to abort...\n");
      getchar();

      // result = store_calibration_to_nvm(bmi2_dev);

      if (result == BMI2_OK) {
        printf("Calibration values saved to NVM successfully.\n");
      } else {
        printf("Failed to save calibration to NVM, error code: %d\n", result);
      }
    } else {
      printf("\n==== NVM WRITE DISABLED ====\n");
      printf("Calibration values are NOT being saved to NVM.\n");
      printf(
          "To save values to NVM, set save_to_nvm = true and run the program "
          "again.\n");
    }
  } else {
    bmi2_error_codes_print_result(result);
  }

  return result;
}

static void print_calibration_registers( struct bmi2_dev *bmi2_dev ) {
  
  int8_t result;

  /* Read Accelerometer Offset Registers */
  uint8_t accel_offset[3];
  result = bmi2_get_accel_offset_comp(accel_offset, bmi2_dev);
  bmi2_error_codes_print_result(result);

  /* Read gyroscope offset values */
  struct bmi2_sens_axes_data gyro_offset;
  result = bmi2_read_gyro_offset_comp_axes(&gyro_offset, bmi2_dev);
  bmi2_error_codes_print_result(result);

  /* Read if offset compensation is enabled for accel and gyro */
  uint8_t nv_conf, gyro_off_en, gyro_gain_en;

  /* Check accel offset compensation enabled status */
  result = bmi2_get_regs(BMI2_NV_CONF_ADDR, &nv_conf, 1, bmi2_dev);
  uint8_t accel_off_en = BMI2_GET_BITS(nv_conf, BMI2_NV_ACC_OFFSET);

  /* Check gyro offset compensation enabled status */
  result = bmi2_get_gyro_offset_comp(&gyro_off_en, bmi2_dev);

  /* Check gyro gain compensation enabled status */
  result = bmi2_get_gyro_gain(&gyro_gain_en, bmi2_dev);

  /* Read gyro cross-sensitivity coefficient */
  struct bmi2_dev dev_copy =
      *bmi2_dev;  // Create a copy to access gyr_cross_sens_zx
  result = bmi2_get_gyro_cross_sense(&dev_copy);
  int8_t gyr_cross_sense_zx = dev_copy.gyr_cross_sens_zx;

  /* Read current sensor data to verify calibration effect */
  struct bmi2_sens_data sensor_data = {0};
  result = bmi2_get_sensor_data(&sensor_data, bmi2_dev);

  /* Read gyro user gain status */
  struct bmi2_gyr_user_gain_status gyr_user_gain_status;
  struct bmi2_feat_sensor_data feat_sensor_data;
  feat_sensor_data.type = BMI2_GYRO_GAIN_UPDATE;
  result = bmi2_get_feature_data(&feat_sensor_data, 1, bmi2_dev);
  if (result == BMI2_OK) {
    gyr_user_gain_status = feat_sensor_data.sens_data.gyro_user_gain_status;
  }

  /* Print all values */
  printf("\n===== Calibration Registers and Values =====\n");

  printf("\nAccelerometer Offset Values (mg):\n");
  printf("X: %d (raw: 0x%02X)\n", (int8_t)accel_offset[0] * 39 / 10,
         accel_offset[0]);
  printf("Y: %d (raw: 0x%02X)\n", (int8_t)accel_offset[1] * 39 / 10,
         accel_offset[1]);
  printf("Z: %d (raw: 0x%02X)\n", (int8_t)accel_offset[2] * 39 / 10,
         accel_offset[2]);

  printf("\nGyroscope Offset Values (mdps):\n");
  printf("X: %d\n", gyro_offset.x * 61 / 1000);
  printf("Y: %d\n", gyro_offset.y * 61 / 1000);
  printf("Z: %d\n", gyro_offset.z * 61 / 1000);

  printf("\nGyroscope Cross Sensitivity Value:\n");
  printf("ZX: %d\n", gyr_cross_sense_zx);

  if (result == BMI2_OK) {
    printf("\nGyroscope User Gain Status:\n");
    printf("Saturation X: %d\n", gyr_user_gain_status.sat_x);
    printf("Saturation Y: %d\n", gyr_user_gain_status.sat_y);
    printf("Saturation Z: %d\n", gyr_user_gain_status.sat_z);
    printf("G-Trigger Status: %d\n", gyr_user_gain_status.g_trigger_status);
  }

  printf("\nOffset/Gain Compensation Status:\n");
  printf("- Accelerometer offset compensation: %s\n",
         accel_off_en ? "ENABLED" : "DISABLED");
  printf("- Gyroscope offset compensation: %s\n",
         gyro_off_en ? "ENABLED" : "DISABLED");
  printf("- Gyroscope gain compensation: %s\n",
         gyro_gain_en ? "ENABLED" : "DISABLED");

  printf("\nCurrent Sensor Readings (raw values):\n");
  printf("Accelerometer: X=%d, Y=%d, Z=%d\n", sensor_data.acc.x,
         sensor_data.acc.y, sensor_data.acc.z);
  printf("Gyroscope: X=%d, Y=%d, Z=%d\n", sensor_data.gyr.x, sensor_data.gyr.y,
         sensor_data.gyr.z);

  printf("\n=========================================\n");
}

static int8_t perform_accel_foc( struct bmi2_dev *bmi2_dev ) {
  
  int8_t result;

  /* Set Accelerometer Axis and Sign (assume the Z-axis is pointing upwards for + 1g) */
  struct bmi2_accel_foc_g_value g_value = { 0 };
  g_value.x = 0;
  g_value.y = 0;
  g_value.z = 1;    /* Z-axis Selected for +1g FOC */
  g_value.sign = 0; /* Positive sign */

  /* Enable Accelerometer Offset Compensation */
  uint8_t offset_en = BMI2_ENABLE;
  
  result = bmi2_set_accel_offset_comp( offset_en, bmi2_dev );
  bmi2_error_codes_print_result( result );

  if ( result != BMI2_OK ) {
    
    printf("Failed to enable accelerometer offset compensation.\n");
    return result;

  }

  bmi2_dev->delay_us( 5000, bmi2_dev->intf_ptr );

  /* Perform Accelerometer FOC */
  printf("Performing Accelerometer FOC...\n");
  
  result = bmi2_perform_accel_foc( &g_value, bmi2_dev );
  bmi2_error_codes_print_result( result );

  bmi2_dev->delay_us( 5000, bmi2_dev->intf_ptr );

  return result;

}

static int8_t perform_gyro_foc( struct bmi2_dev *bmi2_dev ) {
  int8_t result;


  printf("Performing Gyroscope FOC ...\n");
  printf("Please keep the sensor stationary during calibration\n");

  /* Enable Offset Compensation before performing Gyroscope FOC */
  uint8_t offset_en = BMI2_ENABLE;

  result = bmi2_set_gyro_offset_comp( offset_en, bmi2_dev );
  bmi2_error_codes_print_result( result );

  if ( result != BMI2_OK ) {
    
    printf("Failed to enable gyroscope offset compensation\n");
    return result;

  }

  bmi2_dev->delay_us(5000, bmi2_dev->intf_ptr);

  /* Perform Gyroscope FOC */
  result = bmi2_perform_gyro_foc( bmi2_dev );
  bmi2_error_codes_print_result( result );

  bmi2_dev->delay_us(50000, bmi2_dev->intf_ptr);

  return result;

}

static int8_t perform_gyro_crt( struct bmi2_dev *bmi2_dev ) {
  
  int8_t result;

  printf("Starting CRT process (this will take approximately 500ms)\n");
  
  result = bmi2_do_crt( bmi2_dev );
  bmi2_error_codes_print_result( result );

  if ( result == BMI2_OK ) {
    
    printf(" CRT process completed successfully. \n");

    /* Check Gyroscope Self-test Status */
    struct bmi2_gyro_self_test_status gyro_st_status;
    uint8_t status_reg;

    result = bmi2_get_regs(BMI2_GYR_CRT_CONF_ADDR, &status_reg, 1, bmi2_dev);

  }

  return result;

}

static int8_t store_calibration_to_nvm( struct bmi2_dev *bmi2_dev ) {
  
  int8_t result = BMI2_OK;
  result = bmi2_nvm_prog( bmi2_dev );
  
  return result;

}