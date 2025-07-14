/**
 * @file main.h
 * @brief
*/
#ifndef __MAIN_H__
#define __MAIN_H__

#pragma once

#include <stdint.h>

struct _SensorData {
  /* Accelerometer Raw Data in g */
  float ax, ay, az;  
  /* Gyroscope Raw Data in Degrees per Second */
  float gx, gy, gz;
  /* Magnetometer Raw Data in uT */  
  float mx, my, mz;  
  /* Accelerometer, Gyroscope & Magnetometer Timestamps in us */
  uint32_t aTimeStamp, gTimeStamp, mTimeStamp;
};

struct _LinearAcceleration {
  /* Linear Acceleration X Component in m/s^2 */
  float linear_ax;
  /* Linear Acceleration Y Component in m/s^2 */
  float linear_ay;
  /* Linear Acceleration Z Component in m/s^2 */
  float linear_az;
  /* Linear Acceleration Reading Timestamp in us */
  uint32_t laTimestamp;
};

struct _EulerAngles {
  float roll, pitch, yaw;
  /* Euler Angles Reading Timestamp in us */
  uint32_t eaTimestamp;
};

struct _Quaternion {
  /* Quaternion W Component */
  float q0;
  /* Quaternion X Component */
  float q1;
  /* Quaternion Y Component */
  float q2;
  /* Quaternion Z Component */
  float q3;
  /* Quaternion Reading Timestamp in us */
  uint32_t qTimeStamp;
};

struct _TemperatureData {
  /* Temperature in Degrees Centigrade */
  float temperature;
  /* Temperature Reading Timestamp in us */
  uint32_t timeStamp;
};

/**
 * @brief Performs a full calibration of the sensor system.
 *
 * This function executes all necessary calibration routines for the sensors,
 * including accelerometer, gyroscope, and magnetometer. It ensures that
 * sensor biases and offsets are properly determined and applied, improving
 * the accuracy of sensor readings.
 *
 * @note This function may take several seconds to complete, depending on the
 *       calibration procedures implemented.
 */
void fullCalibration();

/**
 * @brief Normalizes the sensor biases.
 *
 * This function adjusts the biases of the sensors to ensure that their readings
 * are centered and consistent. It is typically called after calibration routines
 * to apply the calculated bias corrections.
 *
 * @note This function should be called after fullCalibration() to ensure
 *       optimal sensor performance.
 */
void normalizeBiases();


#endif // __MAIN_H__