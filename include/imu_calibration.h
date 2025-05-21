/*!
 * @file         imu_calibration.h
 *
 * @brief       Definitions for imu_calibration.cpp
 *
 */

#ifndef __IMU_CALIBRATION_H__
#define  __IMU_CALIBRATION_H__

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief This function reads and prints important register values
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 */
static void print_calibration_registers(struct bmi2_dev *bmi2_dev);

/*!
 * @brief This function performs comprehensive calibration routines including
 *        accelerometer FOC, gyroscope FOC, and CRT
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 * @param[in] save_to_nvm : Flag to control NVM storage of calibration values
 * 
 * @return Status of execution
 */
static int8_t perform_comprehensive_calibration(struct bmi2_dev *bmi2_dev, bool save_to_nvm);

/*!
 * @brief This function performs accelerometer FOC calibration
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 * 
 * @return Status of execution
 */
static int8_t perform_accel_foc(struct bmi2_dev *bmi2_dev);

/*!
 * @brief This function performs gyroscope FOC calibration
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 * 
 * @return Status of execution
 */
static int8_t perform_gyro_foc(struct bmi2_dev *bmi2_dev);

/*!
 * @brief This function performs gyroscope CRT calibration
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 * 
 * @return Status of execution
 */
static int8_t perform_gyro_crt(struct bmi2_dev *bmi2_dev);

/*!
 * @brief This function stores calibration values to NVM
 *
 * @note  Writing to NVM should only be done when necessary, as NVM has limited write cycles
 * 
 * @param[in] bmi2_dev  : Structure instance of bmi2_dev
 * 
 * @return Status of execution
 */
static int8_t store_calibration_to_nvm(struct bmi2_dev *bmi2_dev);

#endif /* __IMU_CALIBRATION_H__ */