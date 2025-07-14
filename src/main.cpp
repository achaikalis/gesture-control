/* @file  main.cpp */

#include "main.h"

#include "MPU9250.h"
#include "MPU9250/QuaternionFilter.h"

#include <BasicLinearAlgebra.h>
#include <Reefwing_xIMU3.h>

MPU9250 imu;
Reefwing_xIMU3 rx;
NetworkAnnouncement networkAnnouncement;

_Quaternion q;
_SensorData sa;
_EulerAngles ea;
_TemperatureData td;
_LinearAcceleration la;

const long displayPeriod = 100;
unsigned long previousMillis = 0;

/* Accelerometer, Gyroscope & Magnetometer Biases */
BLA::Matrix<3> b_accel = {-36.69, 39.03, -77.72};
BLA::Matrix<3> b_gyro  = {4.14, -1.31, -0.23};

BLA::Matrix<3> b_mag   = {244.08, 376.44, 393.14};
BLA::Matrix<3> s_mag   = {1.24, 0.93, 0.90};

void setup() {
  Serial.begin(11520);

  Wire.begin();
  delay(2000);

  /* Set Custom Settings Profile for the MPU9250 */
  MPU9250Setting settings;

  /* Set Calibration Flag */
  bool enableCalibration = false;

  /* Accelerometer Configuration */
  settings.accel_fs_sel   = ACCEL_FS_SEL::A16G;  
  settings.accel_fchoice  = 0x01;
  settings.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  /* Gyroscope Configuration */
  settings.gyro_fs_sel   = GYRO_FS_SEL::G2000DPS;
  settings.gyro_fchoice  = 0x03;
  settings.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;

  /* Magnetometer Configuration */
  settings.mag_output_bits  = MAG_OUTPUT_BITS::M16BITS;

  /* FIFO Configuration */
  settings.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;

  /* MPU2950 Setup */
  imu.setup(0x68, settings);

  /* Enhance AHRS with Temperature Readings (Degrees Centigrade) */
  imu.ahrs(true);

  /* Enable Verbose Output for the Calibration procedure */
  imu.verbose(true);

  /* Set Sensor Fusion Filter and Iterations */
  imu.selectFilter(QuatFilterSel::MAHONY);
  imu.setFilterIterations(40);

  /* Set Biases */
  imu.setAccBias(b_accel(0), b_accel(1), b_accel(2));
  imu.setGyroBias(b_gyro(0),  b_gyro(1),  b_gyro(2));
  
  imu.setMagBias(b_mag(0), b_mag(1), b_mag(2));
  imu.setMagScale(s_mag(0), s_mag(1), s_mag(2));

  /* Set Magnetic Declination */
  imu.setMagneticDeclination(4.89);

  while (!Serial);
  
  if (imu.setup(0x68, settings)) {

    rx.sendNotification("IMU has been initialized successfully!");
    rx.sendNotification("Connection with xIMU3 GUI has been established!");

  } else {
    
    rx.sendError("Failed to initialize IMU!");
    rx.sendError("Failed to establish connection with xIMU3 GUI.");
    
    delay(5000);
    while (1);

  }

  if (enableCalibration) { fullCalibration(); }

}

void loop() {
  if (imu.isConnected() && imu.update()) {

    sa.ax = imu.getAccX();
    sa.ay = imu.getAccY();
    sa.az = imu.getAccZ();
    sa.aTimeStamp = micros();

    sa.gx = imu.getGyroX();
    sa.gy = imu.getGyroY();
    sa.gz = imu.getGyroZ();
    sa.gTimeStamp = micros();

    sa.mx = imu.getMagX();
    sa.my = imu.getMagY();
    sa.mz = imu.getMagZ();
    sa.mTimeStamp = micros();

  }

  /* Get Linear Acceleration */
  la.linear_ax = imu.getLinearAccX();
  la.linear_ay = imu.getLinearAccY();
  la.linear_az = imu.getLinearAccZ();
  la.laTimestamp = micros();

  /* Get Euler Angles */
  ea.roll  = imu.getRoll();
  ea.pitch = imu.getPitch();
  ea.yaw   = imu.getYaw();
  ea.eaTimestamp = micros();

  /* Get Quaternions */
  q.q0 = imu.getQuaternionW();
  q.q1 = imu.getQuaternionX();
  q.q2 = imu.getQuaternionY();
  q.q3 = imu.getQuaternionZ();
  q.qTimeStamp = micros();

  /* millis() - previousMillis >= displayPeriod */
  // if (millis() - previousMillis >= displayPeriod)
  if (millis() > previousMillis + 25) {
    
    /* ( Inertial Data --> { Accelerometer, Gyroscope} ) */
    Serial.print("I,");
    
    Serial.print(sa.gTimeStamp);
    Serial.print(",");

    Serial.print(sa.gx, 4);
    Serial.print(",");

    Serial.print(sa.gy, 4);
    Serial.print(",");

    Serial.print(sa.gz, 4);
    Serial.print(",");

    Serial.print(sa.ax, 4);
    Serial.print(",");

    Serial.print(sa.ay, 4);
    Serial.print(",");

    Serial.print(sa.az, 4);
    Serial.print("\r\n");

    /* ( Magnetometer ) */

    Serial.print("M,");

    Serial.print(sa.mTimeStamp);
    Serial.print(",");

    Serial.print(sa.mx, 4);
    Serial.print(",");

    Serial.print(sa.my, 4);
    Serial.print(",");

    Serial.print(sa.mz, 4);
    Serial.print("\r\n");

    /* ( Euler Angles ) */
    Serial.print("A,");

    Serial.print(ea.eaTimestamp, 2);
    Serial.print(",");

    Serial.print(ea.roll, 2);
    Serial.print(",");

    Serial.print(ea.pitch, 2);
    Serial.print(",");

    Serial.print(ea.yaw, 2);
    Serial.print("\r\n");

    /* ( Quaternions ) */

    Serial.print("Q,");

    Serial.print(q.qTimeStamp);
    Serial.print(",");

    Serial.print(q.q0, 4);
    Serial.print(",");

    Serial.print(q.q1, 4);
    Serial.print(",");

    Serial.print(q.q2, 4);
    Serial.print(",");

    Serial.print(q.q3, 4);
    Serial.print("\r\n");

    /* ( Linear Acceleration ) */
    Serial.print("L,");

    Serial.print(q.q0, 4);
    Serial.print(",");

    Serial.print(q.q1, 4);
    Serial.print(",");

    Serial.print(q.q2, 4);
    Serial.print(",");

    Serial.print(q.q3, 4);
    Serial.print(",");    

    Serial.print(la.linear_ax, 4);
    Serial.print(",");

    Serial.print(la.linear_ay, 4);
    Serial.print(",");

    Serial.print(la.linear_az, 4);
    Serial.print("\r\n");

    previousMillis = millis();
    delay(100);
  }
}

void fullCalibration() {
  
  /* Accelerometer & Gyroscope Calibration */
  Serial.println(" ====== MPU9250 Accelerometer & Gyroscope Calibration (in 5 sec) ======");
  Serial.println("Please keep the device still on the flat plane.");
  delay(5000);

  imu.calibrateAccelGyro();

  /* Magnetometer Calibration */
  Serial.println(" ====== Magnetometer Calibration (in 5 sec) ======");
  delay(5000);

  imu.calibrateMag();

  /* Output the Resulting Normalized Biases of the IMU & Magnetometer */
  normalizeBiases();
  imu.verbose(false);

}

void normalizeBiases() {
  
  Serial.println(" === Inertial Measurement Unit and Magnetometer Biases ===");
  Serial.println("Accelerometer Bias [g]: ");
  
  Serial.print(imu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  
  Serial.print(imu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  
  Serial.print(imu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();

  Serial.println("Gyroscope Bias [deg/s]: ");
  
  Serial.print(imu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  
  Serial.print(imu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  
  Serial.print(imu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();

  Serial.println("Magnetometer Bias [mG]: ");
  Serial.print(imu.getMagBiasX());
  Serial.print(", ");
  
  Serial.print(imu.getMagBiasY());
  Serial.print(", ");
  
  Serial.print(imu.getMagBiasZ());
  Serial.println();

  Serial.println("Magnetometer Scale [uT]: ");
  Serial.print(imu.getMagScaleX());

  Serial.print(", ");
  Serial.print(imu.getMagScaleY());

  Serial.print(", ");
  Serial.print(imu.getMagScaleZ());

  Serial.println();

}