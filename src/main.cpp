/* @file  main.cpp */

#include "main.h"

#include "MPU9250.h"
#include "MPU9250/QuaternionFilter.h"

#include <BasicLinearAlgebra.h>

#include <Reefwing_xIMU3.h>

MPU9250 imu;
Reefwing_xIMU3 rx;
NetworkAnnouncement networkAnnouncement;

struct SensorData {
  /* Accelerometer Raw Data in g */
  float ax, ay, az;  
  /* Gyroscope Raw Data in Degrees per Second */
  float gx, gy, gz;
  /* Magnetometer Raw Data in uT */  
  float mx, my, mz;  
  /* Accelerometer, Gyroscope & Magnetometer Timestamps in us */
  uint32_t aTimeStamp, gTimeStamp, mTimeStamp;
} sa;

struct LinearAcceleration {
  /* Linear Acceleration X Component in m/s^2 */
  float linear_ax;
  /* Linear Acceleration Y Component in m/s^2 */
  float linear_ay;
  /* Linear Acceleration Z Component in m/s^2 */
  float linear_az;
  /* Linear Acceleration Reading Timestamp in us */
  uint32_t laTimestamp;
} la;

struct EulerAngles {
  float roll, pitch, yaw;
  /* Euler Angles Reading Timestamp in us */
  uint32_t eaTimestamp;
} ea;

struct Quaternion {
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
} q;

struct TemperatureData {
  /* Temperature in Degrees Centigrade */
  float temperature;
  /* Temperature Reading Timestamp in us */
  uint32_t timeStamp;
} td;

const long displayPeriod = 100;
unsigned long previousMillis = 0;

float deltaT = 0;
long now = 0, lastUpdate = 0;

/* Accelerometer, Gyroscope & Magnetometer Biases */
BLA::Matrix<3, 3> A_accel = {0.000031, 0,         -0.000002, 0,       2.500838,
                             0,        -0.000002, 0,         2.500838};
BLA::Matrix<3> b_accel = {27854.598976, 0.023025, 0.501215};

BLA::Matrix<3, 3> A_gyro = {};
BLA::Matrix<3, 3> b_gyro = {0, 0, 0};

BLA::Matrix<3, 3> A_mag = {0.001119, -0.000005, 0.000005, -0.000005, 0.001062,
                           0.000025, 0.000005,  0.000025, 0.001020};
BLA::Matrix<3> b_mag = {-30.293799, 12.002223, 6.093839};

void setup() {
  Serial.begin(11520);

  Wire.begin();
  delay(2000);

  /* Set Custom Settings Profile for the MPU9250 */
  MPU9250Setting settings;

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

  /* Set Sensor Fusion Filter and Iterations */
  imu.selectFilter(QuatFilterSel::MAHONY);
  imu.setFilterIterations(20);

  /* Set Biases */
  imu.setAccBias(b_accel(0), b_accel(1), b_accel(2));
  imu.setMagBias(b_mag(0), b_mag(1), b_mag(2));
  imu.setGyroBias(b_gyro(0),  b_gyro(1),  b_gyro(2));

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
}

void loop() {
  if (imu.isConnected && imu.update()) {

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

float getDeltaUpdate() {
  now = micros();
  deltaT = ((now - lastUpdate) / 1000000.0f);
  lastUpdate = now;
  return deltaT;
}