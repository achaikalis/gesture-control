/* @file  main.cpp */

#include "main.h"

#include <BasicLinearAlgebra.h>
#include <ReefwingAHRS.h>
#include <Reefwing_xIMU3.h>

#include "MPU9250.h"
#include "MPU9250/QuaternionFilter.h"

MPU9250 imu;
ReefwingAHRS ahrs;
Reefwing_xIMU3 rx;
NetworkAnnouncement networkAnnouncement;

SensorData sa;

const long displayPeriod = 100;
unsigned long previousMillis = 0;

float deltaT = 0;
long now = 0, lastUpdate = 0;

/* Accelerometer & Magnetometer Biases used in Calibration */

BLA::Matrix<3, 3> A_accel = {0.000031, 0,         -0.000002, 0,       2.500838,
                             0,        -0.000002, 0,         2.500838};
BLA::Matrix<3> b_accel = {27854.598976, 0.023025, 0.501215};

// BLA::Matrix<3, 3> A_gyro = {};

// BLA::Matrix<3, 3> b_gyro - {};

// imu.setGyroBias(b_gyro(0),  b_gyro(1),  b_gyro(2))

BLA::Matrix<3, 3> A_mag = {0.001119, -0.000005, 0.000005, -0.000005, 0.001062,
                           0.000025, 0.000005,  0.000025, 0.001020};
BLA::Matrix<3> b_mag = {-30.293799, 12.002223, 6.093839};

void setup() {
  Serial.begin(11520);

  Wire.begin();
  delay(2000);

  /* Set Custom Settings Profile for the MPU9250 */

  MPU9250Setting settings;
  settings.accel_fs_sel = ACCEL_FS_SEL::A16G;
  settings.accel_fchoice = 0x01;
  settings.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  settings.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  settings.gyro_fchoice = 0x03;
  settings.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;

  settings.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  settings.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;

  /* MPU2950 Setup */
  imu.setup(0x68, settings);

  /* Set Sensor Fusion Filter and Iterations */
  imu.selectFilter(QuatFilterSel::MAHONY);
  imu.setFilterIterations(20);

  imu.setAccBias(b_accel(0), b_accel(1), b_accel(2));
  imu.setMagBias(b_mag(0), b_mag(1), b_mag(2));

  /* Set Magnetic Declination */
  imu.setMagneticDeclination(4.89);

  /* AHRS Initialisation and Parameters */
  // ahrs.begin();

  // ahrs.setBoardType(BoardType::NANO33BLE_SENSE_R2);
  // ahrs.setImuType(ImuType::MPU6050);

  // ahrs.setDOF(DOF::DOF_9);
  // ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  // ahrs.setDeclination(4.89);

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
  if (imu.update()) {
    imu.getAccX(sa.ax);
    imu.getAccY(sa.ay);
    imu.getAccZ(sa.az);
    sa.aTimeStamp = millis();

    imu.getGyroX(sa.gx);
    imu.getGyroY(sa.gy);
    imu.getGyroZ(sa.gz);
    sa.gTimeStamp = millis();

    imu.getMagX(sa.mx);
    imu.getMagY(sa.my);
    imu.getMagZ(sa.mz);
    sa.mTimeStamp = millis();
  }

  // BLA::Matrix<3> accelerometer_raw = {sa.mx, sa.my, sa.mz};
  // BLA::Matrix<3> magnetometer_raw = {sa.mx, sa.my, sa.mz};

  // BLA::Matrix<3> accelerometer_cal = A_accel * (accelerometer_raw - b_accel);
  // BLA::Matrix<3> magnetometer_cal = A_mag * (magnetometer_raw - b_mag);

  // sa.ax = accelerometer_cal(0);
  // sa.ay = accelerometer_cal(1);
  // sa.az = accelerometer_cal(2);

  // sa.mx = magnetometer_cal(0);
  // sa.my = magnetometer_cal(1);
  // sa.mz = magnetometer_cal(2);

  // ahrs.setData(sa, true);
  // ahrs.update();

  // getDeltaUpdate();
  // ahrs.mahonyUpdate(sa, getDeltaUpdate());

  /* Update the Sensor Fusion Filter after Acceleromter & Calibration
   * Calibration */

  // imu.update(sa.ax, sa.ay, sa.az,
  //            sa.gx, sa.gy, sa.gz,
  //            sa.mx, sa.my, sa.mz);

  // Quaternion q = ahrs.getQuaternion();
  // q.timeStamp = millis();

  /* millis() - previousMillis >= displayPeriod */
  if (millis() > previousMillis + 25) {
    /* ( Inertial Data ) */

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

    // /* ( Euler Angles ) */

    // Serial.print("A,");

    // Serial.print(ahrs.angles.timeStamp, 2);
    // Serial.print(",");

    // Serial.print(ahrs.angles.roll, 2);
    // Serial.print(",");

    // Serial.print(ahrs.angles.pitch, 2);
    // Serial.print(",");

    // Serial.print(ahrs.angles.yaw, 2);
    // Serial.print("\r\n");

    /* ( Quaternions ) */

    Serial.print("Q,");

    Serial.print(q.timeStamp);
    Serial.print(",");

    // Serial.print(q.q0);
    Serial.print(imu.getQuaternionW(), 4);
    Serial.print(",");

    // Serial.print(q.q1);
    Serial.print(imu.getQuaternionX(), 4);
    Serial.print(",");

    // Serial.print(q.q2);
    Serial.print(imu.getQuaternionY(), 4);
    Serial.print(",");

    // Serial.print(q.q3);
    Serial.print(imu.getQuaternionZ(), 4);
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