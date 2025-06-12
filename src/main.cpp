/*  @file  main.cpp
    @brief Quaternions and Euler Angles Capture using the BMI270 IMU & BMM150
*/

#include "main.h"
#include "Arduino_BMI270_BMM150.h"

#include <BasicLinearAlgebra.h>
#include <ReefwingAHRS.h>
#include <Reefwing_xIMU3.h>

ReefwingAHRS ahrs;
Reefwing_xIMU3 rx;
NetworkAnnouncement networkAnnouncement;

SensorData sa;

const long displayPeriod = 100;
unsigned long previousMillis = 0;

float deltaT = 0;
long now = 0, lastUpdate = 0;

BLA::Matrix<3,3> A = {1.024214, 0.001741, -0.005501, 0.001741, 1.016293, 0.002423, -0.005501, 0.002423, 1.056541};
BLA::Matrix<3>   b = {-44.098118, 12.418187, -22.342269};

void setup() {
  
  /* AHRS Initialisation and Parameters */
  ahrs.begin();

  ahrs.setBoardType(BoardType::NANO33BLE_SENSE_R2);
  ahrs.setImuType(ImuType::BMI270_BMM150);

  ahrs.setDOF(DOF::DOF_9);
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);

  ahrs.setDeclination(4.89);

  Serial.begin(115200);

  while (!Serial);

  if (IMU.begin() && ahrs.getBoardType() == BoardType::NANO33BLE_SENSE_R2) {
    rx.sendNotification("IMU has been initialized successfully!");

    rx.sendNotification("Connection with xIMU3 GUI has been established!");

  } else {

    rx.sendError("Failed to initialize IMU!");

    rx.sendError("Failed to establish connection with xIMU3 GUI.");

    while (1);

  }

  void loop() {

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
      IMU.readAcceleration(sa.ax, sa.ay, sa.az);
      IMU.readGyroscope(sa.gx, sa.gy, sa.gz);
      IMU.readMagneticField(sa.mx, sa.my, sa.mz);
    }

    BLA::Matrix<3> accelerometer_raw = {sa.mx, sa.my, sa.mz}; 
    BLA::Matrix<3> magnetometer_raw  = {sa.mx, sa.my, sa.mz};

    BLA::Matrix<3> accelerometer_cal = A * (accelerometer_raw - b);
    BLA::Matrix<3> magnetometer_cal  = A * (magnetometer_raw - b);

    sa.ax = accelerometer_cal(0);
    sa.ay = accelerometer_cal(1);
    sa.az = accelerometer_cal(2);

    sa.mx = magnetometer_cal(0);
    sa.my = magnetometer_cal(1);
    sa.mz = magnetometer_cal(2);

    ahrs.setData(sa, true);
    ahrs.update();

    ahrs.mahonyUpdate(sa, getDeltaUpdate());

    Quaternion q = ahrs.getQuaternion();

    if (millis() - previousMillis >= displayPeriod) {
      
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

      /* ( Euler Angles ) */

      Serial.print("A,");
      
      Serial.print(ahrs.angles.timeStamp(), 2); 
      Serial.print(",");

      Serial.print(ahrs.angles.roll, 2);
      Serial.print(",");

      Serial.print(ahrs.angles.pitch, 2);
      Serial.print(",");

      Serial.print(ahrs.angles.yaw, 2);
      Serial.print("\r\n");

      /* ( Quaternions ) */

      Serial.print("Q,");

      Serial.print(q.timeStamp);
      Serial.print(",");

      Serial.print(q.q0);
      Serial.print(",");

      Serial.print(q.q1);
      Serial.print(",");

      Serial.print(q.q2);
      Serial.print(",");

      Serial.print(q.q3);
      Serial.print("\r\n");

      previousMillis = millis();
      delay(1000);

    }
  }

  float getDeltaUpdate() {
    now = micros();
    deltaT = ((now - lastUpdate) / 1000000.0f);
    lastUpdate = now;
    return deltaT;
  }