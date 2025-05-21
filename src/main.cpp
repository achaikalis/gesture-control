/***************************************************************

  @file       gesture-control.ino
  @brief      Gesture Control
  @author     Andreas Ioannis Chaikalis
  @date       12 May 2025
  @copyright  ---

***************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <I2C.h>

#include <bmi270.h>
#include <bmm150.h>

#include "Arduino_BMI270_BMM150.h"

#include <ReefwingAHRS.h>
#include <Reefwing_xIMU3.h>

// #include <BasicLinearAlgebra.h>

/******************************************************************************/
/*!                          Class Definitions                                 */

ReefwingAHRS ahrs;

Reefwing_xIMU3 rx;

NetworkAnnouncement networkAnnouncement;

/******************************************************************************/
/*!                         Struct Definitions                                 */

SensorData  sa;

InertialMessage im;

ScaledData sd;

/******************************************************************************/
/*!                         Global Variables                                  */

int loopFrequency = 0;

const long displayPeriod = 100, notificationPeriod = 1000; 

unsigned long previousMillis = 0;

float deltaT = 0; long now = 0; long lastUpdate = 0;

//  BLA::Matrix<3,3> A = {1.024214, 0.001741, -0.005501, 0.001741, 1.016293, 0.002423, -0.005501, 0.002423, 1.056541};

//  BLA::Matrix<3> b = {-44.098118, 12.418187, -22.342269};

 float getDeltaUpdate ();

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

    /* NOTE: Consult interrupts_subclassing.ino for examples on changing the Accelerometer / Gyroscope Sampling Rate
     
    Maximum Accelerometer Output Data Rate (ODR) : 1.6 kHz

    Maximum Gyroscope Output Data Rate (ODR)     : 6.4 kHz 

    The Arduino_BMI270_BMM150 Library initializes the

    Accelerometer and Gyroscope (IMU) at a fixed Output Data Rate (ODR) of 99.84 Hz

    Magnetometer at a fixed Output Data Rate (ODR) of 10 Hz
  
  */
  
  } else {

    rx.sendError("Failed to initialize IMU!");

    rx.sendError("Failed to establish connection with xIMU3 GUI.");
    
    while (1);

  }
}

void loop() {

  /* Convention for the Arduino Nano 33 Sense BLE (Rev2)

    Since the magnetometer y-axis value is reversed:

      Roll --> x-axis.  |
                        | --> Attitude Information
      Pitch --> y-axis  |

      Yaw   --> z-axis

  */

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
   
    IMU.readAcceleration(sa.ax, sa.ay, sa.az);

    IMU.readGyroscope(sa.gx, sa.gy, sa.gz);

    IMU.readMagneticField(sa.mx, sa.my, sa.mz);
    
  }

  /* Magnetometer Calibration */

  // BLA::Matrix<3> magnetometer_raw = {sa.mx, sa.my, sa.mz};

  // BLA::Matrix<3> magnetometer_cal = A * (magnetometer_raw - b);

  // sa.mx = magnetometer_cal(0);
  // sa.my = magnetometer_cal(1);
  // sa.mz = magnetometer_cal(2);

  ahrs.setData(sa, true);

  ahrs.update();

  ahrs.mahonyUpdate(sa, getDeltaUpdate());
  
  if (millis() - previousMillis >= displayPeriod) {

    // Serial.print("\tRoll: ");
    // Serial.print(ahrs.angles.roll, 2);
    
    // Serial.println();

    // Serial.print("\tPitch: ");
    // Serial.print(ahrs.angles.pitch, 2);
    
    // Serial.println();

    // Serial.print("\tYaw: ");
    // Serial.print(ahrs.angles.yaw, 2);
    
    // Serial.println();

    // Serial.print("\tHeading: ");
    // Serial.print(ahrs.angles.heading, 2);

    // Serial.println();
    
    Serial.print("R,");
    
    // Serial.print(sa.gTimeStamp);
    Serial.print(",");

    Serial.print(ahrs.angles.roll, 2);
    Serial.print(",");

    Serial.print(ahrs.angles.pitch, 2);
    Serial.print(",");

    Serial.print(ahrs.angles.pitch, 2);
    Serial.print(",");

    Serial.print(ahrs.angles.heading, 2);
    Serial.print(",");
    
    Serial.print("\r\n");

    loopFrequency = 0;
    previousMillis = millis();

    delay(1000);

  }

  loopFrequency++;

}

float getDeltaUpdate () {

  now = micros();

  deltaT = ((now - lastUpdate) / 1000000.0f);

  lastUpdate = now;

  return deltaT;

}