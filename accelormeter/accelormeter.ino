#include <Wire.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

MPU6050 mpu;

int16_t accY, accZ, gyroX, gyroRate;
float accAngle, gyroAngle=0, currentAngle, previousAngle=0;
float tau=0.075;
float a=0.0;
unsigned long curTime, prevTime=0, loopTime,dt;

void setup() {
  // put your setup code here, to run once:
  mpu.initialize();
  mpu.setYAccelOffset(149);
  mpu.setZAccelOffset(1203);
  mpu.setXGyroOffset(77);
  Serial.begin(115200);
}

void loop() {
  dt = loopTime/1000;
  curTime = millis();
  loopTime = curTime - prevTime;
  prevTime = curTime;

  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*dt;
  
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  currentAngle = 0.9934 * (previousAngle + (float)gyroRate*dt) + 0.0066 * (accAngle);
  Serial.print("accelormeter angle ");
  Serial.print(accAngle);
  Serial.print(" gyroscope angle ");
  Serial.print(gyroAngle);
  Serial.print(" calculated angle ");
  Serial.println(currentAngle);
  previousAngle = currentAngle;
}
