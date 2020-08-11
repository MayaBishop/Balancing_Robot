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

int16_t accY, accZ, gyroX, gryoRate;
float accAngle, gyroAngle=0;
unsigned long curTime, prevTime=0, loopTime;

void setup() {
  // put your setup code here, to run once:
  mpu.initialize();
  Serial.begin(115200);
}

void loop() {
  curTime = millis();
  loopTime = curTime - prevTime;
  prevTime = curTime;

  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250));
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  
  yAccel = mpu.getAccelerationY();
  zAccel = mpu.getAccelerationZ();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;

}
