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
  curTime = millis();
  loopTime = curTime - prevTime;
  dt = loopTime/1000.0;
  prevTime = curTime;

  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*dt;
  
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  currentAngle = 0.9934 * (previousAngle + (float)gyroRate*dt) + 0.0066 * (accAngle);
  Serial.print("time\t");
  Serial.print(loopTime, 6);
  Serial.print("\tdt\t");
  Serial.print(dt, 6);
  Serial.print("\t\taccelormeter angle\t");
  Serial.print(accAngle, 6);
  Serial.print("\t\tgyroscope rate\t");
  Serial.print(gyroRate, 6);
  Serial.print("\tgyroscope angle\t");
  Serial.print(gyroAngle, 6);
  Serial.print("\t\tcalculated angle\t");
  Serial.println(currentAngle, 6);
  previousAngle = currentAngle;
}
