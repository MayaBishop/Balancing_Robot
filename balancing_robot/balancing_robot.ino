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

// motor one
int spd1 = 10;
int fwd1 = 9;
int rev1 = 8;
// motor two
int spd2 = 5;
int fwd2 = 7;
int rev2 = 6;

MPU6050 mpu;

int16_t accY, accZ, gyroX, gyroRate;
float accAngle, gyroAngle=0;
unsigned long curTime, prevTime=0, loopTime;

void setup() {
  mpu.initialize();
  Serial.begin(115200);
  pinMode(spd1, OUTPUT);
  pinMode(spd2, OUTPUT);
  pinMode(fwd1, OUTPUT);
  pinMode(rev1, OUTPUT);
  pinMode(fwd2, OUTPUT);
  pinMode(rev2, OUTPUT);
}

void loop() {

}

void moveWheels(){
  
}

float getAngle(){
  curTime = millis();
  loopTime = curTime - prevTime;
  prevTime = curTime;

  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  Serial.println(accAngle);
  Serial.println(gyroAngle);
  return accAngle;
}
