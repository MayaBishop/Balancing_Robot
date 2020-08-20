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

// left motor
#define spdLeft 10
#define fwdLeft 9
#define revLeft 8
// motor two
#define spdRight 5
#define fwdRight 7
#define revRight 6

#define Kp 0
#define Ki 0
#define Kd 0

//accelerometer/gyroscope
MPU6050 mpu;
int16_t accY, accZ, gyroX, gyroRate;
float accAngle, gyroAngle=0, currentAngle, previousAngle=0;
unsigned long curTime, prevTime=0, loopTime,dt;

void setup() {
  mpu.initialize();
  mpu.setXGyroOffset(77);
  mpu.setYAccelOffset(149);
  mpu.setZAccelOffset(1203);
  Serial.begin(115200);
  
  pinMode(spdLeft, OUTPUT);
  pinMode(spdRight, OUTPUT);
  pinMode(fwdLeft, OUTPUT);
  pinMode(revLeft, OUTPUT);
  pinMode(fwdRight, OUTPUT);
  pinMode(revRight, OUTPUT);
}

void loop() {
  gyroX = mpu.getRotationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  motorPower = constrain(motorPower, -255, 255);
  moveWheels(motorPower)
}

void moveWheels(int motorSpeed){
  Serial.print(motorSpeed);
  if(motorSpeed >= 0){
    analogWrite(spdLeft, motorSpeed);
    digitalWrite(fwdLeft, HIGH);
    digitalWrite(revLeft, LOW);
    analogWrite(spdRight, motorSpeed);
    digitalWrite(fwdRight, HIGH);
    digitalWrite(revRight, LOW);
  }else{
    analogWrite(spdLeft, -motorSpeed);
    digitalWrite(fwdLeft, LOW);
    digitalWrite(revLeft, HIGH);
    analogWrite(spdRight, -motorSpeed);
    digitalWrite(fwdRight, LOW);
    digitalWrite(revRight, HIGH);
  }
}

float getAngle(){
  curTime = millis();
  loopTime = curTime - prevTime;
  dt = loopTime/1000.0;
  prevTime = curTime;

  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*dt;

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  currentAngle = 0.9934 * (previousAngle + (float)gyroRate*dt) + 0.0066 * (accAngle);
  previousAngle = currentAngle;
  Serial.println(accAngle);
  Serial.println(gyroAngle);
  Serial.println(currentAngle);
  return accAngle;
}

int getMotorSpeed(float angle){
  error = angle - targetAngle;
  errorSum += error;
  errorSum = constrain(errorSum, -300, 300);
  motorPower = Kp*(error) + Ki*(errorSum)*loopTime - Kd*(currentAngle-prevAngle)/loopTime;
  return motorPower;
}
