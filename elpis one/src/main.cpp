#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 accelerometer;
int SCL_pin = D1;
int SDA_pin = D2;

void setup() {
  Serial.begin(9600);

  //Initializing the accelerometer
  Serial.println("Initalizing accelerometer MPU6050");
  while(!accelerometer.beginSoftwareI2C(SCL_pin, SDA_pin, MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid sensor, retrying in 500ms");
    delay(500);
  }

}

void loop() {
  Vector acceleration = accelerometer.readNormalizeAccel();
  Vector rotAcceleration = accelerometer.readRawGyro();
  
  Serial.print("X: ");
  Serial.print(acceleration.XAxis);
  Serial.print("Y: ");
  Serial.print(acceleration.YAxis);
  Serial.print("Z: ");
  Serial.print(acceleration.ZAxis);
  Serial.println();

  delay(100);
}