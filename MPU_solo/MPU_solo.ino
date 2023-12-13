/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

unsigned long timer = 0;
int temp_offset = -6.5;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("Iniciando MPU6050: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(2000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

  if(millis() - timer > 1000){ // print data every second
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX()*10);Serial.print("m/s^2  ");
    Serial.print("\tY: ");Serial.print(mpu.getAccY()*10);Serial.print("m/s^2  ");
    Serial.print("\tZ: ");Serial.print(mpu.getAccZ()*10);Serial.println("m/s^2  ");
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());Serial.print(" °/s  ");
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());Serial.print(" °/s  ");
    Serial.print("\tZ: ");Serial.print(mpu.getGyroZ());Serial.println(" °/s  ");
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());Serial.print(" °  ");
    Serial.print("\tY: ");Serial.print(mpu.getAccAngleY());Serial.println(" °  ");
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());Serial.print(" °  ");
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());Serial.print(" °  ");
    Serial.print("\tZ: ");Serial.print(mpu.getAngleZ());Serial.println(" °  ");
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }

}