#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050_light.h>
#include <TinyGPS.h>

Adafruit_BMP280 bmp; // Create an instance of the BMP280 object
QMC5883LCompass compass;
MPU6050 mpu(Wire);
TinyGPS gps;

// I2C address of BMP280 (you can try both addresses)
#define BMP280_I2C_ADDRESS_1 0x76
#define BMP280_I2C_ADDRESS_2 0x77

// Sea level pressure at your location (in hectopascals)
#define SEALEVELPRESSURE_HPA (1013.25)

#define GPS_BAUDRATE 9600
#define RXD2 16
#define TXD2 17

HardwareSerial NEO6M(1);

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  NEO6M.begin(GPS_BAUDRATE, SERIAL_8N1, RXD2, TXD2);

  Serial.println(F("Initializing..."));

  if (!bmp.begin(BMP280_I2C_ADDRESS_1) && !bmp.begin(BMP280_I2C_ADDRESS_2)) {
    Serial.println(F("Error initializing BMP280. Check connections and I2C address."));
    while (1);
  }

  compass.init();
  compass.setCalibrationOffsets(-945.00, 422.00, 342.00);
  compass.setCalibrationScales(0.97, 1.00, 1.03);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("Initializing MPU6050: "));
  Serial.println(status);
  while (status != 0); // stop everything if could not connect to MPU6050

  delay(2000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("MPU6050 calibration done!");
}

void loop() {
  // Check BMP280 connection during loop
  if (!bmp.begin()) {
    Serial.println(F("Lost connection with BMP280. Check connections and I2C address."));
    while (1);
  }

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature() - 8);
  Serial.println(" °C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure() / 100.0F); // Pressure is read in Pa, convert to hPa
  Serial.println(" hPa");

  Serial.print(F("Approximate Altitude = "));
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) + 500);
  Serial.println(" meters");

  compass.read();
  byte a = compass.getAzimuth();

  char myArray[3];
  compass.getDirection(myArray, a);

  Serial.print(F("Compass Direction = "));
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
  Serial.println();

  mpu.update();

  if (millis() - timer > 1000) { // print data every second
    Serial.print(F("ACCELEROMETER X: ")); Serial.print(mpu.getAccX() * 10); Serial.print("m/s^2  ");
    Serial.print("\tY: "); Serial.print(mpu.getAccY() * 10); Serial.print("m/s^2  ");
    Serial.print("\tZ: "); Serial.print(mpu.getAccZ() * 10); Serial.println("m/s^2  ");

    Serial.print(F("GYRO X: ")); Serial.print(mpu.getGyroX()); Serial.print(" °/s  ");
    Serial.print("\tY: "); Serial.print(mpu.getGyroY()); Serial.print(" °/s  ");
    Serial.print("\tZ: "); Serial.print(mpu.getGyroZ()); Serial.println(" °/s  ");

    Serial.print(F("ACC ANGLE X: ")); Serial.print(mpu.getAccAngleX()); Serial.print(" °  ");
    Serial.print("\tY: "); Serial.print(mpu.getAccAngleY()); Serial.println(" °  ");

    Serial.print(F("ANGLE X: ")); Serial.print(mpu.getAngleX()); Serial.print(" °  ");
    Serial.print("\tY: "); Serial.print(mpu.getAngleY()); Serial.print(" °  ");
    Serial.print("\tZ: "); Serial.print(mpu.getAngleZ()); Serial.println(" °  ");


    timer = millis();
  }

  // Read and print GPS data
  while (NEO6M.available() > 0) {
    Serial.write(NEO6M.read());
  }
  delay(2000); // Wait 2 seconds before reading again
  Serial.println(F("=====================================================\n"));
  
}
