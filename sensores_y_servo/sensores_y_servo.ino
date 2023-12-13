#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050_light.h>
#include <TinyGPS.h>
#include <Servo.h>



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

#define SERVO_PIN 26 // ESP32 pin GPIO26 connected to servo motor

Servo servoMotor;
int currentPos = 120;

HardwareSerial NEO6M(1);

double altura_de_corte = 3025.5;

unsigned long timer = 0;

String measurement = ""; // Declare a String variable to store measurements


void setup() {
  Serial.begin(115200);
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
  servoMotor.write(121);
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
  // ... (your existing code)
    // Check BMP280 connection during loop
  if (!bmp.begin()) {
    Serial.println(F("Lost connection with BMP280. Check connections and I2C address."));
    while (1);
  }

  // Concatenate each line to the measurement variable
  measurement += "Temperature = " + String(bmp.readTemperature() - 8) + " °C\n";
  measurement += "Pressure = " + String(bmp.readPressure() / 100.0F) + " hPa\n";
  double altitud = bmp.readAltitude(SEALEVELPRESSURE_HPA) + 500;
  measurement += "Approximate Altitude = " + String(altitud) + " meters\n";

  compass.read();
  byte a = compass.getAzimuth();
  char myArray[3];
  compass.getDirection(myArray, a);
  measurement += "Compass Direction = " + String(myArray[0]) + String(myArray[1]) + String(myArray[2]) + "\n";

  mpu.update();

  if (millis() - timer > 1000) { // print data every second
    // Concatenate accelerometer, gyro, and angle data to the measurement variable
    measurement += "ACCELEROMETER X: " + String(mpu.getAccX() * 10) + "m/s^2  \tY: " + String(mpu.getAccY() * 10) + "m/s^2  \tZ: " + String(mpu.getAccZ() * 10) + "m/s^2\n";
    measurement += "GYRO X: " + String(mpu.getGyroX()) + " °/s  \tY: " + String(mpu.getGyroY()) + " °/s  \tZ: " + String(mpu.getGyroZ()) + " °/s\n";
    measurement += "ACC ANGLE X: " + String(mpu.getAccAngleX()) + " °  \tY: " + String(mpu.getAccAngleY()) + " °\n";
    measurement += "ANGLE X: " + String(mpu.getAngleX()) + " °  \tY: " + String(mpu.getAngleY()) + " °  \tZ: " + String(mpu.getAngleZ()) + " °\n";
    timer = millis();
  }

  // Read and print GPS data
  while (NEO6M.available() > 0) {
    char c = NEO6M.read();
    // Serial.write(c);
    measurement += c;
  }

  if (altitud >= altura_de_corte) {
    servoMotor.write(20);
    measurement += "seguro abierto";
  }
  if (altitud < altura_de_corte) {
    servoMotor.write(120);
    measurement += "seguro cerrado";
  }

  delay(2000); // Wait 2 seconds before reading again
  measurement += "=====================================================\n";
  Serial.println(measurement);
  // Clear the contents of the measurement variable
  measurement = "";
}

