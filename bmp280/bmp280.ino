#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // Crear una instancia del objeto BMP280

// Dirección I2C del BMP280 (puedes probar ambas direcciones)
#define BMP280_I2C_ADDRESS_1 0x76
#define BMP280_I2C_ADDRESS_2 0x77

// Definir la presión al nivel del mar en tu ubicación (en hectopascales)
#define SEALEVELPRESSURE_HPA (1013.25)

void setup() {
  Serial.begin(9600);
  Serial.println(F("Iniciando..."));

  if (!bmp.begin(BMP280_I2C_ADDRESS_1) && !bmp.begin(BMP280_I2C_ADDRESS_2)) {
    Serial.println(F("Error al inicializar el BMP280. Verifica las conexiones y la dirección I2C."));
    while (1);
  }

  Serial.println(F("BMP280 inicializado correctamente."));
}

void loop() {
  // Verificar la conexión con el BMP280 durante el bucle
  if (!bmp.begin()) {
    Serial.println(F("Conexión perdida con el BMP280. Verifica las conexiones y la dirección I2C."));
    while (1);
  }

  Serial.print(F("Temperatura = "));
  Serial.print(bmp.readTemperature()-8);
  Serial.println(" °C");

  Serial.print(F("Presión = "));
  Serial.print(bmp.readPressure() / 100.0F); // La presión se lee en Pa, lo convertimos a hPa
  Serial.println(" hPa");

  Serial.print(F("Altitud aproximada = "));
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) + 500);
  Serial.println(" metros");

  Serial.println();

  delay(2000); // Espera 2 segundos antes de leer nuevamente
}