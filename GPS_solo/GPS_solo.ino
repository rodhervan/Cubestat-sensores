#include <Wire.h>
#include <TinyGPS.h>

#define GPS_BAUDRATE 9600
#define RXD2 16
#define TXD2 17

HardwareSerial NEO6M(1);

void setup() {
  Serial.begin(115200);
  NEO6M.begin(GPS_BAUDRATE, SERIAL_8N1, RXD2, TXD2);

}

void loop() {
  while (NEO6M.available()> 0){
    Serial.write(NEO6M.read());
  }
}
