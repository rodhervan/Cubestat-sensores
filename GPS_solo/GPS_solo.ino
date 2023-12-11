#include <SoftwareSerial.h>

SoftwareSerial NEO6M(9,8);

void setup() {
  Serial.begin(115200);

  NEO6M.begin(9600);

}

void loop() {
  while (NEO6M.available()> 0){
    Serial.write(NEO6M.read());
  }

}
