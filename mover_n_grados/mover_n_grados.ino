#include <Servo.h>

#define SERVO_PIN 26 // ESP32 pin GPIO26 connected to servo motor

Servo servoMotor;
int currentPos = 20;

void setup() {
  Serial.begin(115200);  // initialize serial communication
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
}

void loop() {
  if (Serial.available() > 0) {
    // If there is data available on the serial port
    String input = Serial.readStringUntil('\n'); // Read the entire line until newline

    // Convert the string to an integer
    int newPos = input.toInt();

    // Ensure that the angle is within valid servo range (0 to 180)
    newPos = constrain(newPos, 0, 180);

    // Move the servo to the specified angle if it's different from the current position
    if (newPos != currentPos) {
      servoMotor.write(newPos);
      currentPos = newPos; // Update the current position
    }

    // Wait for the servo to reach the position
    delay(15);
  }
}
