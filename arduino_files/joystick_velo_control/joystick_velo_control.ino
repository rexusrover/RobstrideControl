#include "RobstrideControl.h"

Motor motor1(2);

void setup() {
  // put your setup code here, to run once:
  initializeCAN();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        if (command.startsWith("VEL:")) {
            float velocity = command.substring(4).toFloat();
            motor1.setVelocity(velocity);
            Serial.print("ACK: Received Velocity: ");
            Serial.println(velocity);
        }
    }

}
