#include "RobstrideControl.h"

Motor motor(2);

void setup() {
    initializeCAN();
    motor.resetPosition();
    //motor.setVelocity(-1.0);
    //motor.setPosition(0.0);
}

void loop() {
    if (Serial.available()) {
        // Read the incoming angle as a string
        String received_angle = Serial.readStringUntil('\n');

        // Echo the angle back to ROS
        // Serial.print("ACK: ");
        // Serial.println(received_angle);

        // Optional: Parse and log the received angle
        float angle = received_angle.toFloat();
        //Serial.print("Received angle: ");
        //Serial.println(angle, 4);  // Print with 4 decimal precision
        motor.setPosition(angle);

    }
}
