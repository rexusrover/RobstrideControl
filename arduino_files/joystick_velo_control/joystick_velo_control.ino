#include "RobstrideControl.h"

// Define motor speed as a macro
#define MOTOR_SPEED 0.1

// Initialize motors
Motor joint1(21);
Motor joint2(22);
Motor joint3(23);
Motor joint4(24);
Motor joint5(25);
Motor joint6(26);
Motor joint7(127);

void setup() {
    Serial.begin(115200);
    initializeCAN(); // Initialize CAN bus

    // Reset all joint positions
    // joint1.resetPosition();
    // joint2.resetPosition();
    // joint3.resetPosition();
    // joint4.resetPosition();
    // joint5.resetPosition();
    // joint6.resetPosition();
    // joint7.resetPosition();

    Serial.println("System Initialized");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // Read command
        command.trim(); // Remove whitespace

        // Process commands for Joint 3
        if (command == "J3_FWD") {
            joint3.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J3_FWD");
        } else if (command == "J3_REV") {
            joint3.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J3_REV");
        } else if (command == "J3_STOP") {
            joint3.setVelocity(0);
            Serial.println("ACK_J3_STOP");
        }

        // Process commands for Joint 1
        else if (command == "J1_FWD") {
            joint1.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J1_FWD");
        } else if (command == "J1_REV") {
            joint1.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J1_REV");
        } else if (command == "J1_STOP") {
            joint1.setVelocity(0);
            Serial.println("ACK_J1_STOP");
        }

        // Process commands for Joint 2
        else if (command == "J2_FWD") {
            joint2.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J2_FWD");
        } else if (command == "J2_REV") {
            joint2.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J2_REV");
        } else if (command == "J2_STOP") {
            joint2.setVelocity(0);
            Serial.println("ACK_J2_STOP");
        }

        // Process commands for Joint 4
        else if (command == "J4_FWD") {
            joint4.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J4_FWD");
        } else if (command == "J4_REV") {
            joint4.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J4_REV");
        } else if (command == "J4_STOP") {
            joint4.setVelocity(0);
            Serial.println("ACK_J4_STOP");
        }

        // Process commands for Joint 5
        else if (command == "J5_FWD") {
            joint5.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J5_FWD");
        } else if (command == "J5_REV") {
            joint5.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J5_REV");
        } else if (command == "J5_STOP") {
            joint5.setVelocity(0);
            Serial.println("ACK_J5_STOP");
        }

        // Process commands for Joint 6
        else if (command == "J6_FWD") {
            joint6.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J6_FWD");
        } else if (command == "J6_REV") {
            joint6.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J6_REV");
        } else if (command == "J6_STOP") {
            joint6.setVelocity(0);
            Serial.println("ACK_J6_STOP");
        }

        // Process commands for Joint 7
        else if (command == "J7_FWD") {
            joint7.setVelocity(MOTOR_SPEED);
            Serial.println("ACK_J7_FWD");
        } else if (command == "J7_REV") {
            joint7.setVelocity(-MOTOR_SPEED);
            Serial.println("ACK_J7_REV");
        } else if (command == "J7_STOP") {
            joint7.setVelocity(0);
            Serial.println("ACK_J7_STOP");
        }

        // Handle unknown command
        else {
            Serial.println("ERR_UNKNOWN_CMD");
        }
    }
}
