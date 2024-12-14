#include "mbed.h"
#include "WitEncoderControl.h"

Encoder encoder(0x50);

// Define CAN pins for Arduino GIGA
//mbed::CAN can1(PB_5, PB_13); // TX: PB_5, RX: PB_13
int counter = 0;
void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
    while (!Serial);      // Wait for the Serial Monitor to open

    // Initialize CAN with a baud rate of 250K (default for the encoder)
    initializeCAN();
    encoder.setReturnRate(ReturnRate::SINGLE_RETURN);
}

//
void loop() {
    encoder.readAngle();
    encoder.readAngularSpeed();
    encoder.readRotation();
    Serial.print("Angle : ");
    Serial.print(encoder.getAngle());
    Serial.print(" | Rotation : ");
    Serial.print(encoder.getRotation());
    Serial.print(" | Ang Speed : ");
    Serial.println(encoder.getAngularVelocity());
    delay(100); // Small delay to avoid flooding the Serial Monitor
}