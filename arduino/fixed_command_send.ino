#include "CAN.h"

// Define CAN pins for Arduino GIGA R1
mbed::CAN can1(PB_5, PB_13);  // TX: PB_5, RX: PB_13

void setup() {
  // Initialize Serial Communication for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for the Serial Monitor to open

  // Initialize CAN interface
  if (can1.frequency(1000000)) { // Set CAN bus frequency to 1 Mbps
    Serial.println("CAN bus initialized at 1 Mbps");
  } else {
    Serial.println("Failed to initialize CAN bus");
    while (true);
  }
}

void loop() {
  // Define the message ID and data
  uint32_t msgID = 0x1200FD7F;
  uint8_t data[8] = {0x05, 0x70, 0x00, 0x00, 0x07, 0x01, 0x82, 0xF9};
  uint8_t dataLen = 8; // Data length is 8 bytes

  // Create a CAN message
  mbed::CANMessage msg(msgID, data, dataLen, CANData, CANAny);

  // Send the CAN message
  if (can1.write(msg)) {
    Serial.print("Sent: ID: ");
    Serial.print(msgID, HEX);
    Serial.print(" | Data: ");
    for (int i = 0; i < dataLen; i++) {
      Serial.print("0x");
      if (data[i] < 0x10) Serial.print("0"); // Add leading zero for single-digit hex
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to send CAN message.");
  }

  // Wait for 500ms before sending the next message
  delay(500);
}
