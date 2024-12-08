#ifndef ROBSTRIDE_CONTROL_H
#define ROBSTRIDE_CONTROL_H

#include "CAN.h"
#include "mbed.h"

// Define CAN pins for Arduino GIGA R1
mbed::CAN can1(PB_5, PB_13); // TX: PB_5, RX: PB_13

void initializeCAN(uint32_t baudRate = 1000000) {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial Monitor to open

    if (can1.frequency(baudRate)) {
        Serial.println("CAN bus initialized at 1 Mbps");
    } else {
        Serial.println("Failed to initialize CAN bus");
        while (true);
    }
}

// Constants for default settings
#define DEFAULT_MAX_CURRENT 23.0
#define DEFAULT_SPEED 1.0
#define DEFAULT_MAX_ACC 1.0

// Parameter Data Types
enum class DataType {
    FLOAT,
    INT16,
    INT8
};

// Parameter Metadata Structure
struct Parameter {
    uint16_t index;   // Parameter index (2 bytes)
    DataType type;    // Data type (FLOAT, INT16, INT8)
    uint8_t size;     // Size in bytes
};

// Parameter Definitions
const Parameter RUN_MODE = {0x0570, DataType::INT8, 1};
const Parameter SPEED_MAX_CURRENT = {0x1870, DataType::FLOAT, 4};
const Parameter SPEED_TARGET = {0x0A70, DataType::FLOAT, 4};
const Parameter POSITION_SPEED_LIMIT = {0x1770, DataType::FLOAT, 4};
const Parameter POSITION_TARGET = {0x1670, DataType::FLOAT, 4};
const Parameter MECH_POS = {0x1970, DataType::FLOAT, 4};
const Parameter SPEED_ACCELERATION = {0x2270, DataType::FLOAT, 4};
const Parameter POSITION_03_SPEED = {0x2470, DataType::FLOAT, 4};
const Parameter POSITION_ACCELERATION = {0x2570, DataType::FLOAT, 4};

// Communication Types
enum CommunicationType {
    COMM_TYPE_ENABLE = 3,
    COMM_TYPE_DISABLE = 4,
    COMM_TYPE_WRITE = 18,
    COMM_TYPE_READ = 17,
    COMM_TYPE_RESET = 6
};

// Motor Class
class Motor {
public:
    uint8_t motorID;
    uint8_t hostID = 253; // Default host ID

    Motor(uint8_t id) : motorID(id) {}

    // Build a CAN message ID
    uint32_t buildMessageID(uint8_t commType) {
        return (commType << 24) | (0x00 << 16) | (hostID << 8) | motorID;
    }

    // Build a CAN data array
    void buildData(const Parameter* param, float value, uint8_t* data) {
        memset(data, 0, 8); // Clear the data array

        if (param) {
            data[0] = (param->index >> 8) & 0xFF; // Parameter Index High Byte
            data[1] = param->index & 0xFF;        // Parameter Index Low Byte
            data[2] = 0x00;                       // Reserved
            data[3] = 0x00;                       // Reserved

            if (param->type == DataType::FLOAT) {
                uint32_t valueHex;
                memcpy(&valueHex, &value, sizeof(float));
                data[4] = (valueHex >> 0) & 0xFF;  // Byte 4 (Little Endian)
                data[5] = (valueHex >> 8) & 0xFF;  // Byte 5
                data[6] = (valueHex >> 16) & 0xFF; // Byte 6
                data[7] = (valueHex >> 24) & 0xFF; // Byte 7
            } else if (param->type == DataType::INT8) {
                data[4] = static_cast<uint8_t>(value); // Byte 4
            } else if (param->type == DataType::INT16) {
                uint16_t valueHex = static_cast<uint16_t>(value);
                data[4] = (valueHex >> 0) & 0xFF; // Byte 4 (Little Endian)
                data[5] = (valueHex >> 8) & 0xFF; // Byte 5
            }
        }
    }

    // Send a CAN message
    void sendCommand(uint8_t commType, const Parameter* param = nullptr, float value = 0.0, bool interpretResponse = false) {
        uint32_t msgID = buildMessageID(commType);
        uint8_t data[8];
        buildData(param, value, data);

        mbed::CANMessage msg(msgID, data, sizeof(data), CANData, CANAny);

        // Send the message
        if (can1.write(msg)) {
            Serial.print("Sent: ID: ");
            Serial.print(msgID, HEX);
            Serial.print(" | Data: ");
            for (int i = 0; i < 8; i++) {
                Serial.print("0x");
                if (data[i] < 0x10) Serial.print("0"); // Add leading zero for single-digit hex
                Serial.print(data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.println("Failed to send CAN message.");
        }

        // Receive and process response
        if (interpretResponse) {
            readAndInterpretResponse();
        } else {
            readResponse();
        }
    }

    // Reset Position to 0
    void resetPosition() {
        uint32_t msgID = buildMessageID(COMM_TYPE_RESET);
        uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        mbed::CANMessage msg(msgID, data, sizeof(data), CANData, CANAny);

        if (can1.write(msg)) {
            Serial.print("Sent: ID: ");
            Serial.print(msgID, HEX);
            Serial.print(" | Data: ");
            for (int i = 0; i < 8; i++) {
                Serial.print("0x");
                if (data[i] < 0x10) Serial.print("0");
                Serial.print(data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.println("Failed to send Reset Position command.");
        }
        readResponse();
    }

    // Enable Motor
    void enable() {
        sendCommand(COMM_TYPE_ENABLE);
    }

    void disable() {
        sendCommand(COMM_TYPE_DISABLE);
    }

    // Write Parameter Command
    void writeParameter(const Parameter& param, float value) {
        sendCommand(COMM_TYPE_WRITE, &param, value);
    }

    // Read Parameter Command with Interpretation
    void readParameter(const Parameter& param) {
      // Clear CAN buffer by reading any old messages
      mbed::CANMessage tempMsg;
      while (can1.read(tempMsg)) {} // Read and discard old messages

      // Send the read parameter command
      sendCommand(COMM_TYPE_READ, &param, 0.0, true);
    }


    // Read and Interpret Response
    void readAndInterpretResponse() {
        mbed::CANMessage receivedMsg;

        if (can1.read(receivedMsg)) {
            Serial.print("Received: ID: ");
            Serial.print(receivedMsg.id, HEX);
            Serial.print(" | Data: ");
            for (int i = 0; i < receivedMsg.len; i++) {
                Serial.print("0x");
                if (receivedMsg.data[i] < 0x10) Serial.print("0");
                Serial.print(receivedMsg.data[i], HEX);
                Serial.print(" ");
            }

            // Extract parameter index
            uint16_t paramIndex = (receivedMsg.data[0] << 8) | receivedMsg.data[1];

            // Extract value based on parameter type
            float floatValue = 0.0;
            uint16_t int16Value = 0;
            uint8_t int8Value = 0;

            if (paramIndex == MECH_POS.index || paramIndex == SPEED_MAX_CURRENT.index ||
                paramIndex == SPEED_TARGET.index || paramIndex == POSITION_TARGET.index) {
                memcpy(&floatValue, &receivedMsg.data[4], sizeof(float));
                Serial.print(" | Parameter: ");
                Serial.print("0x");
                Serial.print(paramIndex, HEX);
                Serial.print("(MECH_POS)");
                Serial.print(" | Value: ");
                Serial.println(floatValue, 4);
            } else if (paramIndex == RUN_MODE.index) {
                int8Value = receivedMsg.data[4];
                Serial.print(" | Parameter: ");
                Serial.print("0x");
                Serial.print(paramIndex, HEX);
                Serial.print("(RUN_MODE)");
                Serial.print(" | Value: ");
                Serial.println((int)int8Value);
            }
        } else {
            Serial.println("No response received.");
        }
    }

    // Read raw response
    void readResponse() {
        mbed::CANMessage receivedMsg;

        if (can1.read(receivedMsg)) {
            Serial.print("Received: ID: ");
            Serial.print(receivedMsg.id, HEX);
            Serial.print(" | Data: ");
            for (int i = 0; i < receivedMsg.len; i++) {
                Serial.print("0x");
                if (receivedMsg.data[i] < 0x10) Serial.print("0");
                Serial.print(receivedMsg.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    // Set Position
    void setPosition(float position, float speed = DEFAULT_SPEED, float maxAcc = DEFAULT_MAX_ACC) {
        writeParameter(RUN_MODE, 1);
        enable();
        writeParameter(POSITION_SPEED_LIMIT, speed);
        writeParameter(POSITION_03_SPEED, speed);
        writeParameter(POSITION_ACCELERATION, maxAcc);
        writeParameter(POSITION_TARGET, position);
    }

    void setVelocity(float velocity, float maxCurrent = DEFAULT_MAX_CURRENT, float maxAcc = DEFAULT_MAX_ACC) {
        writeParameter(RUN_MODE, 2);
        enable();
        writeParameter(SPEED_MAX_CURRENT, maxCurrent);
        writeParameter(SPEED_ACCELERATION, maxAcc);
        writeParameter(SPEED_TARGET, velocity);
    }
};

#endif