#include <iostream>
#include <bitset>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>
#include <windows.h> // Windows API for serial communication

using namespace std;

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

// Command Structure
struct CANMessage {
    uint8_t header[2] = {0x41, 0x54}; // Fixed header
    uint8_t extendedHeader[4] = {0};  // Extended header (calculated dynamically)
    uint8_t dataLength = 0x08;        // Fixed data length
    uint8_t data[8] = {0};            // Data area (zero-filled initially)
    uint8_t tail[2] = {0x0D, 0x0A};   // Fixed tail
};

enum CommunicationType {
    COMM_TYPE_ENABLE = 3,    // Enable motor
    COMM_TYPE_DISABLE = 4,   // Disable motor
    COMM_TYPE_WRITE = 18,     // Write parameter
    COMM_TYPE_READ = 17
};

// Motor Class
class Motor {
public:
    uint8_t motorID;
    uint8_t hostID = 253; // Default host ID
    HANDLE hSerial;       // Serial handle

    Motor(uint8_t id, HANDLE serialHandle) : motorID(id), hSerial(serialHandle) {}

    // Calculate the extended header
    vector<uint8_t> calculateExtendedHeader(uint8_t commType) {
        string binary = bitset<8>(commType).to_string() +
                        bitset<8>(0).to_string() +
                        bitset<8>(hostID).to_string() +
                        bitset<8>(motorID).to_string();

        // Add '100' to the right
        binary += "100";

        // Drop the first 3 bits
        binary = binary.substr(3);

        // Convert binary to hexadecimal
        uint32_t hexValue = bitset<32>(binary).to_ulong();

        // Convert to 4 bytes in big-endian format
        vector<uint8_t> extendedHeader = {
            static_cast<uint8_t>((hexValue >> 24) & 0xFF),
            static_cast<uint8_t>((hexValue >> 16) & 0xFF),
            static_cast<uint8_t>((hexValue >> 8) & 0xFF),
            static_cast<uint8_t>(hexValue & 0xFF)
        };

        return extendedHeader;
    }

    // Build a CAN message
    CANMessage buildCommand(uint8_t commType, const Parameter* param = nullptr, float value = 0.0) {
        CANMessage msg;

        // Calculate the extended header
        vector<uint8_t> extendedHeader = calculateExtendedHeader(commType);
        memcpy(msg.extendedHeader, extendedHeader.data(), 4);

        // Zero out the data area if no parameter is provided
        memset(msg.data, 0, 8);

        // If a parameter is provided, fill in the data area
        if (param) {
            msg.data[0] = (param->index >> 8) & 0xFF; // Parameter Index High Byte
            msg.data[1] = param->index & 0xFF;        // Parameter Index Low Byte
            msg.data[2] = 0x00;                      // Reserved
            msg.data[3] = 0x00;                      // Reserved

            // Encode value based on data type
            if (param->type == DataType::FLOAT) {
                uint32_t valueHex;
                memcpy(&valueHex, &value, sizeof(float));
                msg.data[4] = (valueHex >> 0) & 0xFF;    // Byte 4 (Little Endian)
                msg.data[5] = (valueHex >> 8) & 0xFF;    // Byte 5
                msg.data[6] = (valueHex >> 16) & 0xFF;   // Byte 6
                msg.data[7] = (valueHex >> 24) & 0xFF;   // Byte 7
            } else if (param->type == DataType::INT8) {
                msg.data[4] = static_cast<uint8_t>(value); // Byte 4
            } else if (param->type == DataType::INT16) {
                uint16_t valueHex = static_cast<uint16_t>(value);
                msg.data[4] = (valueHex >> 0) & 0xFF; // Byte 4 (Little Endian)
                msg.data[5] = (valueHex >> 8) & 0xFF; // Byte 5
            }
        }

        return msg;
    }

    // Send a CAN message
    void sendCommand(CANMessage msg, bool interpretResponse = false) {
        vector<uint8_t> message;
        message.insert(message.end(), msg.header, msg.header + 2);
        message.insert(message.end(), msg.extendedHeader, msg.extendedHeader + 4);
        message.push_back(msg.dataLength);
        message.insert(message.end(), msg.data, msg.data + 8);
        message.insert(message.end(), msg.tail, msg.tail + 2);

        DWORD bytesWritten;
        if (!WriteFile(hSerial, message.data(), message.size(), &bytesWritten, NULL)) {
            cerr << "Error writing to serial port" << endl;
            return;
        }

        // Display the sent message
        cout << "Sent: ";
        for (const auto& byte : message) {
            cout << hex << setw(2) << setfill('0') << (int)byte << " ";
        }
        cout << endl;

        // Read the response
        if (interpretResponse) {
            interpretResponseMessage();
        } else {
            readResponse();
        }
    }

    // Read and interpret response from the motor
    void interpretResponseMessage() {
        uint8_t buffer[1024] = {0};
        DWORD bytesRead;

        if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL)) {
            // Ensure the response has enough bytes
            if (bytesRead < 14) {
                cerr << "Error: Incomplete response received." << endl;
                return;
            }

            // Extract the parameter index
            uint16_t paramIndex = (buffer[8] << 8) | buffer[9];

            // Extract the value based on the parameter type
            float value = 0.0;
            if (paramIndex == MECH_POS.index) {
                memcpy(&value, &buffer[12], sizeof(float));
            }

            // Print the interpreted response
            cout << "Parameter: MECH_POS, Value: " << value << endl;
        } else {
            cerr << "Error reading from serial port" << endl;
        }
    }

    // Read response from the motor (non-interpreted)
    void readResponse() {
        uint8_t buffer[1024] = {0};
        DWORD bytesRead;

        if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL)) {
            cout << "Received: ";
            for (DWORD i = 0; i < bytesRead; ++i) {
                cout << hex << setw(2) << setfill('0') << (int)buffer[i] << " ";
            }
            cout << endl;
        } else {
            cerr << "Error reading from serial port" << endl;
        }
    }

    // Enable Motor
    void enable() {
        CANMessage enableMsg = buildCommand(COMM_TYPE_ENABLE);
        sendCommand(enableMsg);
    }

    // Disable Motor
    void disable() {
        CANMessage disableMsg = buildCommand(COMM_TYPE_DISABLE);
        sendCommand(disableMsg);
    }

    // Write Parameter Command
    void writeParameter(const Parameter& param, float value) {
        CANMessage writeMsg = buildCommand(COMM_TYPE_WRITE, &param, value);
        sendCommand(writeMsg);
    }

    // Read Parameter Command
    void readParameter(const Parameter& param) {
        CANMessage readMsg = buildCommand(COMM_TYPE_READ, &param);
        sendCommand(readMsg, true);
    }
};

// Initialize the serial port
HANDLE initializeSerial(const string& portName, int baudRate) {
    HANDLE hSerial = CreateFile(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Error: Could not open serial port " << portName << endl;
        exit(1);
    }

    // Configure serial port settings
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        cerr << "Error: Could not get serial port state." << endl;
        CloseHandle(hSerial);
        exit(1);
    }
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        cerr << "Error: Could not set serial port state." << endl;
        CloseHandle(hSerial);
        exit(1);
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        cerr << "Error: Could not set timeouts." << endl;
        CloseHandle(hSerial);
        exit(1);
    }

    return hSerial;
}

// Main function
int main() {
    HANDLE hSerial = initializeSerial("\\\\.\\COM7", 921600);

    Motor j1(127, hSerial);
    Motor j2(1, hSerial);

    j1.writeParameter(RUN_MODE, 2);
    j1.enable();
    j1.writeParameter(SPEED_MAX_CURRENT, 23.0);
    j1.writeParameter(SPEED_TARGET, 1.0);
    while (1) {
        j1.readParameter(MECH_POS);
    }

    CloseHandle(hSerial);
    return 0;
}
