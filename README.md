# Motor Control Arduino Library - ReadMe

---

## **Table of Functions**

Below is a list of functions available in the `Motor` class to control motors. Each function includes its purpose, example usage, parameters, and parameter details.

| **Function**                | **Purpose**                                                                                  | **Example Usage**                              | **Parameters**                                                                                                         |
|-----------------------------|----------------------------------------------------------------------------------------------|------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| `Motor(uint8_t id)`         | Initializes the motor instance with a specific CAN ID.                                       | `Motor motor(127);`                            | **id (uint8_t)**: Motor CAN ID (compulsory).                                                                          |
| `void enable()`             | Enables the motor.                                                                           | `motor.enable();`                              | None                                                                                                                  |
| `void disable()`            | Disables the motor.                                                                          | `motor.disable();`                             | None                                                                                                                  |
| `void resetPosition()`      | Resets the mechanical position of the motor to zero.                                         | `motor.resetPosition();`                       | None                                                                                                                  |
| `void setVelocity(float velocity, float maxCurrent, float maxAcc)` | Sets the motor to velocity mode and controls its speed.                            | `motor.setVelocity(2.0, 23.0, 1.0);`           | **velocity (float)**: Target velocity (rad/s, compulsory).<br>**maxCurrent (float)**: Max current (A, optional).<br>**maxAcc (float)**: Max acceleration (rad/s², optional). |
| `void setPosition(float position, float speed, float maxAcc)`    | Sets the motor to position mode and moves it to the desired angle.                  | `motor.setPosition(90.0, 1.0, 0.5);`           | **position (float)**: Target position (rad, compulsory).<br>**speed (float)**: Max speed (rad/s, optional).<br>**maxAcc (float)**: Max acceleration (rad/s², optional). |
| `void writeParameter(const Parameter& param, float value)`      | Writes a specific parameter value to the motor.                                      | `motor.writeParameter(RUN_MODE, 1);`           | **param (Parameter)**: Target parameter (compulsory).<br>**value (float)**: Value to write (compulsory).              |
| `void readParameter(const Parameter& param)`                    | Reads the value of a specific parameter from the motor.                              | `motor.readParameter(MECH_POS);`               | **param (Parameter)**: Target parameter (compulsory).                                                                 |
| `void sendCommand(uint8_t commType, const Parameter* param, float value, bool interpretResponse)` | Sends a custom CAN command to the motor.                  | `motor.sendCommand(18, &RUN_MODE, 1.0, false);` | **commType (uint8_t)**: Communication type (compulsory).<br>**param (Parameter\*)**: Target parameter (optional).<br>**value (float)**: Parameter value (optional).<br>**interpretResponse (bool)**: Interpret response (optional). |

---

## **List of Parameters**

Below are the motor parameters available, including their indexes and descriptions:

| **Parameter Name**          | **Index** | **Type**  | **Description**                                                                          |
|-----------------------------|-----------|-----------|------------------------------------------------------------------------------------------|
| `RUN_MODE`                 | `0x0570`  | `INT8`    | Controls the operation mode: 0 (off), 1 (position), 2 (speed), 3 (current).              |
| `SPEED_MAX_CURRENT`        | `0x1870`  | `FLOAT`   | Maximum current allowed in speed mode (in amps).                                         |
| `SPEED_TARGET`             | `0x0A70`  | `FLOAT`   | Target speed for the motor in speed mode (rad/s).                                        |
| `POSITION_SPEED_LIMIT`     | `0x1770`  | `FLOAT`   | Maximum speed in position mode (rad/s).                                                 |
| `POSITION_TARGET`          | `0x1670`  | `FLOAT`   | Target position for the motor in position mode (rad).                                    |
| `MECH_POS`                 | `0x1970`  | `FLOAT`   | Current mechanical position of the motor (rad).                                          |
| `SPEED_ACCELERATION`       | `0x2270`  | `FLOAT`   | Acceleration limit in speed mode for ROB03 (rad/s²).                                     |
| `POSITION_03_SPEED`        | `0x2470`  | `FLOAT`   | Speed for position control for ROB03 (rad/s).                                            |
| `POSITION_ACCELERATION`    | `0x2570`  | `FLOAT`   | Acceleration limit in position control for ROB03 (rad/s²).                                |

---

## **Command Formation**

Below is an overview of how CAN commands are formed for the motor:

| **Communication Type**      | **Description**                                                                                                                                                                          | **Example Command**                                                                                  |
|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| `COMM_TYPE_WRITE (18)`      | Writes a parameter value to the motor. The data area includes the parameter index and the value in IEEE754 (little-endian for floats).                                                   | `ID: 0x1200FD7F Data: 0x18 0x70 0x00 0x00 0x00 0x00 0xC0 0x41` (Write RUN_MODE = 2).                  |
| `COMM_TYPE_READ (17)`       | Reads a parameter value from the motor. The data area includes the parameter index, and the motor responds with the current value.                                                       | `ID: 0x1100FD7F Data: 0x19 0x70 0x00 0x00 0x00 0x00 0x00 0x00` (Read MECH_POS).                      |
| `COMM_TYPE_ENABLE (3)`      | Enables the motor. The data area is all zeros.                                                                                                                                           | `ID: 0x0300FD7F Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.                                      |
| `COMM_TYPE_DISABLE (4)`     | Disables the motor. The data area is all zeros.                                                                                                                                          | `ID: 0x0400FD7F Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.                                      |
| `COMM_TYPE_RESET (6)`       | Resets the motor's position to zero. The data area is `0x01 0x00` followed by zeros.                                                                                                     | `ID: 0x0600FD7F Data: 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.                                      |

---

### **General Notes on Command Formation**:
1. **Message ID**:
   - The message ID includes the communication type, host ID, and motor CAN ID.
   - Example: `0x1200FD7F` represents a write command(comm type 18) to motor `127` from host `253`.

2. **Data Area**:
   - Parameter index: First 2 bytes.
   - Reserved: Next 2 bytes (always `0x00 0x00`).
   - Value: Remaining bytes based on the parameter type (e.g., IEEE754 for floats). Float values are encoded in IEEE754 single-precision format in little-endian order.
---

compile command for motor_control.cpp: g++ -o motor_control motor_control.cpp -lstdc++ -lsetupapi

