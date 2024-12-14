#ifndef __ENCODER_CONTROL_H__
#define __ENCODER_CONTROL_H__

#include "mbed.h"
#include "CAN.h"

// Define CAN pins for Arduino GIGA R1
mbed::CAN can1(PB_5, PB_13); // TX: PB_5, RX: PB_13

/*
@todo: implement error handling for functions
*/

#if DEBUG
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) do { Serial.print(__VA_ARGS__); Serial.println(); } while (0)
#else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
#endif

/*
@func: enum of available baudrates
*/
//

enum class BaudRate {
    B_1000K = 0x00,
    B_800K  = 0x01,
    B_500K  = 0x02,
    B_400K  = 0x03,
    B_250K  = 0x04,
    B_200K  = 0x05,
    B_125K  = 0x06,
    B_100K  = 0x07,
    B_80K   = 0x08,
    B_50K   = 0x09,
    B_40K   = 0x0a,
    B_20K   = 0x0b,
    B_10K   = 0x0c,
    B_5K    = 0x0d,
    B_3K    = 0x0e
};

/*
@func: enum of spin directions
*/

enum class SpinDirection {
    CLOCKWISE = 0x00,
    ANTI_CLOCKWISE = 0x01
};

/*
@func: enum of return rate
*/
enum class ReturnRate {
    Hz_1_10 = 0x00,
    Hz_2_10 = 0x01,
    Hz_5_10 = 0x02,
    Hz_1    = 0x03,
    Hz_2    = 0x04,
    Hz_5    = 0x05,
    Hz_10   = 0x06,
    Hz_20   = 0x07,
    Hz_50   = 0x08,
    Hz_100  = 0x09,
    Hz_125  = 0x0a,
    Hz_200  = 0x0b,
    Hz_1000 = 0x0c,
    Hz_2000 = 0x0d,
    SINGLE_RETURN = 0x0e
};

enum class SingleReg {
  BAUD        = 0x04,
  ANGLE       = 0x11,
  REVOLUTION  = 0x12,
  ANGSPEED    = 0x13
};

/*
@func: Initialize CAN bus at a speed, default set to 250000
@param: (uint32_t) baudRate -> the baud rate used for the communication
@ret:
*/

void initializeCAN(uint32_t baudRate = 250000){
    if(baudRate == 250000){
    if (can1.frequency(baudRate)) {
        DEBUG_PRINTLN("CAN bus initialized at 1 Mbps");
    } else {
        DEBUG_PRINTLN("Failed to initialize CAN bus");
        while (true);
    }
    } else{
    if(can1.frequency(baudRate)){
        DEBUG_PRINTLN("CAN bus initialized at some speed");
    } else {
        DEBUG_PRINTLN("Failed to initialize CAN bus");
        while(true);
    }
    }
}

class Encoder {

//==================== PUBLIC ZONE ===================//

public:
/*
@func: Initialize encoder object with default ID
@param:
@ret:
*/
Encoder(){
  samplingTime = 0.1;
  encoderID = 0x50;
}

/*
@func: Initialize encoder object with a designated ID
@param: (uint8_t) id -> the id of the encoder object
@ret: 
*/

Encoder(uint8_t id, float samplingTime = 0.1){
    encoderID = id;
}


/*
@func:  read the data on the bus, the function must be called before
        calling getAngle, getAngleVelocity and getRotations for latest
@param:
@ret: 
*/

void readData_continuous(){
    mbed::CANMessage msg;

    if(can1.read(msg)){
        if(msg.id == encoderID && msg.len == 8){
            uint16_t angleRaw = (msg.data[3] << 8) | msg.data[2];
            angle = angleRaw * 360.0 / 32768.0;

            // Extract angular velocity (bytes 4 and 5)
            int16_t angularVelocityRaw = (msg.data[5] << 8) | msg.data[4];
            angularVelocity = angularVelocityRaw * 360.0 / (32768.0 * samplingTime); // Assuming default sampling time of 0.1s

            // Extract rotations (bytes 6 and 7)
            rotations = (msg.data[7] << 8) | msg.data[6];
        }
    }
}

/*
@func: get the angle in float data type
@param: NaN
@ret: angle, in degree
*/

float getAngle(){
    return angle;
}

/*
@func: get the angular velocity in float data type
@param: NaN
@ret: angular velocity, in degree
*/

float getAngularVelocity(){
    return angularVelocity;
}

/*
@func: get the current rotations in int16_t type
@param: NaN
@ret: rotations, in int16_t
*/

int16_t getRotation(){
    return rotations;
}

/*
@func: read the angle on direct register, prevent flooding in continuous mode
        please use getAngle after this to get the data
@param:
@ret:
*/

void readAngle(){
  readSingleReg(SingleReg::ANGLE);
  mbed::CANMessage msg;

  if(can1.read(msg)){
    if(msg.id == encoderID && msg.len == 8){
      uint16_t angleRaw = (msg.data[3] << 8) | msg.data[2];
      angle = angleRaw * 360.0 / 32768.0;
    }
  }
}

/*
@func: read the revolution on direct register, prevent flooding in continuous mode
        please use getRotation after this to get the data
@param:
@ret:
*/

void readRotation(){
  readSingleReg(SingleReg::REVOLUTION);
  mbed::CANMessage msg;

  if(can1.read(msg)){
    if(msg.id == encoderID && msg.len == 8){
      rotations = (msg.data[3] << 8) | msg.data[2];
    }
  }
}

/*
@func: read the angulare velocity on direct register, prevent flooding in continuous mode
        please use readAngularSpeed after this to get the data
@param:
@ret:
*/

void readAngularSpeed(){
  readSingleReg(SingleReg::ANGSPEED);
  mbed::CANMessage msg;

  if(can1.read(msg)){
    if(msg.id == encoderID && msg.len == 8){
      uint16_t angularVelocityRaw = (msg.data[5] << 8) | msg.data[4];
      angularVelocity = angularVelocityRaw * 360.0 / (32768.0 * samplingTime); // Assuming default sampling time of 0.1s
    }
  }
}

/*
@func: set the measuring angle to desired angle
*/
void setAngle(uint8_t target_angle){
    unlockSettings();
    uint8_t data[8] = {0xff, 0xaa, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00};  

    uint32_t ang_val = target_angle * 32768 / 360;

    data[3]  = ang_val && 0x000000ff;
    data[4]  = ( ang_val >> 16 ) && 0x000000ff; 

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
}

/*
@func: set the amount of rotation detected to desired amount
@param: (uint32_t) a target_rotation number larger than and equal to 0
@ret:
*/

void setRotation(uint32_t target_rotation){
    unlockSettings();
    uint8_t data[8] = {0xff, 0xaa, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = target_rotation && 0x000000ff;
    data[4]  = ( target_rotation >> 8 ) && 0x000000ff; 

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
}

/*
@func: set the baudrate of the CAN bus
@param: Baud Rate of (BaudRate class type)
@ret
*/

void setBaudRate(BaudRate target_baudrate){
    unlockSettings();

    uint8_t data[8] = {0xff, 0xaa, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = (uint8_t) target_baudrate;

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
    saveSettings();
}

/*
@func:  set rotation direction 
@param: direction , either clockwise or anti-clockwise, 
        defined by type SpinDirection
@ret: 
*/

void setRotationDirection(SpinDirection target_direction){
    unlockSettings();

    uint8_t data[8] = {0xff, 0xaa, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = (uint8_t) target_direction;

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
    saveSettings();
}

/*
@func:  set rotation direction 
@param: direction , either clockwise or anti-clockwise, 
        defined by type SpinDirection
@ret: 
*/

void setCAN_ID(uint16_t target_CAN_ID){
    unlockSettings();

    uint8_t data[8] = {0xff, 0xaa, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = target_CAN_ID && 0x000000ff;
    data[4]  = (target_CAN_ID >> 8 ) && 0x000000ff;

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
    saveSettings();
}

/*
@func:  set rotation direction 
@param: direction , either clockwise or anti-clockwise, 
        defined by type SpinDirection
@ret: 
*/

void setReturnRate(ReturnRate target_return_rate){
    unlockSettings();

    uint8_t data[8] = {0xff, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = (uint8_t) target_return_rate;

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
    saveSettings();
}

void readSingleReg(SingleReg target_reg){
    uint8_t data[8] = {0xff, 0xaa, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00};  

    data[3]  = (uint8_t) target_reg;

    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
    saveSettings();

}

//==================== PRIVATE ZONE ===================//

private:
    uint8_t encoderID;
    float   angle;              // in degree
    float   angularVelocity;    // in degree/s
    int16_t rotations;          //
    
    float samplingTime;         // in s

/*
@func:  must unlock the settings before we set any parameter
        will be used in any set function, must keep private
@param:
@ret:
*/

void unlockSettings(){
    uint8_t data[8] = {0xff, 0xaa, 0x69, 0x88, 0xb5, 0x00, 0x00, 0x00};
    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
}

/*
@func: to save settings for future use, must use this command
@param:
@ret:
*/
void saveSettings(){
    uint8_t data[8] = {0xff, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    mbed::CANMessage msg(0x50, data, 5, CANData, CANStandard);

    if(can1.write(msg)){
    } else{
    }
}

};

#endif
