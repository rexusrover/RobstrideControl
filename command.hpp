#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__


// ID assigned manually, default to be 1
extern int preset_CAN_ID;
extern int baud_rate;
extern int parameter;

extern int data_int;
extern float data_float;

extern float target_angle;
extern float target_omega;
extern float target_kp;
extern float target_kd;

void init();
void print_debug();

void command_construct(int host_CAN_ID, int target_CAN_ID, int command);
//void float_to_binary(float value);
#endif