#ifndef __COMMAND_H__
#define __COMMAND_H__


// ID assigned manually, default to be 1
extern int preset_CAN_ID;
extern int baud_rate;

void init();
void print_debug();

void command_construct(int host_CAN_ID, int target_CAN_ID, int command);
#endif