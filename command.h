#ifndef __COMMAND_H__
#define __COMMAND_H__

void init();
void print_debug();

void command_construct(int host_CAN_ID, int target_CAN_ID, int command);
#endif