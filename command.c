#include "command.h"
#include <stdio.h>

char send_command[34] = {0};

void init(){
    // header frame
    send_command[0]  = '4';
    send_command[1]  = '1';
    send_command[2]  = '5';
    send_command[3]  = '4';

    // number of data bits
    send_command[12] = '0';
    send_command[13] = '8';

    // ending frame
    send_command[30] = '0';
    send_command[31] = 'd';
    send_command[32] = '0';
    send_command[33] = 'a';
}

void print_debug(){
    int i;
    printf("Current full command: ");
    for(i = 0; i < 34; i++){
        if(i % 2==0) printf("%c", send_command[i]);
        else printf("%c ", send_command[i]);
    }
    printf("\n");
}

void command_construct(int host_CAN_ID, int target_CAN_ID, int command){
    int i;

    unsigned int extended_frame = 0x0;
    extended_frame = command;           // the command first enters the command
    extended_frame <<= 16;              // make space for 8 blank bytes and 8 host_CAN_ID bytes
    extended_frame += host_CAN_ID;      // bits ?? - ?? = host_CAN_ID
    extended_frame <<= 8;               // make space for 8 blank bytes and 8 target_CAN_ID bytes
    extended_frame += target_CAN_ID;    // bits ?? - ?? = target_CAN_ID
    extended_frame <<= 3;               // make space for 3 bits - 100 at the end
    extended_frame += 4;                // 3 last bits = 100

    // Debug message
    /*
    printf("%lld\n", extended_frame);
    printf("%x\n", extended_frame);
    */

    int temp_4bit;
    i = 11;
    while(extended_frame > 0){
        temp_4bit = extended_frame % 16;
        if(temp_4bit < 10) send_command[i] = (char) (temp_4bit + 48);
        else send_command[i] = (char) (temp_4bit + 97 - 10);

        extended_frame >>= 4;
        i--;
    }

    printf("Extended frame part: ");
    for(i = 4; i < 12; i++){
        printf("%c", send_command[i]);
    }
    printf("\n");
    
}

void craft_databits(int command, int data){
    // little indian mode
}