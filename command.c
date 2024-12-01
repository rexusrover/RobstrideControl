#include "command.h"
#include <stdio.h>

#define HEADER_SIZE     2 * 2
#define EXTENDED_SIZE   4 * 2
#define NUMBER_SIZE     1 * 2
#define DATA_SIZE       8 * 2
#define ENDING_SIZE     2 * 2

//============================================================//
//                                                            //
//                     SEND_COMMAND FORMAT                    //
//                                                            //
// Each command will be represented by 17 bytes, each byte    //
// is represented by 2 chars, 1 char corresponds to a hex     //
// digit                                                      //
//                                                            //
//  Structure:                                                //
//  2 bytes | 4 bytes        | 1 byte               |         //
// -----------------------------------------------------------//
//  41 54   | XX XX XX XX    | 08                   |         //
//  header  | Extended frame | Number of data bytes |         //
//                                                            //
//  8 bytes                 | 2 bytes               |         //
// -----------------------------------------------------------//
//  XX XX XX XX XX XX XX XX | 0d 0a                 |         //
//  Data bytes              | Ending frame          |         //
//
// total send_command frame
char send_command[34] = {0};

// sections to edit within the command
char *header_f      = send_command;
char *extended_f    = send_command + 4;
char *number_f      = send_command + 12;
char *data_f        = send_command + 14;
char *end_f         = send_command + 30;

//                                                            //
//============================================================//

int preset_CAN_ID = 1;
int baud_rate     = 1000000;

void init();
void craft_databits(int command);

void command_0();
void command_1();
void command_2();
void command_7();

void command_construct(int host_CAN_ID, int target_CAN_ID, int command);
void print_debug();

void init(){
    // header frame
    send_command[0]  = '4';
    send_command[1]  = '1';
    send_command[2]  = '5';
    send_command[3]  = '4';

    // number of data bits
    /*
    send_command[12] = '0';
    send_command[13] = '8';
    */
    number_f[0]      = '0';
    number_f[1]      = '8';

    // ending frame
    /*
    send_command[30] = '0';
    send_command[31] = 'd';
    send_command[32] = '0';
    send_command[33] = 'a';
    */
    end_f[0]         = '0';
    end_f[1]         = 'd';
    end_f[2]         = '0';
    end_f[3]         = 'a';
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
    extended_frame = command;               // the command first enters the command
    if(command != 7)
        extended_frame <<= 16;              // make space for 8 blank bytes and 8 host_CAN_ID bytes
    else{
        extended_frame <<= 8;
        extended_frame += preset_CAN_ID;
        extended_frame <<= 8;
    }
    extended_frame += host_CAN_ID;          // bits ?? - ?? = host_CAN_ID
    extended_frame <<= 8;                   // make space for 8 blank bytes and 8 target_CAN_ID bytes
    extended_frame += target_CAN_ID;        // bits ?? - ?? = target_CAN_ID
    extended_frame <<= 3;                   // make space for 3 bits - 100 at the end
    extended_frame += 4;                    // 3 last bits = 100

    // Debug message
    /*
    printf("%lld\n", extended_frame);
    printf("%x\n", extended_frame);
    */

    int temp_4bit;
    i = EXTENDED_SIZE - 1;
    while(extended_frame > 0){
        temp_4bit = extended_frame % 16;
        if(temp_4bit < 10) extended_f[i] = (char) (temp_4bit + 48);
        else extended_f[i] = (char) (temp_4bit + 97 - 10);

        extended_frame >>= 4;
        i--;
    }

    for(i = 0; i < EXTENDED_SIZE; i++){
        if(extended_f[i] == 0) extended_f[i] = '0';
        else break;
    }

    craft_databits(command);

    printf("Extended frame part: ");
    for(i = 0; i < EXTENDED_SIZE; i++){
        printf("%c", extended_f[i]);
    }
    printf("\n");
    
}

void craft_databits(int command){
    // little indian mode
    switch(command){
        case 0:
            command_0();                // retrieve host ID
            break;
        case 1:
            command_1();                // ?? control
            break;
        case 2:
            command_2();                // this might be the response?
            break;
        case 3:                         // command 3 - motor enable operation
        case 4:                         // command 4 - motor stop
            command_0();                // motor enable
            break;
        case 6:                         // command 6 - set mechanical position to 0
            command_0();                // set most data bits to 0
            data_f[1] = '1';            // set byte data to 1, or 2nd digit represented in data frame
            break;
        case 7:                         // command 7 - manually assign motor_CAN_ID
            command_0();
            break;
        case 17:
            break;
        case 18:
            break;
        case 21:
            command_0();
            break;
        case 22:                        // command 22 - manually set baud_rate
            command_0();
            switch (baud_rate)
            {
            case 500000:
                data_f[1] = '2';
                break;
            case 250000:
                data_f[1] = '3';
                break;
            case 125000:  
                data_f[1] = '4';  
            case 1000000:
            default:
                data_f[1] = '1';
                break;
            }
        default:
            break;
    }
}

void command_0(){
    int i = 0;
    for(; i < DATA_SIZE ;i++){
        data_f[i] = '0';
    }
}

void command_1(){

}

void command_2(){

}