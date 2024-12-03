#include "command.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



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

Motor::Motor(){
    init_commands();
    init_values();
}

Motor::~Motor(){
    free(send_command);
    free(recv_command);
    free(header_f);
    free(extended_f);
    free(number_f);
    free(data_f);
    free(end_f);
}

void Motor::init_commands(){
    send_command[0]  = '4';
    send_command[1]  = '1';
    send_command[2]  = '5';
    send_command[3]  = '4';

    number_f[0]      = '0';
    number_f[1]      = '8';

    end_f[0]         = '0';
    end_f[1]         = 'd';
    end_f[2]         = '0';
    end_f[3]         = 'a';
}

void Motor::init_values(){

}

void Motor::print_debug(){
    int i;
    printf("\nCurrent full command: ");
    for(i = 0; i < COMMAND_SIZE; i++){
        printf("%c", send_command[i]);
        if(i % 2 == 1) printf(" ");
    }
    printf("\n");
}


char send_command[34] = {0};

// sections to edit within the command
char *header_f      = send_command;
char *extended_f    = send_command + 4;
char *number_f      = send_command + 12;
char *data_f        = send_command + 14;
char *end_f         = send_command + 30;

//                                                            //
//============================================================//

// define values
int preset_CAN_ID   = 1;
int baud_rate       = 1000000;
int parameter       = 0x700a;

int data_int        = 2;
float data_float    = 1.0;

float target_angle  = 1.0;
float target_omega  = -1.0;
float target_kp     = 1.0;
float target_kd     = -1.0;

void init();
void craft_databits(int command);

void command_0();
void command_1(float , float, float , float);
void command_2();
void command_7();
void command_17(int parameter);
void command_18(int parameter);
void parameter_handler(int parameter);
void data_handler(int parameter);
void float_to_binary(float value);

void command_construct(int host_CAN_ID, int target_CAN_ID, int command);
void print_debug();

void init(){
    // header frame
    send_command[0]  = '4';
    send_command[1]  = '1';
    send_command[2]  = '5';
    send_command[3]  = '4';

    number_f[0]      = '0';
    number_f[1]      = '8';

    end_f[0]         = '0';
    end_f[1]         = 'd';
    end_f[2]         = '0';
    end_f[3]         = 'a';
}

void print_debug(){
    int i;
    printf("\n========================================\n");
    printf("Current full command: ");
    
    for(i = 0; i < 34; i++){
        if(i % 2==0) printf("%c", send_command[i]);
        else printf("%c ", send_command[i]);
    }
    printf("\n========================================\n");
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
            command_1(target_angle, target_omega, target_kp, target_kd);                // ?? control
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
        case 17:                        // command 17 - read data
            command_0();                // first occupy all data with 0
            command_17(parameter);    // then fill the remaining fields according to data sheet
            break;
        case 18:                        // command 18 - write data
            command_0();                // first occupy all data with 0
            command_18(parameter);    // then fill the remaining fields according to data sheet
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

void command_1(float angle, float rad, float kp, float kd){
    int data_field  = 0x0;
    int step        = 65535;
    int temp;
    int i;

    // angle (-4pi -> 4pi = 0 -> 65535)
    float angle_quant = (4.0 - (-4.0)) / (float) step;
    temp = (int)((angle - (-4.0)) / angle_quant);
    data_field = temp;
    for( i = 3; i >= 0 ;i--){
        if(data_field % 16 < 10) data_f[i] = (char) (data_field % 16) + 48;
        else data_f[i] = (data_field % 16 + 97 - 10);
        data_field >>= 4;
    }
    data_field = 0;
    // rad (-44rad/s -> 44rad/s = 0 -> 65535)
    float rad_quant = (44.0 - (-44.0)) / (float) step;
    temp = (int)((rad - (-44.0)) / rad_quant);
    data_field = temp;
    for( i = 3; i >= 0 ;i--){
        if(data_field % 16 < 10) data_f[4 + i] = (char) (data_field % 16) + 48;
        else data_f[4 + i] = (data_field % 16 + 97 - 10);
        data_field >>= 4;
    }
    data_field = 0;
    // kp (0 -> 500 = 0 -> 65535)
    float kp_quant = (500.0 - 0.0) / (float) step;
    temp = (int)((angle - 0.0) / kp_quant);
    data_field = temp;
    for( i = 3; i >= 0 ;i--){
        if(data_field % 16 < 10) data_f[8 + i] = (char) (data_field % 16) + 48;
        else data_f[8 + i] = (data_field % 16 + 97 - 10);
        data_field >>= 4;
    }
    data_field = 0;
    // kd (0 -> 5 = 0 -> 35655)
    float kd_quant = (5.0 - 0.0) / (float) step;
    temp = (int)((angle - 0.0) / kd_quant);
    data_field += temp;
    for( i = 3; i >= 0 ;i--){
        if(data_field % 16 < 10) data_f[12 + i] = (char) (data_field % 16) + 48;
        else data_f[12 + i] = (data_field % 16 + 97 - 10);
        data_field >>= 4;
    }
    
}

void command_2(){

}

void command_18(int parameter){
    parameter_handler(parameter);
    data_handler(parameter);
}

void command_17(int parameter){
    parameter_handler(parameter);
}

void parameter_handler(int parameter){
    if(parameter % 16 < 10) data_f[1] = (char) (parameter % 16 + 48);
    else data_f[1] = (char)(parameter % 16 + 97 - 10);

    parameter >>= 4;

    if(parameter % 16 < 10) data_f[0] = (char) (parameter % 16 + 48);
    else data_f[0] = (char)(parameter % 16 + 97 - 10);

    parameter >>= 4;

    if(parameter % 16 < 10) data_f[3] = (char) (parameter % 16 + 48);
    else data_f[3] = (char)(parameter % 16 + 97 - 10);

    parameter >>= 4;

    if(parameter % 16 < 10) data_f[2] = (char) (parameter % 16 + 48);
    else data_f[2] = (char)(parameter % 16 + 97 - 10);
}

void data_handler(int parameter){
    char *data_zone = data_f + 8;
    switch (parameter){
        case 0x7005:
            if(data_int < 0 || data_int > 4) data_zone[1] = '0';
            else data_zone[1] = (char) (data_int + 48);
            break;
        case 0x701d:
            // this logic needs to be implemented
            break;
        case 0x7006:
        case 0x700a:
            float_to_binary(data_float);
            break;
        case 0x700b:
        case 0x7010:
        case 0x7011:
        case 0x7014:
        case 0x7016:
        case 0x7017:
        case 0x7018:
            float_to_binary(data_float);
            break;
        case 0x7019:
        case 0x701a:
        case 0x701b:
        case 0x701c:
        case 0x701e:
        case 0x701f:
        case 0x7020:
            float_to_binary(data_float);
        default:
            break;
    }
}

void float_to_binary(float value){
    char *data_zone = data_f + 8;

    int hex_value = 0;
    int integer_part;
    float fraction_part;

    // sign bit
    if(value < 0) {
        hex_value++; value = -value;
    }
    hex_value <<= 8;

    integer_part  = (int) value;
    fraction_part = value - (float) integer_part;

    char *temp_integer = (char*) malloc(sizeof(char) * 23);

    // process int part
    int i = 0;
    //printf("Integer part: %d\n", integer_part);
    //printf("Fraction part: %f\n", fraction_part);
    while(integer_part >0){
        if(integer_part % 2 == 1) temp_integer[i] = '1';
        else temp_integer[i] = '0';

        integer_part >>= 1;
        i++;
    }

    int int_size        = i - 1;
    int fraction_size   = 23 - int_size;
    //printf("Number of bits: %d \n", i);
    hex_value += 127 + int_size;

    char* temp_fraction = (char *) malloc(sizeof(char) * fraction_size);
    i = 0;
    while(i < fraction_size){
        if(fraction_part == 0){
            temp_fraction[i] = '0';
            i++;
            continue;
        }
        if(fraction_part >= 0.5){
            temp_fraction[i] = '1';
            fraction_part = 2 * fraction_part - 1;
        } else{
            temp_fraction[i] = '0';
            fraction_part = 2 * fraction_part;
        }
        i++;
    }

    /*
    printf("Fraction part in binary: ");
    for(i = 0; i < fraction_size; i++){
        printf("%c", temp_fraction[i]);
    }
    */

    for(i = int_size - 1; i >= 0; i--){
        hex_value <<= 1;
        if(temp_integer[i] == '1') hex_value++;
    }

    for(i = 0; i < fraction_size; i++){
        hex_value <<= 1;
        if(temp_fraction[i] == '1') hex_value++;
    }

    //printf("\nHex value : %x\n", hex_value);

    for(i = 0; i < 4; i++){
        if(hex_value % 16 < 10) data_zone[1 + 2*i] = (char) (hex_value%16 + 48);
        else data_zone[1+ 2*i] = (char) (hex_value % 16 + 97 - 10);
        hex_value >>= 4;
        if(hex_value % 16 < 10) data_zone[2*i] = (char) (hex_value%16 + 48);
        else data_zone[2*i] = (char) (hex_value % 16 + 97 - 10);
        hex_value >>= 4;
    } 

    free(temp_fraction);
    free(temp_integer);
    //delete(temp_fraction);
    //delete(temp_integer);

    return;
}