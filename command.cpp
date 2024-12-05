#include <stdio.h>
#include <stdlib.h>

#include "command.hpp"

Motor::Motor(){
    motor_CAN = DEFAULT_MOTOR_CAN;
    init_command();
}

Motor::Motor(int motor_CAN_){
    motor_CAN = motor_CAN_;
    init_command();
}

void Motor::motor_enable(){
    build_extended_frame(MOTOR_ENABLE);
}

void Motor::build_extended_frame(com_type com){
    int com_value = com;

    unsigned int extended_frame = 0x0;
    extended_frame = com_value;               // the command first enters the command
    
    if(com_value != 7)
        extended_frame <<= 16;              // make space for 8 blank bytes and 8 host_CAN_ID bytes
    else{
        extended_frame <<= 8;
        //extended_frame += preset_CAN_ID;
        extended_frame <<= 8;
    }

    extended_frame += HOST_CAN;          // bits ?? - ?? = host_CAN_ID
    extended_frame <<= 8;                   // make space for 8 blank bytes and 8 target_CAN_ID bytes
    extended_frame += motor_CAN;        // bits ?? - ?? = target_CAN_ID
    extended_frame <<= 3;                   // make space for 3 bits - 100 at the end
    extended_frame += 4;                    // 3 last bits = 100

    // Debug message 
    // printf("%lld\n", extended_frame);
    // printf("%x\n", extended_frame);
    
    int i;

    int temp_4bit;
    i = EXTENDED_SIZE - 1;
    while(extended_frame > 0){
        temp_4bit = extended_frame % 16;
        if(temp_4bit < 10) extend_f[i] = (char) (temp_4bit + 48);
        else extend_f[i] = (char) (temp_4bit + 97 - 10);

        extended_frame >>= 4;
        i--;
    }

    for(i = 0; i < EXTENDED_SIZE; i++){
        if(extend_f[i] == 0) extend_f[i] = '0';
        else break;
    }

}

void Motor::print_debug(){
    int i;
    printf("\nCurrent command is: \n");
    for( i = 0 ; i < COMMAND_SIZE; i++){
        printf("%c", send_command[i]);
        if(i % 2 == 1) printf(" ");
    }
    printf("\n");
}

void Motor::init_command(){
    header_f        = send_command;
    extend_f        = send_command + HEADER_SIZE;
    data_length_f   = extend_f + EXTENDED_SIZE;
    data_f          = data_length_f + DATA_LENGTH_SIZE;
    ending_f        = data_f + DATA_SIZE;

    int i;
    for( i = 0 ; i < COMMAND_SIZE ; i++){
        send_command[i] = '0';
        recv_command[i] = '0';
    }

    header_f[0] = '4';
    header_f[1] = '1';
    header_f[2] = '5';
    header_f[3] = '4';

    data_length_f[0] = '0';
    data_length_f[1] = '8';

    ending_f[0] = '0';
    ending_f[1] = 'd';
    ending_f[2] = '0';
    ending_f[3] = 'a';
}

void Motor::setParameter(param_index param, run_mode runmode){
    int runmode_value = runmode;
    setParameter(param, runmode_value);
}

void Motor::setParameter(param_index param, int data_int){
    char *data_zone = data_f + 8;
    int parameter = param;

    parameter_handler(parameter);

    int i;
    for(i = 0; i < 4 ; i++){
        if(data_int % 16 > 10) data_zone[1 + 2*i] = (char) (data_int % 16 + 97 - 10);
        else data_zone[1 + 2*i] = (char) (data_int % 16 + 48);
        data_int >>= 4;
        if(data_int % 16 > 10) data_zone[2*i] = (char) (data_int % 16 + 97 - 10);
        else data_zone[2*i] = (char) (data_int % 16 + 48);
        data_int >>= 4;
    }
    
}

void Motor::parameter_handler(int parameter){
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

    data_f[4] = '0';
    data_f[5] = '0';
    data_f[6] = '0';
    data_f[7] = '0';
}

void Motor::setParameter(param_index param, float value){
    int parameter = param;
    parameter_handler(parameter);

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