#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#define HOST_CAN          253
#define DEFAULT_MOTOR_CAN 127

#define HEADER_SIZE         2 * 2
#define EXTENDED_SIZE       4 * 2
#define DATA_LENGTH_SIZE    1 * 2
#define DATA_SIZE           8 * 2
#define ENDING_SIZE         2 * 2
#define COMMAND_SIZE        (HEADER_SIZE + EXTENDED_SIZE + DATA_LENGTH_SIZE + DATA_SIZE + ENDING_SIZE)

enum com_type{
    GET_MCU_ID,
    OPERATIONAL,
    COMMAND_2,
    MOTOR_ENABLE,
    WRITE_PARAMETER = 18,
};

enum param_index{
    RUN_MODE        = 0x7005,
    SPD_MODE        = 0x700a,
    CURRENT_LIMIT   = 0x7018,
};

enum run_mode{
    OPERATION_CONTROL_MODE,
    POSITION_MODE,
    SPEED_MODE,
    CURRENT_MODE
};

enum err_t{
    ERR_OK,
    ERR_NOT
};

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

class Motor{
public:
    Motor();
    Motor(int motor_CAN_);

    //~Motor();

    void motor_enable();
    void print_debug();
    void setParameter(param_index, run_mode);
    void setParameter(param_index, int);
    void setParameter(param_index, float);
    void setParameter(param_index param, double value){
        setParameter(param, (float) value);
    }

private:
    int motor_CAN;

    char send_command[COMMAND_SIZE];
    char recv_command[COMMAND_SIZE];

    char *header_f;
    char *extend_f;
    char *data_length_f;
    char *data_f;
    char *ending_f;

    void init_command();

    void build_extended_frame(com_type);

    void parameter_handler(int);
    
};

#endif