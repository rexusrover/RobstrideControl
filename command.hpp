#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#define HEADER_SIZE     2 * 2
#define EXTENDED_SIZE   4 * 2
#define NUMBER_SIZE     1 * 2
#define DATA_SIZE       8 * 2
#define ENDING_SIZE     2 * 2

#define COMMAND_SIZE    (HEADER_SIZE + EXTENDED_SIZE + NUMBER_SIZE + DATA_SIZE + ENDING_SIZE)

class Motor{
public:
    Motor(){}

    ~Motor(){}

    void rx_command();
    void tx_command();

private:
    char send_command[COMMAND_SIZE] = {0};
    char recv_command[COMMAND_SIZE] = {0};

    // sections to edit within the command
    char *header_f      = send_command;
    char *extended_f    = send_command + 4;
    char *number_f      = send_command + 12;
    char *data_f        = send_command + 14;
    char *end_f         = send_command + 30;

    void init_commands();
    void init_values();
    void print_debug();

    int  getMCU_CAN_ID();               // command 0
    void manual_command();              // command 1
    void motor_enable();                // command 3
    void reset_position();              // command 6
    void setMotor_CAN_ID();             // command 7
    void getParameter();                // command 17
    void setParameter();                // command 18
    
};

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