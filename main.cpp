#include <stdio.h>
#include "command.hpp"

int main(){

    init();

    // command_construct(253, 1  , 0);
    // print_debug();
    // command_construct(253, 1, 1);
    // print_debug();
    // command_construct(253, 1, 3);
    // print_debug();
    command_construct(253, 1, 4);
    print_debug();
    // command_construct(253, 1, 6);
    // print_debug();
    // command_construct(253, 127, 7);
    // print_debug();
    // command_construct(253, 127, 17);
    // print_debug();
    // command_construct(253, 1, 18);
    // print_debug();

    //printf("\nPrint float value: %x\n", 12.53);
    //float_to_binary(12.53);
    return 0;
}