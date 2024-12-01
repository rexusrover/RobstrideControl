#include <stdio.h>
#include "command.h"

int main(){

    init();

    command_construct(253,1,0);
    print_debug();
    command_construct(253, 127, 3);
    print_debug();
    command_construct(253, 127, 6);
    print_debug();
    command_construct(253, 127, 7);
    print_debug();
    return 0;
}