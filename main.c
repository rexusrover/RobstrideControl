#include <stdio.h>
#include "command.h"

int main(){

    init();

    command_construct(253,1,18);
    print_debug();
    return 0;
}