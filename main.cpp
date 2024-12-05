#include <stdio.h>
#include "command.hpp"

int main(){

    printf("Entering here\n");
    Motor j1;

    j1.motor_enable();
    j1.print_debug();
    j1.setParameter(RUN_MODE, SPEED_MODE);

    // under is unhandled command, pls dont use into motor
    // j1.setParameter(RUN_MODE, 1.0);
    j1.print_debug();
    j1.setParameter(SPD_MODE, 1.0);
    j1.print_debug();
    j1.setParameter(CURRENT_LIMIT, 23.0);
    j1.print_debug();
    printf("Out here\n");

    return 1;
}