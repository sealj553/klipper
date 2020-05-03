#include "init.h"
#include "internal.h"

#include "pin_mux.h"
#include "board.h"

void SystemInit(void) {
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

}
