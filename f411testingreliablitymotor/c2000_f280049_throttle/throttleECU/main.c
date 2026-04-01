#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "throttle_ecu.h"

void main(void)
{
    Device_init();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    Board_initHW();
    Throttle_init();

    EINT;
    ERTM;

    for (;;) {
        Throttle_runOnce();
    }
}
