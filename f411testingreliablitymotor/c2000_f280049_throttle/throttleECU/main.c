#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "throttle_ecu.h"

/* ── DRV8873H bring-up test ──────────────────────────────────────────────────
 * Define DRV_TEST to replace the full throttle ECU with a bare-metal motor
 * driver probe: ramps 25% → 50% → 75% → coast, loops forever.
 * Comment out (or undefine) to restore normal operation.
 * ─────────────────────────────────────────────────────────────────────────── */
/* #define DRV_TEST */   /* DISABLED — uncomment only for hardware bring-up */

#ifdef DRV_TEST
extern void DRV8873_runTest(void);
#endif

void main(void)
{
    Device_init();

    Interrupt_initModule();
    Interrupt_initVectorTable();

#ifdef DRV_TEST
    /* Bare-metal test — never returns */
    DRV8873_runTest();
#else
    Board_initHW();
    Board_digitalRelay(1U);  /* TODO: remove — temp solenoid enable for bring-up */
    Throttle_init();

    EINT;
    ERTM;

    for (;;) {
        Throttle_runOnce();
    }
#endif
}
