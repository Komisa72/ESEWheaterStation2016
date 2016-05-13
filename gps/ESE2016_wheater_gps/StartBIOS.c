/*
* CCSv6 project using TI-RTOS
*
*/

/*
* ======== StartBIOS.c ========
*/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>


/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

/*application header files*/
#include <ctype.h>
#include <string.h>

#include <UART_Task.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/

#include "BoosterPack.h"

// setup read sample rate with 1200 ms
#define READ_SAMPLE_RATE    1200

/* Forward declarations */
extern int SetupAltiudeTask(BoosterPackType boosterPack);
extern int SetupClockTask(uint32_t wait_ticks);
extern int setup_Temp_Task(BoosterPackType boosterPack);

/**
 * /fn main
 * /brief The main entry point of the program.
 * /return Always 0.
 */
int main(void)
{
    uint32_t ui32SysClock;
    BoosterPackType boosterPackThermo = BOOSTER_PACK_2;
    BoosterPackType boosterPackAltitude = BOOSTER_PACK_1;

    /* Call board init functions. */
    ui32SysClock = Board_initGeneral(120*1000*1000);
    (void)ui32SysClock; // We don't really need this (yet)

    Board_initI2C();

    (void) SetupUartTask();
    System_printf("Created UART Task\n");

#if USE_THERMO_CLICK
    /*Call task for temperature*/
    setup_Temp_Task(boosterPackThermo);
#endif

#if USE_ALTITUDE_CLICK
    (void) SetupAltiudeTask(boosterPackAltitude);
    System_printf("Created Altitude Task\n");
#endif

    (void) SetupClockTask(READ_SAMPLE_RATE);
    System_printf("Created Clock Task\n");

    /* will only print to the console upon calling flush or exit */
    System_printf("Start BIOS\n");
    System_flush();

    BIOS_start();

    return 0;
}

