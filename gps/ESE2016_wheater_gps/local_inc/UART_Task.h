/*! \file UART_Task.h
    \brief UART task
    \author Andrea Maierhofer

    Wheater station uart.

*/
#ifndef UART_TASK_H_
#define UART_TASK_H_

#include <stdbool.h>
#include <stdint.h>
/* Drivers Header files - fall back to driverlib for gpio*/
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <inc/hw_memmap.h>


/*! \fn SetupUartTask
 *  \brief Setup UART tasks.
 *
 *  Task has priority 15 and receives 1kB of stack.
 *
 *  \return always zero. In case of error the system halts.
*/
int SetupUartTask(void);



#endif
