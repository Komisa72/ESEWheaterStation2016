/*
 * UART_Task.c
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */

#include <stdbool.h>
#include <inc/hw_memmap.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/

#include <driverlib/interrupt.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

#include <ti/sysbios/hal/hwi.h>
#include <inc/hw_ints.h>

#include <ctype.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <Board.h>

#include "BoosterPack.h"
#include "ClockTask.h"

static void TransferFunction(UArg arg0, UArg arg1);

/**
 * /fn ButtonFunction
 * /brief Interrupt function when User switch 1 is pressed.
 * /param arg not used.
 * /return void.
 */
static void ButtonFunction(UArg arg)
{
	GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
	/* turn off led 1 */
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
}

/**
 * /fn InitializeLedUserSwitch
 * /brief Initialize led D1 und USR SW1.
 * /return void.
 */
static void InitializeLedUserSwitch()
{
    uint32_t strength;
    uint32_t pinType;
    Hwi_Params buttonHWIParams;
    Hwi_Handle buttonHwi;
    Error_Block eb;

    // enable port N
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	// LED2
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    /* set pin gpio port to output */
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    /*configure pad as standard pin with default output current*/
    GPIOPadConfigGet(GPIO_PORTN_BASE, GPIO_PIN_1, &strength, &pinType);
    GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_1, strength, GPIO_PIN_TYPE_STD);

    /* turn off led 1 */
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);

    /* configure switch 1 with pull up as input on GPIO_PIN_0 as pull-up pin */
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigGet(GPIO_PORTJ_BASE, GPIO_PIN_0, &strength, &pinType);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, strength, GPIO_PIN_TYPE_STD_WPU);

    Error_init(&eb);
    Hwi_Params_init(&buttonHWIParams);
    buttonHWIParams.arg = 0;
    buttonHWIParams.enableInt = false;

    buttonHwi = Hwi_create(INT_GPIOJ_TM4C129, ButtonFunction, &buttonHWIParams, &eb);

    if (buttonHwi == NULL)
    {
    	System_abort("Button Hardware interrupt create failed.");
    }
    Hwi_enableInterrupt(INT_GPIOJ_TM4C129);
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);

}

/**
 * /fn SetupUartTask
 * /brief Setup UART tasks.
 *
 * Task has priority 15 and receives 1kB of stack.
 *
 * /return Always 0. In case of error the system halts.
 */
int SetupUartTask(void) {
	Task_Params taskTransferParams;
	Task_Handle taskTransfer;
	Error_Block eb;

	/* Enable and configure the peripherals used by the UART7 */
	SysCtlPeripheralEnable(GPIO_PORTC_BASE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	GPIOPinConfigure(GPIO_PC4_U7RX);
	GPIOPinConfigure(GPIO_PC5_U7TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	UART_init();

	InitializeLedUserSwitch();

	Error_init(&eb);
	Task_Params_init(&taskTransferParams);
	taskTransferParams.stackSize = 1024;
	taskTransferParams.priority = 15;
	taskTransfer = Task_create((Task_FuncPtr) TransferFunction,
			&taskTransferParams, &eb);
	if (taskTransfer == NULL) {
		System_abort("UART task create failed");
	}

	return (0);
}

/**
 * /fn TransferFunction
 * /brief Functions transfers read values from altitude/thermo click via uart7.
 *
 * /param arg0 not used.
 * /param arg1 not used.
 * /return void.
 */
static void TransferFunction(UArg arg0, UArg arg1) {
	TransferMessageType message;
	UInt firedEvents;
	UART_Handle uart7;
	UART_Params uart7Params;
	char result[20 + 1]; // +1 for \0, must be transferred
	int length;
	int precision;
	int width;
	float value;

	UART_Params_init(&uart7Params);
	uart7Params.writeDataMode = UART_DATA_BINARY;
	uart7Params.readDataMode = UART_DATA_BINARY;
	uart7Params.readReturnMode = UART_RETURN_FULL;
	uart7Params.readEcho = UART_ECHO_OFF;
	uart7Params.baudRate = 9600;
	uart7 = UART_open(Board_UART3, &uart7Params);

	if (uart7 == NULL) {
		System_abort("Error opening the UART");
	}

	while (true) {
		firedEvents = Event_pend(transferEvent, Event_Id_NONE,
				TRANSFER_MESSAGE_EVENT,
				BIOS_WAIT_FOREVER);
		if (firedEvents & TRANSFER_MESSAGE_EVENT) {
			// Get the posted message.
			// Mailbox_pend() will not block since Event_pend()
			// has guaranteed that a message is available.
			Mailbox_pend(transferMailbox, &message, BIOS_NO_WAIT);
			switch (message.kind) {
			case TRANSFER_TEMPERATURE:
				result[0] = ID_TEMPERATURE;
				width = 1;
				precision = TEMPERATURE_PRECISION;
				if (TEMPERATURE_PRECISION > 0)
				{
					width = 3;
				}
				value = message.value;
				break;
			case TRANSFER_PRESSURE:
				result[0] = ID_PRESSURE;
                value = message.value / 100 ; /* hPa */
				width = 1;
				precision = PRESSURE_PRECISION;
				if (PRESSURE_PRECISION > 0)
				{
					width = 3;
				}
				break;
			case TRANSFER_ALTITUDE:
				result[0] = ID_ALTITUDE;
				value = message.value;
				precision = ALTITUDE_PRECISION;
				if (ALTITUDE_PRECISION > 0)
				{
					width = 3;
				}
				break;
			default:
				System_printf("Error TransferFunction: Received unknown message %d.\n",
						message.kind);
				System_flush();
				// unknown, nothing special
				continue; /* no break, would be unreachable code */
			}
			(void)snprintf (&result[1], 20, "%*.*f", width, precision, value);
			length = strlen(result) + 1;
			UART_write(uart7, &result[0], length);
#ifdef DEBUG
			System_printf("%s transferred.\n", result);
			System_flush();
#endif // DEBUG
		}
	}
}

