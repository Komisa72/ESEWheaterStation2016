/*
 * ClockTask.c
 *
 *  Created on: 05.01.2016
 *      Author: amaierhofer
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "ClockTask.h"
#include "BoosterPack.h"

/* defines */

/* global */
Event_Handle measureThermoEvent; // trigger measurement of thermo click
Event_Handle measureAltitudeEvent; // trigger measurement of altitude click
Event_Handle transferEvent; // trigger transfer of read data
Mailbox_Handle transferMailbox; // contains data to be transferred

/**
 * /fn ClockFunction
 * /brief Task function called on every clock tick.
 *
 * /param arg0 not used.
 */
static void ClockFunction(UArg arg0)
{
	UInt eventId;
#if USE_THERMO_CLICK
    eventId = MEASURE_THERMO_EVENT;
	Event_post(measureThermoEvent, eventId);
#endif
#if USE_ALTITUDE_CLICK
    eventId = MEASURE_ALTITUDE_EVENT;
	Event_post(measureAltitudeEvent, eventId);
#endif

}

/**
 *  /fn setupClockTask
 *  /brief Setup clock task
 *
 *  Task has highest priority and receives 1kB of stack
 *
 *  /param time to wait for new measurement of temperature, pressure etc.
 *
 *  /return always zero. In case of error the system halts.
 */
int SetupClockTask(uint32_t wait_ticks)
{
	Mailbox_Params mailboxParams;
	Clock_Params clockParams;
	Clock_Handle myClock;
	Error_Block eb;

	Error_init(&eb);

	measureThermoEvent = Event_create(NULL, &eb);
	if (measureThermoEvent == NULL)
	{
		System_abort("Measure thermo event create failed");
	}
	measureAltitudeEvent = Event_create(NULL, &eb);
	if (measureAltitudeEvent == NULL)
	{
		System_abort("Measure altitude event create failed");
	}

	transferEvent = Event_create(NULL, &eb);
	if (transferEvent == NULL) {
		System_abort("Transfer event create failed");
	}
	Mailbox_Params_init(&mailboxParams);
	mailboxParams.readerEvent = transferEvent;
	// Assign TRANSFER_MESSAGE_EVENT to Mailbox "not empty" event
	mailboxParams.readerEventId = TRANSFER_MESSAGE_EVENT;
	transferMailbox = Mailbox_create(sizeof(TransferMessageType),
			TRANSFER_MAILBOX_SIZE, &mailboxParams, &eb);
	if (transferMailbox == NULL) {
		System_abort("Transfer mailbox create failed");
	}

	Clock_Params_init(&clockParams);
	if (wait_ticks < MINIMUM_SAMPLING_TIME)
	{
		wait_ticks = MINIMUM_SAMPLING_TIME;
	}

	clockParams.period = wait_ticks;
	clockParams.startFlag = TRUE;
	clockParams.arg = NULL;

	myClock = Clock_create((Clock_FuncPtr)ClockFunction, wait_ticks, &clockParams, &eb);
	if (myClock == NULL)
	{
		System_abort("Clock create failed");
	}
	return 0;
}
