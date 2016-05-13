/*
 * ClockTask.h
 *
 *  Created on: 05.01.2016
 *      Author: amaierhofer
 */

#ifndef CLOCKTASK_H_
#define CLOCKTASK_H_

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>

/* typedef */

/**
 * /brief Which kind of data will be transferred via mailbox.
 */
typedef enum TransferMessageKindEnum {
	TRANSFER_TEMPERATURE,
	TRANSFER_PRESSURE,
	TRANSFER_ALTITUDE,
	TRANSFER_HEIGHT_ALARM,
} TransferMessageKindType;

/**
 * /brief Mailbox entry structure for measured values.
 */
typedef struct TransferMessageStruct {
	TransferMessageKindType kind;
	float value;      // temperature in °C, pressure in Pa, altitude m
} TransferMessageType;

/* defines */
// event ids
#define MEASURE_THERMO_EVENT Event_Id_00
#define MEASURE_ALTITUDE_EVENT Event_Id_00

#define TRANSFER_MESSAGE_EVENT Event_Id_00
#define TRANSFER_MAILBOX_SIZE 10 // max. 10 entries in mailbox

// precisions for measured values.
#define TEMPERATURE_PRECISION 2
#define ALTITUDE_PRECISION 3
#define PRESSURE_PRECISION 1

// identifiers for transferred data over uart followed by string (converted float)
#define ID_TEMPERATURE 'T'
#define ID_ALTITUDE 'A'
#define ID_PRESSURE 'P'

/* minimum time between triggering a new measurement in 1/1000 s*/
#define MINIMUM_SAMPLING_TIME  1000

/* global */
extern Event_Handle measureAltitudeEvent; // trigger measurement of altitude click
extern Event_Handle measureThermoEvent; // trigger measurement of thermo click
extern Event_Handle transferEvent; // trigger transfer of read data
extern Mailbox_Handle transferMailbox; // contains data to be transferred

/**
 * /fn setupClockTask
 * /brief Setup clock task
 *
 *  Setup clock task
 *  Task has highest priority and receives 1kB of stack
 *
 *  /param time to wait for new measurement of temperature, pressure etc.
 *
 *  /return always zero. In case of error the system halts.
 */
int SetupClockTask(uint32_t wait_ticks);

#endif /* CLOCKTASK_H_ */
