/*
 * GPSTask.c
 *
 *  Created on: 14.05.2016
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

#include <Board.h>
#include <string.h>

#include "BoosterPack.h"
#include "GPSTask.h"
#include "ClockTask.h"

static char gpsString[82]; // NMEA sentence max. 80 characters + CR + LF

// * + 2 bytes checksum + CR + LF
#define TRAILING_CHARACTERS 5
// ringbuffer size
#define SIZE_TRANSFER_ARRAY 10

// ringbuffer for transferring position values
static PositionType transferPosition[SIZE_TRANSFER_ARRAY];
// ringbuffer for transferring altitude values
static PositionType transferAltitude[SIZE_TRANSFER_ARRAY];
// ringbuffer for transferring date time values
static DateTimeType transferDateTime[SIZE_TRANSFER_ARRAY];

// forward references
static void ParseData();

// sent only new data if date/time differs
static Int32 hhmmssLast = -1;
static Int32 ddmmyyLast = -1;

// functions

/*
 * /fn gpsFunction Task function for reading gps data.
 * /param arg0 always NULL
 * /param arg0 always NULL
 */
static void GpsFunction(UArg arg0, UArg arg1) {
	UART_Handle uart6;
	UART_Params uart6Params;
	int length = 1;
	UInt8 read;
	int index = 0;
	bool start = false;
	bool full = false;
	int waitChecksum = 0;
	int tranferPositionIndex = -1;
	int transferAltitudeIndex = -1;

	// gps click is on booster pack 2

	UART_Params_init(&uart6Params);
	uart6Params.writeDataMode = UART_DATA_BINARY;
	uart6Params.readDataMode = UART_DATA_BINARY;
	uart6Params.readReturnMode = UART_RETURN_FULL;
	uart6Params.readEcho = UART_ECHO_OFF;
	uart6Params.baudRate = 9600;
	uart6 = UART_open(Board_UART2, &uart6Params);

	if (uart6 == NULL) {
		System_abort("Error opening the UART6");
	}

	while (true) {
		UART_read(uart6, &read, length);
		if (read == '$') {
			// start of nmea string
			index = 0;
			start = true;
		}
		if (start) {
			gpsString[index] = read;
			if (read == '*') {
				waitChecksum = TRAILING_CHARACTERS;
			}
			if (waitChecksum > 0) {
				--waitChecksum;
				if (waitChecksum == 0) {
					// make 0 string
					gpsString[index] = '\0';
					full = true;
				}
			}
			if (full) {
				full = false;
				ParseData(&tranferPositionIndex, &transferAltitudeIndex);
#ifdef DEBUG
				start = false;
#endif
			}
			++index;
		}
	}
}

/*
 * /fn parseData parse data read out of gps3 click
 *
 * /param transferPositionIndex where to write in the ringbuffer of positions
 * /param transferAltitudeIndex where to write in the ringbuffer of altitudes
 */
static void ParseData(int* transferPositionIndex, int* transferAltitudeIndex) {
	char* timeStart;
	char* timeEnd;
	char* latitudeStart;
	char* latitudeEnd;
	char* latitudeNStart;
	char* latitudeNEnd;
	char* longitudeStart;
	char* longitudeEnd;
	char* longitudeWStart;
	char* longitudeWEnd;
	char* warningA;
	char* warningAEnd;
	char* dummy;
	char* dateStart;
	char* dateEnd;
	int indexPos;
	int count;
	TransferMessageType gpsTransfer;
	TransferMessageType dateTimeTransfer;
	Int32 hhmmss;
	Int32 ddmmyy;
	int i;
	int faktor;
	int k;
	bool transfer;

	if (strstr(gpsString, "GPRMC") != NULL) {
		timeStart = &gpsString[7];
		timeEnd = strchr(timeStart, ',');
		warningA = timeEnd + 1;
		warningAEnd = strchr(warningA, ',');
		if (*warningA == 'A') {
			// valid string
			latitudeStart = warningAEnd + 1;
			latitudeEnd = strchr(latitudeStart, ',');
			latitudeNStart = latitudeEnd + 1;
			latitudeNEnd = strchr(latitudeNStart, ',');

			longitudeStart = latitudeNEnd + 1;
			longitudeEnd = strchr(longitudeStart, ',');
			longitudeWStart = longitudeEnd + 1;
			longitudeWEnd = strchr(longitudeWStart, ',');

			dummy = longitudeWEnd + 1;
			dummy = strchr(dummy, ',');
			dummy = dummy + 1;
			dummy = strchr(dummy, ',');
			dateStart = dummy + 1;
			dateEnd = strchr(dateStart, ',');

			faktor = 10000;
			hhmmss = 0;
			for (i = 0; i < 6; ++i) {
				hhmmss += (timeStart[i] - '0') * faktor;
				faktor /= 10;
			}
			faktor = 10000;
			ddmmyy = 0;
			for (i = 0; i < 6; ++i) {
				ddmmyy += (dateStart[i] - '0') * faktor;
				faktor /= 10;
			}
			transfer = (hhmmss != hhmmssLast) || (ddmmyy != ddmmyyLast);

			if (transfer) {
				hhmmssLast = hhmmss;
				ddmmyyLast = ddmmyy;

				// via indexPos realize a ringbuffer for storing the data
				indexPos = *transferPositionIndex;
				indexPos = (indexPos + 1) % SIZE_TRANSFER_ARRAY;
				*transferPositionIndex = indexPos;

				for (i = 0; i < 6; ++i) {
					transferDateTime[indexPos].dateTimeString[8 + i] =
							timeStart[i];
					k++;
				}
				transferDateTime[indexPos].dateTimeString[0] = '2';
				transferDateTime[indexPos].dateTimeString[1] = '0';

				k = 4;
				for (i = 0; i < 6; ++i) {
					transferDateTime[indexPos].dateTimeString[2 + i] =
							dateStart[k];
					++k;
					if (k == 6) {
						k = 2;
					} else if (k == 4) {
						k = 0;
					}
				}
				transferDateTime[indexPos].dateTimeString[14] = '\0';

				transferPosition[indexPos].latitude[0] = *latitudeNStart;
				memcpy(&transferPosition[indexPos].latitude[1], latitudeStart,
						2);
				latitudeStart += 2;
				transferPosition[indexPos].latitude[3] = '°';
				count = latitudeEnd - latitudeStart;
				memcpy(&transferPosition[indexPos].latitude[4], latitudeStart,
						count);
				transferPosition[indexPos].latitude[4 + count] = '\0';
				transferPosition[indexPos].longitude[0] = *longitudeWStart;

				memcpy(&transferPosition[indexPos].longitude[1], longitudeStart,
						3);
				longitudeStart += 3;
				transferPosition[indexPos].longitude[4] = '°';
				count = longitudeEnd - longitudeStart;
				memcpy(&transferPosition[indexPos].longitude[5], longitudeStart,
						count);
				transferPosition[indexPos].longitude[5 + count] = '\0';

				// let the message consumer use a pointer into my ringbuffer of date/time
				dateTimeTransfer.data = &transferDateTime[indexPos];
				dateTimeTransfer.kind = TRANSFER_DATE_TIME;
				/* implicitly posts TRANSFER_MESSAGE_EVENT to transferEvent */
				Mailbox_post(transferMailbox, &dateTimeTransfer,
						BIOS_WAIT_FOREVER);

				// let the message consumer use a pointer into my ringbuffer of positions
				gpsTransfer.data = &transferPosition[indexPos];
				gpsTransfer.kind = TRANSFER_GPS_LOCATION;
				/* implicitly posts TRANSFER_MESSAGE_EVENT to transferEvent */
				Mailbox_post(transferMailbox, &gpsTransfer, BIOS_WAIT_FOREVER);
			}
		}
	} else if (strstr(gpsString, "GPGGA") != NULL) {
		// get altitude
		gpsString[0] = '$';
	}

}

/*
 *  /fn setupGpsTask setup task function
 *  /return always 0 in case of error a system abort occurs
 */
int SetupGpsTask() {
	/* Setup Task and create it */
	Task_Params taskParams;
	Task_Handle taskHandle;
	Error_Block eb;

	Error_init(&eb);
	Task_Params_init(&taskParams);
	taskParams.stackSize = 2048;
	taskParams.arg0 = NULL;
	taskParams.arg1 = NULL;
	taskParams.priority = 14;
	taskHandle = Task_create((Task_FuncPtr) GpsFunction, &taskParams, &eb);
	if (taskHandle == NULL) {
		System_abort("Error creating gps task");
	}
	return 0;
}

