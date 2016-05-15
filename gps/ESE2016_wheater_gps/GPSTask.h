/*
 * GPSTask.h
 *
 *  Created on: 15.05.2016
 *      Author: amaierhofer
 */

#ifndef GPSTASK_H_
#define GPSTASK_H_

typedef struct DateTimeStruct {
	char dateTimeString[15]; // "YYYYMMDDHHMMSS"
} DateTimeType;

typedef struct PositionStruct {

	char latitude[20];  // "N48° 16.1245"
	char longitude[20]; // "E016° 14.1245"
} PositionType;

typedef struct AltiudeStruct {

	char altitude[10]; // in meter
} AltitudeType;



#endif /* GPSTASK_H_ */
