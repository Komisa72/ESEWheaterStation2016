/*
 * BoosterPack.h
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */

#ifndef BOOSTERPACK_H_
#define BOOSTERPACK_H_

// compile time switch to turn on / off used hardware
#define USE_THERMO_CLICK   0
//#define USE_THERMO_CLICK   1

// compile time switch to turn on / off used hardware
//#define USE_ALTITUDE_CLICK   0
#define USE_ALTITUDE_CLICK   1

/**
 * /brief Determine which booster pack is used.
 */
typedef enum BoosterPackEnum
{
	BOOSTER_PACK_1,
	BOOSTER_PACK_2

} BoosterPackType;


#endif /* BOOSTERPACK_H_ */
