/*
 * Altitude.h
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */

#ifndef ALTITUDE_H_
#define ALTITUDE_H_


// I2C bus address of Altitude Click
#define BOARD_ALTIUDE_CLICK 0x60

// MPL3115A2 register definitions
#define MPL3115A2_DR_STATUS   0x00    // Status register
#define MPL3115A2_OUT_P_MSB   0x01    // Pressure/Altitude data (MSB)
#define MPL3115A2_OUT_P_CSB   0x02    // Pressure/Altitude data (middle)
#define MPL3115A2_OUT_P_LSB   0x03    // Pressure/Altitude data (LSB)
#define MPL3115A2_OUT_T_MSB   0x04    // Temperature data (MSB)
#define MPL3115A2_OUT_T_LSB   0x05    // Temperature data (LSB)
#define MPL3115A2_INT_SOURCE  0x12    // Int source register
#define MPL3115A2_PT_DATA_CFG 0x13    // Event flag
#define MPL3115A2_BAR_IN_MSB  0x14    // Barometric input for Altitude calculation
#define MPL3115A2_BAR_IN_LSB  0x15    // Barometric input for Altitude calculation
#define MPL3115A2_P_TGT_LSB   0x16    // Atlitude mode: target LSB in meter
#define MPL3115A2_P_TGT_MSB   0x17    // Atlitude mode: target MSB in meter

#define MPL3115A2_T_TGT       0x18    // temperature target LSB

#define MPL3115A2_P_WND_LSB   0x19    // Atlitude mode: window LSB in meter
#define MPL3115A2_P_WND_MSB   0x1A    // Atlitude mode: window MSB in meter
#define MPL3115A2_T_WND   0x1B    // temperature window LSB


#define MPL3115A2_CTRL_REG1   0x26    // Control register 1
#define MPL3115A2_CTRL_REG2   0x27    // Control register 2
#define MPL3115A2_CTRL_REG3   0x28    // pin configuration for interrupt
#define MPL3115A2_CTRL_REG4   0x29    // enable interrupt


// Start height in meter, FH Technikum Wien see http://de.mygeoposition.com/
#define START_ALTITUDE 164

// Alarm height in meter
#define ALARM_ALTITUDE 162
#define ALARM_WINDOW_ALTITUDE 1  // in meter
// Alarm temperature in °C
#define ALARM_TEMPERATURE 20

// bit numbers
//control register 1
#define BIT_STANDBY 0
#define BIT_ONE_SHOT 1
#define BIT_ALTIMETER  7

//interupt status register
#define BIT_SOURCE_TTH 4 // temperature threshold reached
#define BIT_SOURCE_PTH 8 // Pressure threshold reached
#define BIT_SOURCE_PTW 0x20 // Pressure threshold reached

// status register
#define BIT_PRESSURE_DATA 4 // pressure data ready

// control register 4
#define BIT_INTERRUPT_TEMP 4  // enable temperature interrupt
#define BIT_INTERRUPT_ALTITUDE 8 // enable altitude interrupt
#define BIT_INTERRUPT_WINDOW_ALTITUDE 0x20 // enable altitude window interrupt


// pressure/altittude raw value
typedef struct ReadDataStruct
{
	uint8_t low_byte;
	uint8_t middle_byte;
	uint8_t high_byte;
} ReadDataType;


#endif /* ALTITUDE_H_ */
