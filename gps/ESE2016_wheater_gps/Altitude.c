/*
 * Altitude.c
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */
#include <math.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
//#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/hal/hwi.h>
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/

/* Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

#include "BoosterPack.h"
#include "Altitude.h"
#include "ClockTask.h"

/* Defines */
#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))
#define SetBit(reg, bit)         reg |= (1<<(bit))
#define INTERRUPT_EVENT Event_Id_00

/* globals */

/* static */
static I2C_Handle i2c;
static Event_Handle interruptEvent; // got interrupt, read i2C in task context

// Fractional bit values for altitude
static const float ALT_FRAC_B7 = 0.5;    // Fractional value for bit 7
static const float ALT_FRAC_B6 = 0.25;   // Fractional value for bit 6
static const float ALT_FRAC_B5 = 0.125;  // Fractional value for bit 5
static const float ALT_FRAC_B4 = 0.0625; // Fractional value for bit 4

// Fractional bit values for pressure
static const float PRES_FRAC_B5 = 0.5;   // Fractional value for bit 5
static const float PRES_FRAC_B4 = 0.25;  // Fractional value for bit 4

/* Forward references */
void ReadData(I2C_Handle i2c, ReadDataType* pread);
float PressureRead(I2C_Handle i2c);
void SwitchToStandby(I2C_Handle i2c);
void SetOSR(I2C_Handle i2c, unsigned char value);

/**
 * /fn AlarmFunction
 * /brief Interrupt function of altitude click.
 * /return void.
 */
static void AlarmFunction(UArg arg) {
	UInt eventId;

	GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
	// can not write with i2c driver to clear interrupt of altitude click
	// in interrupt context

	// turn on led 1
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
	// task read out the data
	eventId = INTERRUPT_EVENT;
	Event_post(interruptEvent, eventId);

}

/**
 * /fn MPL3115A2_Write
 * /brief Write 1 byte to the given address.
 * /param i2c handle to open I2C connection.
 * /param register address where to write.
 * /param value to be written.
 * /return void.
 */
void MPL3115A2_Write(I2C_Handle i2c, uint8_t address, uint8_t value) {
	I2C_Transaction i2cTransaction;
	uint8_t txBuffer[2];

	i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
	i2cTransaction.writeBuf = txBuffer;
	i2cTransaction.writeCount = 2;
	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.arg = 0;
	txBuffer[0] = address;
	txBuffer[1] = value;

	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
		System_flush();
	}

}

/*
 * /fn MPL3115A2_Read
 * /brief Read 1 byte from the given address.
 * /param i2c handle to open I2C connection.
 * /param register address where to read.
 * /return read value.
 */
uint8_t MPL3115A2_Read(I2C_Handle i2c, uint8_t address) {
	I2C_Transaction i2cTransaction;
	uint8_t txBuffer[2];
	uint8_t rxBuffer[2];

	i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
	i2cTransaction.writeBuf = txBuffer;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = rxBuffer;
	i2cTransaction.readCount = 1;
	i2cTransaction.arg = 0;
	txBuffer[0] = address;

	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
		System_flush();
	}
	return rxBuffer[0];
}

/**
 * /fn SwitchToStandby
 * /brief switches device to Standby mode for making changes to control registers.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void SwitchToStandby(I2C_Handle i2c) {
	uint8_t controlRegister1;
	controlRegister1 = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	ClearBit(controlRegister1, BIT_STANDBY); // reset Standby bit (switch to Standby mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
}

/**
 * /fn SwitchToAcitve
 * /brief Switch device to active mode.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void SwitchToActive(I2C_Handle i2c)
{
	uint8_t controlRegister1;
	controlRegister1 = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	SetBit(controlRegister1, BIT_STANDBY); // set Standby bit (switch to active mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
}

/**
 * /fn SwitchToAltimeter
 * /brief Switch device to altimeter mode.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void SwitchToAltimeter(I2C_Handle i2c)
{
	uint8_t controlRegister1;
	controlRegister1 = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	SetBit(controlRegister1, BIT_ALTIMETER);
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
}

/**
 * /fn SwitchToBarometer
 * /brief Switch device to barometer mode.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void SwitchToBarometer(I2C_Handle i2c)
{
	unsigned short controlRegister1;
	controlRegister1 = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	ClearBit(controlRegister1, BIT_ALTIMETER);
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
}

/**
 * /fn OneShotMeasurement
 * /brief Trigger one shot measurement.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void OneShotMeasurement(I2C_Handle i2c) {
	uint8_t controlRegister1;

	controlRegister1 = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	SetBit(controlRegister1, BIT_ONE_SHOT);
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
	ClearBit(controlRegister1, BIT_ONE_SHOT);
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, controlRegister1);
}

/**
 * /fn MPL3115A2Calibrate
 * /brief Calibrate the altimeter.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void MPL3115A2Calibrate(I2C_Handle i2c) {
	float current_pressure;
	float calibration_pressure;
	float sea_pressure;
	int i;
	int calculate;
	int value = 0;

	// Clear value
	calibration_pressure = 0;
	SetOSR(i2c,7);

	// Calculate current pressure level
	for (i = 0; i < 8; i++) {
		OneShotMeasurement(i2c);
		Task_sleep(550);      // Wait for sensor to read pressure
		while ((value & BIT_PRESSURE_DATA) != BIT_PRESSURE_DATA)
		{
			value = MPL3115A2_Read(i2c, MPL3115A2_DR_STATUS);

		}
		calibration_pressure = calibration_pressure + PressureRead(i2c);
		value = 0;
	}
	// Find average value of current pressure level readings
	current_pressure = calibration_pressure / 8;

	// Calculate barometric pressure at mean sea level based on a starting altitude
	// (see International Standard Atmosphere Temperature 15 °C = 288,15 K,
	// barometric pressure 1013,25 hPa, Temperature gradient 0,65 K per 100 m */
	sea_pressure = current_pressure
			/ pow(1 - START_ALTITUDE * 0.0000225577, 5.255877);

	// Calibrate the sensor according to the sea level pressure for the
	// current measurement location (2 Pa per LSB) :
	calculate = (int) sea_pressure;
	if (sea_pressure > 0) {
		MPL3115A2_Write(i2c, MPL3115A2_BAR_IN_MSB,
				(uint8_t) ((calculate / 2) >> 8));
		MPL3115A2_Write(i2c, MPL3115A2_BAR_IN_LSB,
				(uint8_t) ((calculate / 2) & 0xFF));
	}
	System_printf("Calibration finished.\n");
}

/**
 * /brief Set Output Sample Rate
 * The OSR_Value is set to 0 - 7 corresponding with Ratios 1 - 128
 * /param i2c handle to open I2C connection.
 * /param value 0 to 7.
 * /return void.
 */
void SetOSR(I2C_Handle i2c, unsigned char value) {
	uint8_t control_register;
	uint8_t mask = ~(7 << 3);

	SwitchToStandby(i2c);
	control_register = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);
	if (value < 8) {
		value <<= 3;
		control_register &= mask;
		MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, control_register | value);
	}
}

/**
 * /fn MPL3115A2Init
 * /brief Common initialisation.
 * /param i2c handle to open I2C connection.
 * /return void.
 */
void MPL3115A2Init(I2C_Handle i2c) {
	SetOSR(i2c, 7);  // oversampling 128/ 512ms
	// enable pressure/temperature data ready flags/interrupt
	MPL3115A2_Write(i2c, MPL3115A2_PT_DATA_CFG, 0x7);
}

/**
 * /fn ReadData
 * /brief Read the pressure/altimeter value.
 * /param i2c handle to open I2C connection.
 * /param pread pointer where to store the read values.
 * /return void.
 */
void ReadData(I2C_Handle i2c, ReadDataType* pread) {
	uint8_t lowTemp;  // read temperature also to reset interrupt just in case
	uint8_t highTemp;

	pread->low_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_LSB);
	pread->middle_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_CSB);
	pread->high_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_MSB);

	// whole degrees are in register out_t_msb
	highTemp = MPL3115A2_Read(i2c, MPL3115A2_OUT_T_MSB);
	lowTemp = MPL3115A2_Read(i2c, MPL3115A2_OUT_T_LSB);

	System_printf("X%d\n", highTemp);

}

/**
 * /fn PressureRead
 * /brief Read barometric pressure value.
 * /param i2c handle to open I2C connection.
 * /return Pressure in pascal.
 */
float PressureRead(I2C_Handle i2c) {
	ReadDataType read;
	unsigned long pressure_int;
	float pressure_frac_value = 0;

	ReadData(i2c, &read);

	// Calculate the integer part of the barometric pressure value
	pressure_int = read.low_byte | ((read.middle_byte << 8))
			| ((unsigned long) read.high_byte << 16);
	pressure_int = pressure_int >> 6;

	// Calculate the fractional part of the barometric pressure value
	if ((read.low_byte & 0x20) == 0x20) {
		pressure_frac_value += PRES_FRAC_B5;
	}
	if ((read.low_byte & 0x10) == 0x10) {
		pressure_frac_value += PRES_FRAC_B4;
	}
	// Sum the integer part and fractional part of the pressure value
	return (float) pressure_int + pressure_frac_value;
}

/**
 * /AltitudeRead
 * /brief Read out the altitude value from registers in meter.
 * /param i2c handle to open I2C connection.
 * /return Altitude in meter.
 */
float AltitudeRead(I2C_Handle i2c) {
	ReadDataType read;
	float altitude_value;
	float altitude_frac_value = 0;

	ReadData(i2c, &read);

	// Calculate the integer part of the altitude value
	altitude_value = (short) ((read.middle_byte
			| ((unsigned short) read.high_byte << 8)));

	// Calculate the fractional part of the altitude value
	if ((read.low_byte & 0x80) == 0x80) {
		altitude_frac_value += ALT_FRAC_B7;
	}
	if ((read.low_byte & 0x40) == 0x40) {
		altitude_frac_value += ALT_FRAC_B6;
	}
	if ((read.low_byte & 0x20) == 0x20) {
		altitude_frac_value += ALT_FRAC_B5;
	}
	if ((read.low_byte & 0x10) == 0x10) {
		altitude_frac_value += ALT_FRAC_B4;
	}
	// Sum the integer part and fractional part of the altitude value
	return altitude_value = altitude_value + altitude_frac_value;
}

/**
 * /fn InterruptFunction
 * /brief Task runs when interrupt has occured.
 *
 * I2C can not be read in interrupt with the used I2C driver.
 *
 * /param arg0 not used.
 * /param arg1 not used.
 * /return void.
 */
void InterruptFunction(UArg arg0, UArg arg1) {
	uint8_t value;

	while (true) {
		// trigger measurement only if event is set
		Event_pend(interruptEvent, Event_Id_NONE, MEASURE_ALTITUDE_EVENT,
				BIOS_WAIT_FOREVER);
		 value = MPL3115A2_Read(i2c, MPL3115A2_INT_SOURCE);

		if ((value & BIT_SOURCE_TTH) == BIT_SOURCE_TTH)
		{
			System_printf("Temperature Alarm!\n");
			System_flush();
		}

		if ((value & (BIT_SOURCE_PTH | BIT_SOURCE_PTW)) > 0)
		{
			System_printf("Pressure Alarm!\n");
			System_flush();
		}
	}

}

/**
 * /fn AltitudeFunction
 * /brief Task function of altitude click.
 * /param arg0 is BoosterPackType to identify which booster pack is used.
 * /param arg1 always NULL.
 */
void AltitudeFunction(UArg arg0, UArg arg1) {
	I2C_Params i2cParams;
	unsigned int index;
	//I2C_Handle i2c;
	float altitude;
	float pressure;
	TransferMessageType altimeter;
	TransferMessageType barometer;
	uint16_t targetAltitude;
	int8_t temp;
	uint8_t status = 0;
	uint16_t windowAltitude;

	altimeter.kind = TRANSFER_ALTITUDE;
	barometer.kind = TRANSFER_PRESSURE;

	/* Create I2C for usage */
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_100kHz;
	i2cParams.transferMode = I2C_MODE_BLOCKING;
	i2cParams.transferCallbackFxn = NULL;
	if ((BoosterPackType) arg0 == BOOSTER_PACK_1) {
		index = Board_I2C0;
	} else {
		// BOOSTER_PACK_2
		index = Board_I2C1;
	}
	i2c = I2C_open(index, &i2cParams);
	if (i2c == NULL) {
		System_abort("Error Initializing I2C\n");
	} else {
		System_printf("I2C Initialized!\n");
	}
#if USE_ALTITUDE_CLICK
	MPL3115A2Init(i2c);
	MPL3115A2Calibrate(i2c);
	SetOSR(i2c, 7);
#endif

	// init interrupt
	SwitchToStandby(i2c);
	// setup both interrupts active high
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG3, 0x22);
	SwitchToAltimeter(i2c);
	targetAltitude = ALARM_ALTITUDE;
	MPL3115A2_Write(i2c, MPL3115A2_P_TGT_LSB, targetAltitude & 0xFF);
	MPL3115A2_Write(i2c, MPL3115A2_P_TGT_MSB, (targetAltitude >> 8));

	// set altitude window size
	windowAltitude = ALARM_WINDOW_ALTITUDE;
	MPL3115A2_Write(i2c, MPL3115A2_P_WND_LSB, windowAltitude & 0xFF);
	MPL3115A2_Write(i2c, MPL3115A2_P_WND_MSB, (windowAltitude >> 8));

	temp = ALARM_TEMPERATURE;
	MPL3115A2_Write(i2c, MPL3115A2_T_TGT, temp);
	// enable temperature threshold interrupt
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG4, BIT_INTERRUPT_TEMP);

	// INT2 of Altitude click
	// pin int2 of altitude click goes to port pin2
	GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);
	GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_INT_PIN_2, GPIO_RISING_EDGE);
	Hwi_enableInterrupt(INT_GPIOH_TM4C129);
	GPIOIntEnable(GPIO_PORTH_BASE, GPIO_INT_PIN_2);

	SwitchToActive(i2c);
	while (true) {
		// trigger measurement only if event is set
		Event_pend(measureAltitudeEvent, Event_Id_NONE, MEASURE_ALTITUDE_EVENT,
				BIOS_WAIT_FOREVER);

 		SwitchToStandby(i2c);
		MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG4, BIT_INTERRUPT_TEMP);
		SwitchToBarometer(i2c);
		SwitchToActive(i2c);
		while ((status & BIT_PRESSURE_DATA) != BIT_PRESSURE_DATA)
		{
			Task_sleep(100);
			status = MPL3115A2_Read(i2c, MPL3115A2_DR_STATUS);
		}
		status = 0;
		pressure = PressureRead(i2c);
		barometer.value = pressure;

		SwitchToStandby(i2c);
		SwitchToAltimeter(i2c);
		OneShotMeasurement(i2c);
		// enable temperature / altitude (pressure) threshold interrupt
		MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG4, BIT_INTERRUPT_TEMP |
				BIT_INTERRUPT_ALTITUDE | BIT_INTERRUPT_WINDOW_ALTITUDE);

		SwitchToActive(i2c);
		while ((status & BIT_PRESSURE_DATA) != BIT_PRESSURE_DATA)
		{
			Task_sleep(100);
			status = MPL3115A2_Read(i2c, MPL3115A2_DR_STATUS);
		}
		status = 0;
		altitude = AltitudeRead(i2c);
		altimeter.value = altitude;

		/* implicitly posts TRANSFER_MESSAGE_EVENT to transferEvent */
		Mailbox_post(transferMailbox, &altimeter, BIOS_WAIT_FOREVER);
		Mailbox_post(transferMailbox, &barometer, BIOS_WAIT_FOREVER);

		// do all the prints from ReadData
		System_flush();

	}

#ifdef DEBUG
	/* Deinitialized I2C */
	I2C_close(i2c);
	System_printf("I2C closed!\n");
	System_flush();
#endif

}

/**
 * /fn SetupAltiudeTask
 * /brief Initialise altitude task and start it.
 * /param boosterPack which booster pack is used for altitude click.
 * /return always 0.
 */
int SetupAltiudeTask(BoosterPackType boosterPack) {
	Task_Params taskAltitudeParams;
	Task_Handle taskAltitude;
	Hwi_Params hwiParams;
	Hwi_Handle altitudeHwi;
	Error_Block eb;
	Task_Params taskInterruptParams;
	Task_Handle taskInterrupt;

	Error_init(&eb);
	Task_Params_init(&taskAltitudeParams);
	taskAltitudeParams.stackSize = 1536;
	taskAltitudeParams.priority = 14;
	taskAltitudeParams.arg0 = (UArg) boosterPack;
	taskAltitudeParams.arg1 = NULL;
	taskAltitude = Task_create((Task_FuncPtr) AltitudeFunction,
			&taskAltitudeParams, &eb);
	if (taskAltitude == NULL) {
		System_abort("Task altitude create failed");
	}


	interruptEvent = Event_create(NULL, &eb);
	if (interruptEvent == NULL) {
		System_abort("Interrupt event create failed");
	}


	Error_init(&eb);
	Task_Params_init(&taskInterruptParams);
	taskInterruptParams.stackSize = 1024;
	taskInterruptParams.priority = 15;
	taskInterruptParams.arg0 = NULL;
	taskInterruptParams.arg1 = NULL;
	taskInterrupt = Task_create((Task_FuncPtr) InterruptFunction,
			&taskAltitudeParams, &eb);
	if (taskInterrupt == NULL) {
		System_abort("Task interrupt create failed.");
	}

	Error_init(&eb);
	Hwi_Params_init(&hwiParams);
	// Set AlarmInterrupt parameters
	hwiParams.arg = 0;
	hwiParams.enableInt = false;

	altitudeHwi = Hwi_create(INT_GPIOH_TM4C129, AlarmFunction, &hwiParams, &eb);
	if (altitudeHwi == NULL) {
		System_abort("Altitude click hardware interrupt create failed.");
	}

	return 0;
}

