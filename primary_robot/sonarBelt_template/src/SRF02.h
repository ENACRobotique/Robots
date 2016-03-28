/*
 * SRF02.h
 *
 *  Created on: Mar 20, 2016
 *      Author: yoyo
 */

#ifndef SRF02_H_
#define SRF02_H_

#include <millis.h>
#include <utility> // pair
#include <cstdint>

#define DBG_SRF02

#ifdef DBG_SRF02
#include <stdio.h>
#endif

#define SRF02_APERT_ANGLE 30  // Aperture angle of the detection cone in degrees
#define SRF02_MEAS_PERIOD 65  // Amount of time in milliseconds to process the measures

// Registers
#define REG_CMD 0
#define REG_SOFT_REV 0
#define REG_RANGE_H 2
#define REG_RANGE_L 3
#define REG_AUTOTUNE_MIN_H 4
#define REG_AUTOTUNE_MIN_L 5


//// Commands
// Ranging modes
#define CMD_MES_INCH (0x50) // in inches
#define CMD_MES_CM (0x51)  // in centimeters
#define CMD_MES_MS (0x52)  // in milliseconds
// Fake ranging modes
#define CMD_FAKE_MES_INCH (0x56) // in inches
#define CMD_FAKE_MES_CM (0x57)  // in centimeters
#define CMD_FAKE_MES_MS (0x58)  // in milliseconds
// Transmit a burst without ranging
#define CMD_BURST (0x5C)
// Force autotune - same as power up
#define CMD_AUTOTUNE (0x60)
// Change I2C address
#define CMD_CHG_ADD_1 (0xA0)
#define CMD_CHG_ADD_2 (0xA5)
#define CMD_CHG_ADD_3 (0xAA)


typedef std::pair<int, int> PoseSonar_t;

typedef enum eSRF02_Info{
	srf02_range,
	srf02_version,
	srf02_autotuneMin,
	eSRF02_TypeInfoMax
}eSRF02_Info;

typedef enum eSRF02_unit{
	cm,
	inch,
	ms
}eSRF02_unit;

typedef enum eSRF02_Cmd{
	srf02_mes_inch,
	srf02_mes_cm,
	srf02_mes_ms,
	srf02_fakeMes_inch,
	srf02_fakeMes_cm,
	srf02_fakeMes_ms,
	srf02_burst,
	srf02_autotuneCmd,
	srf02_chgAddr,
	eSRF02_CMD_Max
}eSRF02_Cmd;


class SRF02 {
public:

	/**
	 * @brief SRF02, initializing SRF02 with address and mode
	 * @param ard I2C Device address
	 * @param mode ranging mode
	 */
	SRF02(uint8_t addr, PoseSonar_t p);
	uint8_t getAddr();
	PoseSonar_t getPose();
	eSRF02_unit getUnit();
	int getMeasDelay_ms();

	private:
		uint8_t _addr;		// address of the sensor
		unsigned long _startTime;	// start time of a ranging, needed
									// to proceed with correct sensor values
		PoseSonar_t _pose;  // circular coordinates: range, azimuth
		eSRF02_unit _unit;
};


#endif /* SRF02_H_ */
