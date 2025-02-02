/*
 * LTC6804-STM-G.h
 *
 *  Created on: Jan 19, 2025
 *      Author: Ajay
 */

#ifndef INC_LTC6804_STM_G_H_
#define INC_LTC6804_STM_G_H_

#include "main.h"

enum ltc6804_command_codes_e {
    CMD_WRCFG   = 0x001, // Write Configuration Register Group
    CMD_RDCFG   = 0x002, // Read Configuration Register Group
    CMD_RDCVA   = 0x004, // Read Cell Voltage Register Group A
    CMD_RDCVB   = 0x006, // Read Cell Voltage Register Group B
    CMD_RDCVC   = 0x008, // Read Cell Voltage Register Group C
    CMD_RDCVD   = 0x00A, // Read Cell Voltage Register Group D
    CMD_RDAUXA  = 0x00C, // Read Auxiliary Register Group A
	CMD_RDAUXB  = 0x00E, // Read Auxiliary Register Group B
    CMD_RDSTATA = 0x010, // Read Status Register Group A
    CMD_RDSTATB = 0x012, // Read Status Register Group B
    CMD_ADCV    = 0x360, // Start Cell Voltage ADC Conversion and Poll Status
    CMD_ADOW    = 0x228, // Start Open Wire ADC Conversion and Poll Status
    CMD_CVST    = 0x207, // Start Self-Test Cell Voltage Conversion and Poll Status
    CMD_ADAX    = 0x560, // Start GPIOs ADC Conversion and Poll Status
    CMD_AXST    = 0x407, // Start Self-Test GPIOs Conversion and Poll Status
    CMD_ADSTAT  = 0x468, // Start Status group ADC Conversion and Poll Status
    CMD_STATST  = 0x40F, // Start Self-Test Status group Conversion and Poll Status
    CMD_ADCVAX  = 0x46F, // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
    CMD_CLRCELL = 0x711, // Clear Cell Voltage Register Group
    CMD_CLRAUX  = 0x712, // Clear Auxiliary Register Group
    CMD_CLRSTAT = 0x713, // Clear Status Register Group
    CMD_PLADC   = 0x714, // Poll ADC Conversion Status
    CMD_DIAGN   = 0x715, // Diagnose MUX and Poll Status
    CMD_WRCOMM  = 0x721, // Write COMM Register Group
    CMD_RDCOMM  = 0x722, // Read COMM Register Group
    CMD_STCOMM  = 0x723  // Start I2C/SPI Communication
};
typedef struct{
	uint16_t packVoltage;
	uint8_t SOC;
}BMSData;

typedef struct{
	uint16_t slaveID;
	uint16_t peakTemp;
	uint16_t lowestTemp;
	uint16_t highestCellVoltage;
	uint16_t lowestCellVoltage;
}
BMSBoard;

// Function to check how many cells are present
uint8_t populateBP(BMSBoard *slaveBoard);

// Get voltage and temperature data from BMS slave
uint16_t getSlaveData(BMSBoard *slaveBoard);

// Get highest cell voltage
uint16_t getPeakCellVoltage(BMSBoard *slaveBoard);

// Get lowest cell voltage
uint16_t getLowCellVoltage(BMSBoard *slaveBoard);

// Get highest cell Temp
uint16_t getPeakCellTemp(BMSBoard *slaveBoard);

// Get lowest cell Temp
uint16_t getLowCellTemp(BMSBoard *slaveBoard);
#endif /* INC_LTC6804_STM_G_H_ */
