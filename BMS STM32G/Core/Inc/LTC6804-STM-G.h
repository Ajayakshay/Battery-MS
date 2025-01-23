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
// GPIO pin definitions for Chip Select (CS) control
#define LTC6804_CS_PIN GPIO_PIN_0
#define LTC6804_CS_PORT GPIOA
// Function to pull CS low, signaling the start of SPI communication
void LTC6804_Select(void) {
    HAL_GPIO_WritePin(LTC6804_CS_PORT, LTC6804_CS_PIN, GPIO_PIN_RESET);//SELECTS SLAVE BY PULLING DOWN LINE
}
// Function to release CS, signaling the end of SPI communication
void LTC6804_Deselect(void) {
    HAL_GPIO_WritePin(LTC6804_CS_PORT, LTC6804_CS_PIN, GPIO_PIN_SET);              // DESELECTS SLAVE BY PULLING UP LINE
}

// Function to send a command to the LTC6804 over SPI
// cmd: Pointer to the command array
// length: Number of bytes to transmit
void LTC6804_SendCommand(uint16_t *cmd, uint16_t length) {
    LTC6804_Select(); // Selects slave by pulling down line
    HAL_SPI_Transmit(&hspi1, cmd, length, HAL_MAX_DELAY); //
    LTC6804_Deselect(); // Deselects slave by pulling up line
}
// Function to send a command and read data from the LTC6804
// cmd: Pointer to the command array
// data: Pointer to the buffer for received data
// length: Number of bytes to read
void LTC6804_ReadData(uint16_t *cmd, uint16_t *data, uint16_t length) {
    LTC6804_Select(); // Pull CS low
    HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY); // Send command (2 bytes)
    HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY); // Receive data
    LTC6804_Deselect(); // Release CS
}
// Function to start cell voltage conversion
// This sends the ADCV (Start Cell Voltage Conversion) command to the LTC6804
void LTC6804_StartCellVoltageConversion(void) {
    uint16_t cmd = CMD_ADCV; // ADCV Normal Mode, Discharge Disabled
    LTC6804_SendCommand(cmd, 2);
}
// Function to read cell voltages from the LTC6804
// voltages: Array to store the cell voltage values (16-bit each)
void LTC6804_ReadCellVoltages(uint16_t *voltages) {
    uint16_t cmd = CMD_RDCVA; // RDCVA Command to read cell voltage group A
    uint8_t rx_data[6]; // Buffer for 6 bytes of received data (3 cell voltages)

    // Send the read command and receive data
    LTC6804_ReadData(cmd, rx_data, 6);

    // Convert the received bytes into cell voltage values
    for (int i = 0; i < 3; i++) {
        voltages[i] = (rx_data[2 * i] | (rx_data[2 * i + 1] << 8)); // Combine LSB and MSB
    }
}

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
