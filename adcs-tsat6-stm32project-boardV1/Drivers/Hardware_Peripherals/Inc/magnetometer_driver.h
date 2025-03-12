/*
 * magnetometer_driver.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Alexandr Yermakov
 */

/*

Datasheet URL:
https://media.digikey.com/pdf/Data%20Sheets/MEMSIC%20PDFs/MMC5983MA_RevA_4-3-19.pdf
*/

#ifndef HARDWARE_PERIPHERALS_INC_MAGNETOMETER_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_MAGNETOMETER_DRIVER_H_

#include "stm32l4xx_hal.h"

#define MAG_XOUT_L 0x00
#define MAG_XOUT_H 0x01
#define MAG_YOUT_L 0x02
#define MAG_YOUT_H 0x03
#define MAG_ZOUT_L 0x04
#define MAG_ZOUT_H 0x05
#define MAG_TEMP_L 0x06
#define MAG_TEMP_H 0x07
#define MAG_STATUS 0x08
#define MAG_CONTROL_0 0x09
#define MAG_PRODUCT_ID 0x2F

// Function to write to MAGNETOMETER register
void MAG_WriteReg(uint8_t reg, uint8_t data);

// Function to read from MAGNETOMETER register
uint8_t MAG_ReadReg(uint8_t reg);

// Function to read magnetic field data
// pData: Pointer to an array where the magnetic field data will be stored
// The array has 3 elements for X, Y, and Z
void MAG_ReadMagneticField(int16_t *pData);

// Function to initialize MAGNETOMETER
void MAG_Init(void);

//Function to get the ProductID
void MAG_ProductID(void);

//Function to convert the measuerements to Teslas
void MAG_ConvertToTeslas(const int16_t *rawData, float *teslaData);

#endif
