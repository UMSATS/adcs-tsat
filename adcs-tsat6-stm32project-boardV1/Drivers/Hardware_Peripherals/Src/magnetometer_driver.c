/*
 * 	magnetometer_driver.c
 *
 *  Created on: Jan 19, 2024
 *  
 * 	Author: Alexandr Yermakov, Andrii Kvasnytsia
 * 	
 * 	Datasheet URL:
 *	https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/333/MMC5983MA_RevA_4-3-19.pdf
 */

#include <magnetometer_driver.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include "main.h"

extern SPI_HandleTypeDef hspi3;


// Function to write to MAGNETOMETER register
void MAG_WriteReg(uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_RESET);
	uint8_t txData[2] = {reg, data};
	HAL_SPI_Transmit(&hspi3, txData, 2, 100);
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
}

// Function to read from MAGNETOMETER register
uint8_t MAG_ReadReg(uint8_t reg) {
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_RESET);
	uint8_t txData = 0x80 | reg; // MSB must be set for read
	uint8_t rxData;
	HAL_SPI_Transmit(&hspi3, &txData, 1, 100);
	HAL_SPI_Receive(&hspi3, &rxData, 1, 100);
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
	return rxData;
}

// Function to read magnetic field data
void MAG_ReadMagneticField(int16_t *pData) {
	MAG_WriteReg(MAG_CONTROL_0, 0x01);
	HAL_Delay(10);

	uint8_t tmpbuffer[6] = {0};

	tmpbuffer[0] = MAG_ReadReg(MAG_XOUT_L);
	tmpbuffer[1] = MAG_ReadReg(MAG_XOUT_H);
	tmpbuffer[2] = MAG_ReadReg(MAG_YOUT_L);
	tmpbuffer[3] = MAG_ReadReg(MAG_YOUT_H);
	tmpbuffer[4] = MAG_ReadReg(MAG_ZOUT_L);
	tmpbuffer[5] = MAG_ReadReg(MAG_ZOUT_H);

	uint16_t rawX = ((uint16_t)tmpbuffer[1] << 8) | tmpbuffer[0];
	uint16_t rawY = ((uint16_t)tmpbuffer[3] << 8) | tmpbuffer[2];
	uint16_t rawZ = ((uint16_t)tmpbuffer[5] << 8) | tmpbuffer[4];

	pData[0] = (int16_t)(rawX - 32768);
	pData[1] = (int16_t)(rawY - 32768);
	pData[2] = (int16_t)(rawZ - 32768);
}

// Function to initialize MAGNETOMETER
void MAG_Init(void) {
	// Set CS pin high
	HAL_GPIO_WritePin(MAG1_nCS_GPIO_Port, MAG1_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
	// Configure magnetic sensor settings
	MAG_WriteReg(MAG_CONTROL_0, 0x01); // Reset sensor
	HAL_Delay(1); // Wait for reset to complete
}

//Function to get the Product ID from the Magnetometer to check if a connection has been established. Expected Return: 0b48
void MAG_ProductID(void){
	uint8_t prodID = MAG_ReadReg(MAG_PRODUCT_ID);
	printf("Magnetometer Product ID: 0x%02X\r\n", prodID);
}

//Function to convert the Gauss Counts to Teslas
static float MAG_CountsToTesla(int16_t counts) {
    // Sensitivity: 4096 counts per Gauss; 1 Tesla = 10,000 Gauss.
    return ((float)counts) / (4096.0f * 10000.0f)*1000;
}

// Converts raw magnetic field data to Teslas
void MAG_ConvertToTeslas(const int16_t *rawData, float *teslaData) {
    teslaData[0] = MAG_CountsToTesla(rawData[0]);
    teslaData[1] = MAG_CountsToTesla(rawData[1]);
    teslaData[2] = MAG_CountsToTesla(rawData[2]);
}


