/*
 * GYRO_A3G4250DTR_driver.c
 *
 *  Created on: Jan 19, 2024
 *      Author: Alexandr Yermakov, Andrii Kvasnytsia
 */

/*

Datasheet URL:
https://www.st.com/resource/en/datasheet/a3g4250d.pdf

*/

#include "GYRO_A3G4250DTR_driver.h"
#include "stm32l4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

void GYRO_WriteReg(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_RESET);

    uint8_t txData[2] = {reg, data};

    HAL_SPI_Transmit(&hspi2, txData, 2, 100);
    HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_SET);
}

uint8_t GYRO_ReadReg(uint8_t reg) {
	HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_RESET);

    uint8_t txData = 0x80 | reg; // MSB must be set for read
	uint8_t rxData;

	HAL_SPI_Transmit(&hspi2, &txData, 1, 100);
	HAL_SPI_Receive(&hspi2, &rxData, 1, 100);
    HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_SET);

    return rxData;
}

void GYRO_ReadAngRate(int16_t *pData){
	uint8_t tmpbuffer[6] = {0};

	uint8_t cmd = 0xC0 | (GYRO_OUT_X_L & 0x3F);  // Set read bit (bit7) and auto-increment bit (bit6)
	uint8_t buffer[6] = {0};

//	tmpbuffer[0] = GYRO_ReadReg(GYRO_OUT_X_L);
//	tmpbuffer[1] = GYRO_ReadReg(GYRO_OUT_X_H);
//	tmpbuffer[2] = GYRO_ReadReg(GYRO_OUT_Y_L);
//	tmpbuffer[3] = GYRO_ReadReg(GYRO_OUT_Y_H);
//	tmpbuffer[4] = GYRO_ReadReg(GYRO_OUT_Z_L);
//	tmpbuffer[5] = GYRO_ReadReg(GYRO_OUT_Z_H);
//
//	pData[0] = (int16_t)(tmpbuffer[1] << 8 | tmpbuffer[0]);
//	pData[1] = (int16_t)(tmpbuffer[3] << 8 | tmpbuffer[2]);
//	pData[2] = (int16_t)(tmpbuffer[5] << 8 | tmpbuffer[4]);

	HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);

	HAL_SPI_Receive(&hspi2, buffer, 6, 100);

	HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_SET);

	pData[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
	pData[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
	pData[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void GYRO_Init(void) {
  // Set CS pin high
  HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GYR2_nCS_GPIO_Port, GYR2_nCS_Pin, GPIO_PIN_SET);
  // Configure gyroscope settings
  GYRO_WriteReg(GYRO_CTRL_REG1, 0x0F); // Enable all axes, normal mode, 100Hz data rate
}

static float GYRO_CountsToDPS(int16_t counts) {
    return counts * 0.00875f;  // 0.00875 dps/digit
}

// Convert an array of raw gyroscope readings to dps.
void GYRO_ConvertToDPS(const int16_t *rawData, float *dpsData) {
    dpsData[0] = GYRO_CountsToDPS(rawData[0]);
    dpsData[1] = GYRO_CountsToDPS(rawData[1]);
    dpsData[2] = GYRO_CountsToDPS(rawData[2]);
}


