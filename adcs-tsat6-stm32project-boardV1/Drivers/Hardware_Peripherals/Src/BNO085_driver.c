/*
 * FILENAME: BNO085_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the BNO085 IMU.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: May 20, 2022
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>

#include "tuk/tuk.h"

#include "stm32l4xx_hal.h"
#include "BNO085_driver.h"

//###############################################################################################
//Define Directives
//###############################################################################################
#define BNO085_ROTATION_VECTOR_TELEMETRY_COMMAND_CODE_1_OF_2    0x35 // TODO Find or add updated tuk command from can_command_list.h
#define BNO085_ROTATION_VECTOR_TELEMETRY_COMMAND_CODE_2_OF_2    0x36 // TODO Find or add updated tuk command from can_command_list.h

#define BNO085_NUMBER_OF_INIT_MESSAGES                          0x02

//###############################################################################################
//Global Variable Declarations
//###############################################################################################
extern SPI_HandleTypeDef BNO085_SPI;
struct Rotation_Vector rotation_vector;
uint8_t init_counter = 0; //used to determine if the init process has completed
uint8_t channel_2_sequence = 0; //used to keep track of the channel 2 sequence number
uint8_t rotation_vector_telemetry_sequence = 0; //used to keep track of CAN sequence number

//###############################################################################################
//Public Driver Functions
//###############################################################################################
HAL_StatusTypeDef BNO085_Init()
{
    HAL_StatusTypeDef operation_status;

    BNO085_Reset();

    //for debugging -> to see how many init messages there are
    //keep track of how many times BNO085_Interrupt_Handler gets called
    HAL_Delay(60000);

    uint8_t timeout_counter = 0;
    while (init_counter < BNO085_NUMBER_OF_INIT_MESSAGES)
    {
        HAL_Delay(10);
        timeout_counter++;
        if (timeout_counter >= 50) //if 0.5s or more has passed
        {
            timeout_counter = 0;
            BNO085_Reset();
        }
    }

    rotation_vector.q_i = 0x0000;
    rotation_vector.q_j = 0x0000;
    rotation_vector.q_k = 0x0000;
    rotation_vector.q_real = 0x0000;
    rotation_vector.accuracy = 0x0000;

    //for debugging -> send the command to get the device id & info
    uint8_t product_id_request[6] = {
            0x06,               //length LSB
            0x00,               //length MSB
            0x02,               //channel
            channel_2_sequence, //sequence number
            0xF9,               //report ID
            0x00                //reserved
    };
    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_RESET);
    operation_status = HAL_SPI_Transmit(&BNO085_SPI, product_id_request, 6, BNO085_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    channel_2_sequence++;
    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_RESET);
    HAL_Delay(30000); //wait for the product id response to come in
    //the delay is so the sequence number of the rotation vector report is correct

    uint8_t set_feature_command[21] = {
            0x15,               //length LSB
            0x00,               //length MSB
            0x02,               //channel
            channel_2_sequence, //sequence number
            0xFD,               //report ID
            0x05,               //feature report id
            0x00,               //feature flags
            0x00,               //change sensitivity LSB
            0x00,               //change sensitivity MSB
            0x00,               //report interval LSB
            0x87,               //report interval
            0x93,               //report interval
            0x03,               //report interval MSB
            0x00,               //batch interval LSB
            0x00,               //batch interval
            0x00,               //batch interval
            0x00,               //batch interval MSB
            0x00,               //sensor-specific configuration word LSB
            0x00,               //sensor-specific configuration word
            0x00,               //sensor-specific configuration word
            0x00                //sensor-specific configuration word MSB
    };
    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_RESET);
    operation_status = HAL_SPI_Transmit(&BNO085_SPI, set_feature_command, 21, BNO085_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    channel_2_sequence++;

error:
    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_SET);
    return operation_status;
}

void BNO085_Reset()
{
    HAL_GPIO_WritePin(BNO085_NRST_GPIO, BNO085_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); //Reset pin must be held low for 10ns
    init_counter = 0;
    channel_2_sequence = 0;
    HAL_GPIO_WritePin(BNO085_NRST_GPIO, BNO085_NRST_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef BNO085_Interrupt_Handler()
{
    HAL_StatusTypeDef operation_status;
    uint8_t shtp_message[100];

    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Receive(&BNO085_SPI, shtp_message, 100, BNO085_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;

    if (init_counter < BNO085_NUMBER_OF_INIT_MESSAGES)
        init_counter++;

    if (shtp_message[2] == 0x02) //if channel 2
        channel_2_sequence++;

    if (shtp_message[2] == 0x03) //if channel 3 - it's a rotation vector report
    {
        rotation_vector.q_i = ((uint16_t)shtp_message[14] << 8) | shtp_message[13];
        rotation_vector.q_j = ((uint16_t)shtp_message[16] << 8) | shtp_message[15];
        rotation_vector.q_k = ((uint16_t)shtp_message[18] << 8) | shtp_message[17];
        rotation_vector.q_real = ((uint16_t)shtp_message[20] << 8) | shtp_message[19];
        rotation_vector.accuracy = ((uint16_t)shtp_message[22] << 8) | shtp_message[21];
        operation_status = BNO085_Send_Rotation_Vector_Telemetry();
    }

error:
    HAL_GPIO_WritePin(BNO085_H_CSN_GPIO, BNO085_H_CSN_PIN, GPIO_PIN_SET);
    return operation_status;

    //get a fixed amount of data (the max of init messages, and rotation report)
    //*We'll set it to 100 for now though, just to make sure we get all the messages at full length, in case there's any
    //we didn't account for, or if some are longer than expected. Then we'll set it back to normal length after figuring that out*
    //there may also be a reset message - we'll find out when we test, then we can just search for the report ID in the documentation
    //(capture it in an array, not a struct, bc of structure padding)
    //we need to make sure only rotation vector reports are sent on channel 3; check what channels the init messages belong to
}

HAL_StatusTypeDef BNO085_Send_Rotation_Vector_Telemetry()
{
	ErrorCode operation_status;

    uint8_t message1[7] = {0};

    message1[0] = rotation_vector_telemetry_sequence;
    message1[1] = rotation_vector.q_i >> 8;
    message1[2] = rotation_vector.q_i;
    message1[3] = rotation_vector.q_j >> 8;
    message1[4] = rotation_vector.q_j;
    message1[5] = rotation_vector.q_k >> 8;
    message1[6] = rotation_vector.q_k;

    operation_status=CANWrapper_Transmit(&hcan1, NODE_CDH, BNO085_ROTATION_VECTOR_TELEMETRY_COMMAND_CODE_1_OF_2, message1);
    if (operation_status != HAL_OK) goto error;


    uint8_t message2[7] = {0};

    message2[0] = rotation_vector_telemetry_sequence;
    message2[1] = rotation_vector.q_real >> 8;
    message2[2] = rotation_vector.q_real;
    message2[3] = rotation_vector.accuracy >> 8;
    message2[4] = rotation_vector.accuracy;

    operation_status=CANWrapper_Transmit(&hcan1, NODE_CDH, BNO085_ROTATION_VECTOR_TELEMETRY_COMMAND_CODE_2_OF_2, message2);
    if (operation_status != HAL_OK) goto error;
    rotation_vector_telemetry_sequence++;

error:
    return operation_status;
}
