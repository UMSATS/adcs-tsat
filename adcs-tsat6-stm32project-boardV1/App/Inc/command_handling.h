/*
 * command_handling.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_COMMAND_HANDLING_H_
#define INC_COMMAND_HANDLING_H_

#include "tuk/tuk.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "magnetorquer_commands.h"

extern CAN_HandleTypeDef hcan1;
/**
* Handles incoming messages from the CAN bus. Called by CAN Wrapper Module.
*
* @param hcan Handle for the source CAN peripheral.
* @param msg The received message.
*/
void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg);

void On_CAN_Error(const CANWrapper_ErrorInfo *error);

#endif /* INC_COMMAND_HANDLING_H_ */
