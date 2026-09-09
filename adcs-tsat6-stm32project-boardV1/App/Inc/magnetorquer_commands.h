/*
 * magnetorquer_commands.h
 *
 *  Created on: Sep 9, 2026
 *      Author: Andrew
 */

#ifndef INC_MAGNETORQUER_COMMANDS_H_
#define INC_MAGNETORQUER_COMMANDS_H_

#include "tuk/tuk.h"



extern CAN_HandleTypeDef hcan1;

// TODO write function description
void setMagnetorquerDirection(uint8_t id,uint8_t direction);

void getMagnetorquerDirection(uint8_t id);

#endif /* INC_MAGNETORQUER_COMMANDS_H_ */
