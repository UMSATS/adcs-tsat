/*
 * magnetoruer_commands.c
 *
 *  Created on: Sep 9, 2026
 *      Author: Andrew
 */

#include "magnetorquer_commands.h"

// Private Variables
uint8_t magnetorquer_id_status[3]= {0};
uint8_t magnetorquer_direction_status[3]= {0};



void setMagnetorquerDirection(uint8_t id,uint8_t direction){
	magnetorquer_direction_status[id]= direction;
	magnetorquer_update(id, direction);
}

void getMagnetorquerDirection(uint8_t id){

	uint8_t ack_msg_body [7] = {0} ;

	SET_MSG_DATA(ack_msg_body, 0, CmdID, CMD_ADCS_GET_MAGNETORQUER_DIRECTION);
	SET_MSG_DATA(ack_msg_body, 1, uint8_t, magnetorquer_direction_status[id] );

	CANWrapper_Transmit(&hcan1, NODE_CDH, CMD_CDH_PROCESS_RETURN, ack_msg_body);

}
