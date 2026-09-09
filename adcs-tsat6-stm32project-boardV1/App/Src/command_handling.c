/*
 * command_handling.c
 */

#include "command_handling.h"
#include "tuk/tuk.h"

void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg)
{
	// TODO TEMP ack
	uint8_t body[CAN_MAX_BODY_SIZE] = {0};
	CANWrapper_Transmit(&hcan1, NODE_PAYLOAD, CMD_PLD_GET_ACTIVE_ENVS, body);

//	uint8_t id = GET_MSG_DATA(msg->body, 0, uint8_t);
//	uint8_t direction = GET_MSG_DATA(msg->body, 1, uint8_t);

	switch (msg->cmd)
	{
	case CMD_ADCS_SET_MAGNETORQUER_DIRECTION:

		setMagnetorquerDirection(GET_MSG_DATA(msg->body, 0, uint8_t), GET_MSG_DATA(msg->body, 1, uint8_t));

		break;
	case CMD_ADCS_GET_MAGNETORQUER_DIRECTION:

		getMagnetorquerDirection(GET_MSG_DATA(msg->body, 0, uint8_t));

		break;
	default:
		break;
	}

	//      MAX6822_WDI_Toggle();
	//
	//      if(!CAN_Queue_IsEmpty(&can_queue))
	//      {
	//          CANMessage_t can_message;
	//          CAN_Queue_Dequeue(&can_queue, &can_message);
	//          switch (can_message.command)
	//          {
	//              case 0xB0: //STM32 Reset
	//                  CAN_Send_Default_ACK(can_message);
	//                  MAX6822_Manual_Reset();
	//                  break;
	//
	//              case 0xB1: //Magnetorquer 1 Full Strength
	//                  Magnetorquer1_Full_Strength();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB2: //Magnetorquer 2 Full Strength
	//                  Magnetorquer2_Full_Strength();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB3: //Magnetorquer 3 Full Strength
	//                  Magnetorquer3_Full_Strength();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB4: //Magnetorquer 1 Off
	//                  Magnetorquer1_Off();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB5: //Magnetorquer 2 Off
	//                  Magnetorquer2_Off();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB6: //Magnetorquer 3 Off
	//                  Magnetorquer3_Off();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB7: //Magnetorquer 1 Forward Direction
	//                  Magnetorquer1_Forward();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB8: //Magnetorquer 2 Forward Direction
	//                  Magnetorquer2_Forward();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xB9: //Magnetorquer 3 Forward Direction
	//                  Magnetorquer3_Forward();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xBA: //Magnetorquer 1 Reverse Direction
	//                  Magnetorquer1_Reverse();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xBB: //Magnetorquer 2 Reverse Direction
	//                  Magnetorquer2_Reverse();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              case 0xBC: //Magnetorquer 3 Reverse Direction
	//                  Magnetorquer3_Reverse();
	//                  CAN_Send_Default_ACK(can_message);
	//                  break;
	//
	//              default:
	//                  break;
	//          }
	//      }
}

void On_CAN_Error(const CANWrapper_ErrorInfo *error)
{
	// TODO: Handle CAN errors.
}

