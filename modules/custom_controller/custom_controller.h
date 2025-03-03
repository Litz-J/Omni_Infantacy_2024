//
// Created by liz on 25-2-12.
//

#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include "bsp_usart.h"

#define CONTROLLER_RECV_SIZE 30u
#define CONTROLLER_SEND_SIZE 36u

#define CUSTOM_KEY_NOW 0
#define CUSTOM_KEY_LAST 1
#define CUSTOM_KEY_LAST_LAST 2

#define CUSTOM_KEY_COUNT_PRESS 0


#pragma pack(1)

typedef struct
{
	float yaw;
	float pitch;
	float roll;

	uint8_t key[3][8];
	uint8_t key_count[3][8];

	int32_t encoder[2];

	int16_t joystick_x,joystick_y;

	float joint_angle[3];

} Custom_Controller_Recv_s;

typedef struct
{

}Custom_Controller_Send_s;
#pragma pack()

Custom_Controller_Recv_s *CustomControllerInit(UART_HandleTypeDef *_handle);

void CustomControllerSend();




#endif //CUSTOM_CONTROLLER_H
