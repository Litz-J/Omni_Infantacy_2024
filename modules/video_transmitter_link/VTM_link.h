//
// Created by liz on 25-2-12.
//

#ifndef VTM_LINK_H
#define VTM_LINK_H

#include <VTM_link_protocol.h>
#include "VTM_link_custom.h"

#include "bsp_usart.h"

typedef enum
{
    VTM_LOST = 0,
    VTM_CONNECTED = 1,
    VTM_ERROR = 2
} VTM_LINK_STATUS;


#pragma pack(1)

typedef struct
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} mouse_keyboard_t;

typedef struct
{
	VTM_LINK_STATUS connected_flag :8;						    // 图传链路连接指示
	mouse_keyboard_t mouse_key_data[2];             // [0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断

} VTM_Recv_s;

typedef struct
{

} VTM_Send_s;
#pragma pack()

VTM_Recv_s *VTM_LinkInit(UART_HandleTypeDef *_handle);

void VTM_LinkSend();


#endif //VTM_LINK_H
