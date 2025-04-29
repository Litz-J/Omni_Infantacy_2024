//
// Created by liz on 25-2-12.
//

#ifndef VTM_LINK_PROTOCOL_H
#define VTM_LINK_PROTOCOL_H
#include "crc16.h"
#include "rm_referee.h"
#include "crc_ref.h"
#include "referee_protocol.h"
#include "remote_control.h"

#pragma pack(1)

typedef struct
{
    uint8_t data[30];
}custom_robot_data_t;

typedef struct
{
    uint8_t data[30];
}robot_custom_data_t;

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}remote_control_t;

// 图传链路数据包
typedef struct
{
    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    custom_robot_data_t Custom_Robot_Data;      //0x302 : 自定义控制器到机器人数据包
    robot_custom_data_t Robot_Custom_Data;      //0x306 : 机器人到自定义控制器数据包
    remote_control_t Remote_Control_Data;       //0x304 : 客户端键鼠数据

    uint8_t init_flag;

} VTM_Data_t;

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum
{
    ID_CUSTOM_ROBOT = 0x302,
    ID_ROBOT_CUSTOM = 0x306,
    ID_REMOTE_ROBOT = 0x304,
} VTMID_e;

typedef enum
{
    LEN_CUSTOM_ROBOT = 30,
    LEN_ROBOT_CUSTOM = 30,
    LEN_REMOTE_ROBOT = 12,
} VTMDataLength_e;


#pragma pack()

#endif //VTM_LINK_PROTOCOL_H
