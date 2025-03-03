//
// Created by liz on 25-2-12.
//

#ifndef CUSTOM_CONTROLLER_PROTOCOL_H
#define CUSTOM_CONTROLLER_PROTOCOL_H
#include "crc16.h"

#define CUSTOM_CONTROLLER_HEADER 0xAE


#pragma pack(1)
static struct
{
    uint8_t head;
    float pitch,roll,yaw;
    uint8_t keyvalue;
    int32_t encoder;
    int16_t joystick_x,joystick_y;
    int16_t joint_angle[3];
    uint16_t crc;
}custom_controller_protocol;


#pragma pack()

#endif //CUSTOM_CONTROLLER_PROTOCOL_H
