//
// Created by 26090 on 25-1-13.
//

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H

#include "main.h"
#include "stdbool.h"

#pragma pack(1)
typedef struct ReceivePacket
{
    uint8_t header ;//0x5A
    uint8_t detect_color :1; // 0-red 1-blue
    bool reset_tracker :1;
    uint8_t reserved :6;
float roll;
float pitch;
float yaw;
float aim_x;
float aim_y;
float aim_z;
uint16_t checksum;
} rv2_protocol_s;

#pragma pack()

#endif //RV2_PROTOCAL_H
