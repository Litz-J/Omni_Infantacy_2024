//
// Created by 26090 on 25-1-13.
//

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H

#pragma pack(1)
typedef struct ReceivePacket
{
    uint8_t header = 0x5A;
    uint8_t detect color :1; // 0-red 1-blue
    bool reset_tracker :1;
    uint8_t reserved :6;
float roll;
float pitch;
float yaw;
float aim x;
float aim y;
float aim z;
uint16 t checksum = 0;

} rv2_protocol_s;

#pragma pack()

#endif //RV2_PROTOCAL_H
