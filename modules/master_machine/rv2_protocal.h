//
// Created by 26090 on 25-1-13.
//

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H
#include "master_process.h"
#include "stdbool.h"

#define RV2_PROTOCAL_HEADER 0x5A


#pragma pack(1)
typedef struct
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
} rv2_send_protocol_s;

typedef struct
{
    uint8_t header;     //0xA5
    bool tracking :1;
    uint8_t id :3;  //0-outpost 6-guard 7-base
    uint8_t armors_num :3;//2-balance 3-outpost 4-no
    uint8_t reserved :1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum;
}rv2_recv_protocol_s;

#pragma pack()

rv2_recv_protocol_s *rv2_protocol_init(void);
void build_rv2_send_data(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len);
void parse_rv2_receive_data(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len);



#endif //RV2_PROTOCAL_H
