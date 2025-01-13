//
// Created by 26090 on 25-1-13.
//

#include "rv2_protocal.h"

#include <crc16.h>
#include <string.h>

void build_rv2_send_data(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len)
{
    static rv2_send_protocol_s rv2_send_data={
        .header = RV2_PROTOCAL_HEADER,
        .reserved = 0x00,
        .aim_x = 0,
        .aim_y = 0,
        .aim_z = 0};

    // 姿态部分
    rv2_send_data.roll=send->roll;
    rv2_send_data.pitch=send->pitch;
    rv2_send_data.yaw=send->yaw;
    //对局信息部分
    rv2_send_data.detect_color=send->enemy_color==COLOR_RED ? 0 : 1;          //0为红，1为蓝
    rv2_send_data.reset_tracker=0;

    //CRC校验
    rv2_send_data.checksum=crc_16((uint8_t *)&rv2_send_data,sizeof(rv2_send_data)-2);

    memcpy(tx_buf,&rv2_send_data,sizeof(rv2_send_data));
    *tx_buf_len=sizeof(rv2_send_data);
}


void parse_rv2_receive_data(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len)
{

}