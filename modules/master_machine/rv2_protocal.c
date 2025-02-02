//
// Created by 26090 on 25-1-13.
//

#include "rv2_protocal.h"

#include <bsp_log.h>
#include <crc16.h>
#include <string.h>

static uint8_t send_color;

static rv2_recv_protocol_s rv2_recv_data={0};

rv2_recv_protocol_s *rv2_protocol_init(void)
{
    memset(&rv2_recv_data,0,sizeof(rv2_recv_data));
    return &rv2_recv_data;
}

void build_rv2_send_data(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len)
{
    static rv2_send_protocol_s rv2_send_data={
        .header = RV2_PROTOCAL_HEADER,
        .reserved = 0x00,
        .aim_x = 0,
        .aim_y = 0,
        .aim_z = 0};

    // // 姿态部分
    rv2_send_data.roll=send->roll;
    rv2_send_data.pitch=send->pitch;
    rv2_send_data.yaw=send->yaw;
    //对局信息部分
    rv2_send_data.detect_color=send->enemy_color==COLOR_RED ? 1 : 0;          //自身颜色0为红，1为蓝
    rv2_send_data.detect_color=send_color;

    // rv2_send_data.reset_tracker=0;

    rv2_send_data.aim_x=send->aim_x;
    rv2_send_data.aim_y=send->aim_y;
    rv2_send_data.aim_z=send->aim_z;

    //CRC校验
    rv2_send_data.checksum=crc_16((uint8_t *)&rv2_send_data,sizeof(rv2_send_data)-2);

    memcpy(tx_buf,&rv2_send_data,sizeof(rv2_send_data));
    *tx_buf_len=sizeof(rv2_send_data);
}


void parse_rv2_receive_data(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len)
{
    //包头校验
    if(rx_buf[0]==0xA5)
    {
        //CRC校验
        uint16_t checksum=crc_16(rx_buf,rx_buf_len-2);
        if(rx_buf[rx_buf_len-1]==((checksum&0xFF00)>>8)&&(rx_buf[rx_buf_len-2]==(checksum&0x00FF)))
        {
            memcpy(&rv2_recv_data,rx_buf,sizeof(rv2_recv_data));
            receive->target_state=rv2_recv_data.tracking;
            //判别识别号码和装甲板数量，属于为了兼容的转换性设置
            switch (rv2_recv_data.id) {
                case 0:
                    receive->target_type = OUTPOST;
                    break;
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                    receive->target_type = rv2_recv_data.id;
                    break;
                case 6:
                    receive->target_type = SENTRY;
                    break;
                case 7:
                    receive->target_type = BASE;
                    break;
                default:
                    break;
            }
        }
        else
        {
            LOGERROR("RV2 Receive checksum error");
            return;
        }
    }
    else
    {
        LOGERROR("RV2 Receive Header error");
        return;
    }
}