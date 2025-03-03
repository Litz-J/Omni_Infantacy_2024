#include "custom_controller.h"
#include "daemon.h"
#include "bsp_log.h"
#include "custom_controller_protocol.h"

#define ANGLE_SMOOTH_COEF 0.9f

static USARTInstance *custom_controller_usart_instance;

static Custom_Controller_Recv_s recv_data;
static Custom_Controller_Send_s send_data;
static DaemonInstance *custom_controller_daemon_instance;


static void DecodeCustom()
{
    DaemonReload(custom_controller_daemon_instance); // 喂狗
    if(custom_controller_usart_instance->recv_buff[0]!=CUSTOM_CONTROLLER_HEADER) {
        LOGWARNING("[custom] custom controller header error.");
    }
    else {
        //包头正确，进行CRC校验
        uint16_t crc_real=crc_16(
            custom_controller_usart_instance->recv_buff,sizeof(custom_controller_protocol)-2);
        uint16_t crc_recv=(custom_controller_usart_instance->recv_buff[sizeof(custom_controller_protocol)-2]
            |custom_controller_usart_instance->recv_buff[sizeof(custom_controller_protocol)-1]<<8);

        if(crc_real!=crc_recv)
        {
            static uint32_t custom_crc_error_counter=0;
            LOGWARNING("[custom] custom controller CRC error.");
            custom_crc_error_counter++;
            return;
        }

        static float last_joint_angle[3];

        memcpy(&custom_controller_protocol,custom_controller_usart_instance->recv_buff,sizeof(custom_controller_protocol));
        recv_data.pitch=custom_controller_protocol.pitch;
        recv_data.roll=custom_controller_protocol.roll;
        recv_data.yaw=custom_controller_protocol.yaw;

        recv_data.encoder[1]=recv_data.encoder[0];
        recv_data.encoder[0]=custom_controller_protocol.encoder;

        recv_data.joystick_x=custom_controller_protocol.joystick_x;
        recv_data.joystick_y=custom_controller_protocol.joystick_y;
        for(int i=0;i<3;i++)
        {
            // recv_data.joint_angle[i]=custom_controller_protocol.joint_angle[i]/100.0f;
            //发来的数据过一下滤波
            recv_data.joint_angle[i] = (1.0f - ANGLE_SMOOTH_COEF) * last_joint_angle[i]+
                          ANGLE_SMOOTH_COEF * (custom_controller_protocol.joint_angle[i]/100.0f);
            last_joint_angle[i]=recv_data.joint_angle[i];
        }

        for(uint8_t i=0;i<8;i++) {
            recv_data.key[CUSTOM_KEY_LAST][i]=recv_data.key[CUSTOM_KEY_NOW][i];
            recv_data.key[CUSTOM_KEY_LAST_LAST][i]=recv_data.key[CUSTOM_KEY_LAST][i];

            recv_data.key[CUSTOM_KEY_NOW][i]=(custom_controller_protocol.keyvalue&(1<<i))?1:0;

            //处理单击
            if(recv_data.key[CUSTOM_KEY_LAST][i]==0&&recv_data.key[CUSTOM_KEY_NOW][i]==1) {
                //计数
                recv_data.key_count[CUSTOM_KEY_COUNT_PRESS][i]++;
            }
        }


    }

}

static void CustomControllerOfflineCallback(void *id)
{
    recv_data.joystick_x=0;
    recv_data.joystick_y=0;

    USARTServiceInit(custom_controller_usart_instance);
    LOGWARNING("[custom] custom controller offline, restart communication.");
}


Custom_Controller_Recv_s *CustomControllerInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeCustom;
    conf.recv_buff_size = CONTROLLER_RECV_SIZE;
    conf.usart_handle = _handle;
    custom_controller_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = CustomControllerOfflineCallback,
        .owner_id = custom_controller_usart_instance,
        .reload_count = 10,
    };
    custom_controller_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void CustomControllerSend() {

}