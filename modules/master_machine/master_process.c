/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "rv2_protocal.h"
#include "rv2_trajectory.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "general_def.h"
#include "bsp_dwt.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

static rv2_recv_protocol_s *rv2_recv;
static trajectory_target_s *trajectory;

//用于低通滤波的时间参数
static float trajectory_pitch_LPF_RC=0.012;
static float trajectory_yaw_LPF_RC=0.02;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = DEGREE_2_RAD*yaw;
    send_data.pitch = DEGREE_2_RAD*pitch;
    send_data.roll = -DEGREE_2_RAD*roll;
}

void VisionSetAimXYZ(float aim_x, float aim_y, float aim_z)
{
    send_data.aim_x = aim_x;
    send_data.aim_y = aim_y;
    send_data.aim_z = aim_z;
}

static USARTInstance *vision_usart_instance;

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    recv_data.offline=1;
    // recv_data.yaw=0;
    // recv_data.pitch=0;
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"


/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 *
 */
static void DecodeVision()
{
    recv_data.offline=0;
    DaemonReload(vision_daemon_instance); // 喂狗

    parse_rv2_receive_data(&recv_data,vision_usart_instance->recv_buff,VISION_RECV_SIZE);

    rv2_trajectory_passin(rv2_recv,&send_data.yaw);

    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    rv2_recv=rv2_protocol_init();
    trajectory=rv2_trajectory_init();

    //recv_data.target=trajectory;

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint32_t vision_send_count=0;
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;

    VisionSetAimXYZ(trajectory->aim_x,trajectory->aim_y,trajectory->aim_z);

    build_rv2_send_data(&send_data,send_buff,&tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.

    vision_send_count++;
}

void VisionTrajectory()
{
    static uint32_t trajectory_dt_cnt=0;
    static float trajectory_dt=0;
    static float current_yaw_circle=0,current_yaw_base=0;
    static float last_pitch,last_yaw;
    bool is_lost=(recv_data.offline!=0)||(recv_data.target_state!=TRACKING);
    trajectory_dt = DWT_GetDeltaT(&trajectory_dt_cnt);

    //计算yaw的圈数用于补偿到总圈数
    current_yaw_circle=(int)(send_data.yaw/PI2);

    //@todo:将自瞄计算放到合适位置
    rv2_trajectory_calculate();

    //自瞄数据过一个低通滤波
    recv_data.pitch=is_lost?last_pitch:trajectory->pitch*RAD_2_DEGREE;
    recv_data.pitch =
        recv_data.pitch * trajectory_dt /(trajectory_pitch_LPF_RC + trajectory_dt) +
        last_pitch * trajectory_pitch_LPF_RC /(trajectory_pitch_LPF_RC + trajectory_dt);
    last_pitch=recv_data.pitch;

    recv_data.yaw=is_lost?last_yaw:trajectory->yaw*RAD_2_DEGREE+current_yaw_circle*360.0f;
    recv_data.yaw =
        recv_data.yaw * trajectory_dt /(trajectory_yaw_LPF_RC + trajectory_dt) +
        last_yaw * trajectory_yaw_LPF_RC /(trajectory_yaw_LPF_RC + trajectory_dt);
    last_yaw=recv_data.yaw;
}
#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    //get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {
        .rx_cbk = DecodeVision
    };
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    build_rv2_send_data(&send_data,send_buff,&tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
