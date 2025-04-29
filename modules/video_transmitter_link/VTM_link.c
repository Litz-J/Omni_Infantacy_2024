#include "VTM_link.h"
#include "daemon.h"
#include "bsp_log.h"

#define VTM_RECV_SIZE 255u
#define VTM_SEND_SIZE 36u


static USARTInstance *vtm_link_usart_instance;

static VTM_Recv_s vtm_recv_data;
static VTM_Send_s vtm_send_data;
static DaemonInstance *vtm_daemon_instance;

static VTM_Data_t vtm_data;


void VTMReadData(uint8_t *buff);
void VTMProcessData(VTMID_e message_id);


void Remote_Control_Process();

void VTMReadData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&vtm_data.FrameHeader, &vtm_link_usart_instance->recv_buff[0], LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                // 2个8位拼成16位int
                vtm_data.CmdID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (vtm_data.CmdID)
                {
                case ID_CUSTOM_ROBOT:
                    memcpy(&vtm_data.Custom_Robot_Data, (buff + DATA_Offset), LEN_CUSTOM_ROBOT);
                    break;
                case ID_REMOTE_ROBOT:
                    memcpy(&vtm_data.Remote_Control_Data, (buff + DATA_Offset), LEN_REMOTE_ROBOT);
                    break;
                }
                // 赋值后进行处理
                VTMProcessData(vtm_data.CmdID);
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + vtm_data.FrameHeader.DataLength + LEN_TAIL) == REFEREE_SOF)
        { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            VTMReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + vtm_data.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}

void VTMProcessData(VTMID_e message_id)
{
    switch (message_id)
    {
        case ID_CUSTOM_ROBOT:
            Custom_Robot_Process();
            break;
        case ID_REMOTE_ROBOT:
            Remote_Control_Process();
            break;
        default:
            break;
    }
}

static void DecodeVTM()
{
    DaemonReload(vtm_daemon_instance); // 喂狗
    vtm_recv_data.connected_flag = VTM_CONNECTED;
    VTMReadData(vtm_link_usart_instance->recv_buff);

}

static void VTMLinkOfflineCallback(void *id)
{
    vtm_recv_data.connected_flag = VTM_LOST;

    vtm_recv_data.mouse_key_data[TEMP].mouse.x=0;
    vtm_recv_data.mouse_key_data[TEMP].mouse.y=0;
    vtm_recv_data.mouse_key_data[TEMP].mouse.z=0;
    vtm_recv_data.mouse_key_data[TEMP].mouse.press_l=0;
    vtm_recv_data.mouse_key_data[TEMP].mouse.press_r=0;

    USARTServiceInit(vtm_link_usart_instance);
    LOGWARNING("[custom] custom controller offline, restart communication.");
}



VTM_Recv_s *VTM_LinkInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVTM;
    conf.recv_buff_size = VTM_RECV_SIZE;
    conf.usart_handle = _handle;
    vtm_link_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VTMLinkOfflineCallback,
        .owner_id = vtm_link_usart_instance,
        .reload_count = 10,
    };
    vtm_daemon_instance = DaemonRegister(&daemon_conf);

    return &vtm_recv_data;
}

void VTM_LinkSend() {

}



void Remote_Control_Process()
{
    remote_control_t *remote = (remote_control_t *)&vtm_data.Remote_Control_Data;
    

    // 鼠标解析
    vtm_recv_data.mouse_key_data[TEMP].mouse.x = remote->mouse_x;
    vtm_recv_data.mouse_key_data[TEMP].mouse.y = remote->mouse_y;
    vtm_recv_data.mouse_key_data[TEMP].mouse.z = remote->mouse_z;
    vtm_recv_data.mouse_key_data[TEMP].mouse.press_l = remote->left_button_down;
    vtm_recv_data.mouse_key_data[TEMP].mouse.press_r = remote->right_button_down;

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS] = remote->keyboard_value;
    if (vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_CTRL] = vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS];
    else
        memset(&vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS].shift) // shift键按下
        vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_SHIFT] = vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS];
    else
        memset(&vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = vtm_recv_data.mouse_key_data[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = vtm_recv_data.mouse_key_data[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = vtm_recv_data.mouse_key_data[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = vtm_recv_data.mouse_key_data[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            vtm_recv_data.mouse_key_data[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            vtm_recv_data.mouse_key_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            vtm_recv_data.mouse_key_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&vtm_recv_data.mouse_key_data[LAST], &vtm_recv_data.mouse_key_data[TEMP], sizeof(mouse_keyboard_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

void Custom_Robot_Process(void)
{
    static custom_controller_protocol_t *custom_controller_data = (custom_controller_protocol_t *)&vtm_data.Custom_Robot_Data;


}