// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "controller.h"
#include "rm_referee.h"
#include "user_lib.h"
#include "rv2_trajectory.h"
#include "VTM_link.h"
// bsp
#include <custom_controller.h>

#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static VTM_Recv_s *vtm_recv_data;
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Custom_Controller_Recv_s *custom_recv_data;   //自定义控制器接收消息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static float chassis_speed_rocker= 0, chassis_speed_mouse = 0;                   //十级10000
float chassis_rotate_speed_mouse,chassis_fastrotate_speed_mouse;

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vtm_recv_data = VTM_LinkInit(&huart1);
    // custom_recv_data
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口
    // custom_recv_data = CustomControllerInit(&huart1);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    shoot_cmd_send.bullet_speed = SMALL_AMU_25;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
        if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}



/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],小陀螺
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode=LOAD_STOP;
        gimbal_cmd_send.lid_mode=LID_CLOSE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        //chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW_DIAGONAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode=LOAD_STOP;
        gimbal_cmd_send.lid_mode=LID_CLOSE;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 跟随
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode=LOAD_STOP;
        gimbal_cmd_send.lid_mode=LID_CLOSE;
        
    }

    gimbal_cmd_send.yaw += 0.002f * (float)rc_data[TEMP].rc.rocker_l_;
    gimbal_cmd_send.pitch -= 0.002f * (float)rc_data[TEMP].rc.rocker_l1;


    // 云台软件限位,这行之后不能再修改云台角度！！！！
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }

    chassis_speed_rocker=0.2;

    chassis_cmd_send.vx = (float)rc_data[TEMP].rc.rocker_r1/660.0f * chassis_speed_rocker; // 竖直方向
    chassis_cmd_send.vy = -(float)rc_data[TEMP].rc.rocker_r_/660.0f * chassis_speed_rocker; // _水平方向

    chassis_cmd_send.wz = 2200.0f;
    shoot_cmd_send.shoot_rate = 8;
}

static void RemoteShootSet()
{
    shoot_cmd_send.shoot_rate = 10;
    //右上默认状态
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode=LOAD_STOP;
    gimbal_cmd_send.lid_mode=LID_CLOSE;

    if(custom_recv_data == NULL || switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        // 无自定义控制器时或右在上，关闭自定义控制器
        if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.load_mode=LOAD_STOP;
            gimbal_cmd_send.lid_mode=LID_OPEN;
        }
        if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
            gimbal_cmd_send.lid_mode=LID_CLOSE;
        }

        gimbal_cmd_send.yaw += 0.002f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch -= 0.002f * (float)rc_data[TEMP].rc.rocker_l1;

    }
    else
    {
        //自定义控制器
        shoot_cmd_send.friction_mode = (custom_recv_data->key_count[CUSTOM_KEY_COUNT_PRESS][2])%2;
        shoot_cmd_send.load_mode = custom_recv_data->key[CUSTOM_KEY_NOW][1] ? LOAD_BURSTFIRE : LOAD_STOP;
        chassis_cmd_send.vx=8000*custom_recv_data->joystick_x/1700.0;
        chassis_cmd_send.vy=8000*custom_recv_data->joystick_y/1700.0;

        gimbal_cmd_send.pitch = custom_recv_data->pitch ;

    }


    // 云台软件限位,这行之后不能再修改云台角度！！！！
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }


}


/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    uint8_t level=chassis_fetch_data.real_level;
    uint16_t chassis_power_limit=chassis_fetch_data.chassis_power_limit;

    gimbal_cmd_send.gimbal_mode= GIMBAL_GYRO_MODE;
    
    //默认值，防止裁判系统断连时过快
    chassis_speed_mouse=0.4;
    chassis_rotate_speed_mouse=2000;
    chassis_fastrotate_speed_mouse=2500;

    //debug模式，超快
    chassis_speed_mouse=1.0;

    //通过最大能量功率最大速度，都是待测
    if(chassis_power_limit>=40)
    {
        chassis_speed_mouse=0.6;
        chassis_rotate_speed_mouse=2500;
        chassis_fastrotate_speed_mouse=3500;
    }
    if(chassis_power_limit>=60)
    {
        chassis_speed_mouse=0.7;
        chassis_rotate_speed_mouse=3500;
        chassis_fastrotate_speed_mouse=4500;
    }
    if(chassis_power_limit>=70)
    {
        chassis_speed_mouse=0.8;
        chassis_rotate_speed_mouse=3500;
        chassis_fastrotate_speed_mouse=4500;
    }
    if(chassis_power_limit>=90)
    {
        chassis_speed_mouse=0.9;
        chassis_rotate_speed_mouse=4000;
        chassis_fastrotate_speed_mouse=6000;
    }
    if(chassis_power_limit>=100)
    {
        chassis_speed_mouse=1.1;
        chassis_rotate_speed_mouse=4000;
        chassis_fastrotate_speed_mouse=6500;
    }

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w *  chassis_speed_mouse - rc_data[TEMP].key[KEY_PRESS].s * chassis_speed_mouse; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].d * chassis_speed_mouse - rc_data[TEMP].key[KEY_PRESS].a * chassis_speed_mouse;

    //右键开自瞄
    if(rc_data[TEMP].mouse.press_r==0)
    {
        gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 6.0f; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 6.0f;
    }
    else if(rc_data[TEMP].mouse.press_r==1&&
        vision_recv_data->offline!=1&&
        vision_recv_data->target_state==TRACKING)
    {
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
        gimbal_cmd_send.yaw=-vision_recv_data->yaw;
    }


    // 不可以在行代码后面再修改云台目标角度！！
    // 云台软件限位
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }


    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键设置发射模式
    {
    case 0:
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        chassis_cmd_send.load_mode = LOAD_STOP;
        break;
    }

    default:
    {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        chassis_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }

    }
    //if(rc_data[TEMP].mouse.press_l==0 | shoot_fetch_data.shoot_status != SHOOT_STOP)
    if(rc_data[TEMP].mouse.press_l==0)
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        chassis_cmd_send.load_mode = LOAD_STOP;
        chassis_cmd_send.shoot_mode = SHOOT_OFF;
    }
    if(rc_data[TEMP].mouse.press_l==1)
    {
        shoot_cmd_send.shoot_rate=10;
        chassis_cmd_send.shoot_mode = SHOOT_ON;
    }
    // else if(rc_data[TEMP].mouse.press_r==1)
    // {
    //     shoot_cmd_send.shoot_rate=10;
    //     chassis_cmd_send.shoot_mode = SHOOT_ON;
    // }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3)  //Q设置底盘模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW_DIAGONAL;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
    {
        shoot_cmd_send.lid_mode = LID_OPEN;
        chassis_cmd_send.lid_mode = LID_OPEN;
        break;
    }

    default:
    {
        shoot_cmd_send.lid_mode = LID_CLOSE;
        chassis_cmd_send.lid_mode = LID_CLOSE;
        break;
    }

    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        {
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            break;
        }

    default:
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            chassis_cmd_send.friction_mode = FRICTION_ON;
            break;
        }
    }

    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 按shift
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    default:

        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].v) // 按下V小陀螺加速
    {
        case 1:
            chassis_cmd_send.wz = chassis_fastrotate_speed_mouse;
            break;
        default:
            // chassis_cmd_send.wz = 3500.0f + 500.0f * float_constrain(arm_sin_f32(DWT_GetTimeline_s()*3.14*0.85)*arm_sin_f32(DWT_GetTimeline_s()*3.14*0.375),-0.45,0.675);
            chassis_cmd_send.wz = chassis_rotate_speed_mouse;
            break;
    }

}


static void VTMSet()
{
    uint16_t chassis_power_limit=chassis_fetch_data.chassis_power_limit;

    gimbal_cmd_send.gimbal_mode= GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;

    //默认值，防止裁判系统断连时过快
    chassis_speed_mouse=0.4;
    chassis_rotate_speed_mouse=2000;
    chassis_fastrotate_speed_mouse=2500;

    //无接收到底盘功率情况下：
    chassis_speed_mouse=1.5;

    //通过最大能量功率最大速度，都是待测
    if(chassis_power_limit>=40)
    {
        chassis_speed_mouse=0.6;
        chassis_rotate_speed_mouse=2500;
        chassis_fastrotate_speed_mouse=3500;
    }
    if(chassis_power_limit>=60)
    {
        chassis_speed_mouse=0.7;
        chassis_rotate_speed_mouse=3500;
        chassis_fastrotate_speed_mouse=4500;
    }
    if(chassis_power_limit>=70)
    {
        chassis_speed_mouse=0.8;
        chassis_rotate_speed_mouse=3500;
        chassis_fastrotate_speed_mouse=4500;
    }
    if(chassis_power_limit>=90)
    {
        chassis_speed_mouse=0.9;
        chassis_rotate_speed_mouse=4000;
        chassis_fastrotate_speed_mouse=6000;
    }
    if(chassis_power_limit>=100)
    {
        chassis_speed_mouse=1.1;
        chassis_rotate_speed_mouse=4000;
        chassis_fastrotate_speed_mouse=6500;
    }

    chassis_cmd_send.vx = vtm_recv_data->mouse_key_data[TEMP].key[KEY_PRESS].w *  chassis_speed_mouse
        - vtm_recv_data->mouse_key_data[TEMP].key[KEY_PRESS].s * chassis_speed_mouse; // 系数待测
    chassis_cmd_send.vy = vtm_recv_data->mouse_key_data[TEMP].key[KEY_PRESS].d * chassis_speed_mouse
        - vtm_recv_data->mouse_key_data[TEMP].key[KEY_PRESS].a * chassis_speed_mouse;

    //右键开自瞄
    if(vtm_recv_data->mouse_key_data[TEMP].mouse.press_r==0)
    {
        gimbal_cmd_send.yaw += (float)vtm_recv_data->mouse_key_data[TEMP].mouse.x / 660 * 6.0f; // 系数待测
        gimbal_cmd_send.pitch += (float)vtm_recv_data->mouse_key_data[TEMP].mouse.y / 660 * 6.0f;
    }
    else if(vtm_recv_data->mouse_key_data[TEMP].mouse.press_r==1&&
        vision_recv_data->offline!=1&&
        vision_recv_data->target_state==TRACKING)
    {
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
        gimbal_cmd_send.yaw=-vision_recv_data->yaw;
    }


    // 不可以在行代码后面再修改云台目标角度！！
    // 云台软件限位
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }


    switch (vtm_recv_data->mouse_key_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3)  //Q设置底盘模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW_DIAGONAL;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }

    // shift强制小陀螺
    switch (vtm_recv_data->mouse_key_data[TEMP].key[KEY_PRESS].shift) // 按shift
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    default:

        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].v) // 按下V小陀螺加速
    {
    case 1:
        chassis_cmd_send.wz = chassis_fastrotate_speed_mouse;
        break;
    default:
        // chassis_cmd_send.wz = 3500.0f + 500.0f * float_constrain(arm_sin_f32(DWT_GetTimeline_s()*3.14*0.85)*arm_sin_f32(DWT_GetTimeline_s()*3.14*0.375),-0.45,0.675);
            chassis_cmd_send.wz = chassis_rotate_speed_mouse;
        break;
    }

    // 打开测试模式
    switch (vtm_recv_data->mouse_key_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)
    {
    case 0://默认状态：测试模式
        chassis_speed_mouse=0.5;
        chassis_rotate_speed_mouse=2400;
        chassis_fastrotate_speed_mouse=2400;
        break;
    case 1:     // 正式模式
    default:
    break;

    }

    // 发弹设置
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        {
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            break;
        }

    default:
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            chassis_cmd_send.friction_mode = FRICTION_ON;
            break;
        }
    }

    switch(vtm_recv_data->mouse_key_data[TEMP].mouse.press_l)
    {
    case 1:
        shoot_cmd_send.shoot_rate=10;
        chassis_cmd_send.shoot_mode = SHOOT_ON;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_STOP;
        chassis_cmd_send.load_mode = LOAD_STOP;
        chassis_cmd_send.shoot_mode = SHOOT_OFF;
        break;
    }

    // 打开测试模式
    switch (vtm_recv_data->mouse_key_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)
    {
    case 0://默认状态：测试模式
        chassis_speed_mouse=0.8;
        chassis_rotate_speed_mouse=2000;
        chassis_fastrotate_speed_mouse=2000;
        break;
    case 1:     // 正式模式
    default:
    break;

    }

}


//对速度进行处理，包括归一化、小陀螺速度重新分配等
static void SpeedDistribution()
{
    static float v_now, reduction_v;

    v_now = Sqrt(float_Square(chassis_cmd_send.vx)+float_Square(chassis_cmd_send.vy));
    if(v_now!=0)
    {
        reduction_v=(chassis_speed_mouse+chassis_speed_rocker)/v_now;
        if(reduction_v<1)
        {
            chassis_cmd_send.vx *= reduction_v;
            chassis_cmd_send.vy *= reduction_v;
        }
    }
    
    //移动时小陀螺减速
    if(chassis_cmd_send.vx != 0 || chassis_cmd_send.vy != 0)
    {
        if(chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
            chassis_cmd_send.wz *= 0.7f;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
        //AlarmSetStatus(robocmd_alarm, ALARM_OFF);  
    }
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
        //AlarmSetStatus(robocmd_alarm, ALARM_ON);
    }
    //如果云台停止控制，则让yaw轴ref持续更新
    if(gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE )
    {
        gimbal_cmd_send.yaw=GYRO2GIMBAL_DIR_YAW*gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    //清零计数，因为二者只能有一个不为0，用于速度缩放，后续修改方法
    chassis_speed_mouse=0;
    chassis_speed_rocker=0;

    VisionTrajectory();

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    // if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    {
        RemoteControlSet();
    }
    // else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    {
        MouseKeySet();
    }
    // else if(switch_is_mid(rc_data[TEMP].rc.switch_left))
    else if(switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        RemoteShootSet();
    }
    else if(vtm_recv_data->connected_flag == VTM_CONNECTED)
    {
        VTMSet();
    }


    SpeedDistribution();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    VisionSetFlag(chassis_fetch_data.enemy_color,0,chassis_fetch_data.bullet_speed);

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    //@todo:当前数据回传频率受限于CMD任务频率。后面需要分离
    static uint8_t vision_cnt;
    if(++vision_cnt>=2)
    {
        VisionSend(&vision_send_data);
        vision_cnt=0;
    }

}
