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
// bsp
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
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static PIDInstance *pid_pitch_vision,*pid_yaw_vision;

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

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

    //定义自瞄PID
    PID_Init_Config_s pid_pitch_vision_config=
    {
        .Kp = 0.000599999796, // 4.5
        .Ki = 0.00135000004,  // 0
        .Kd = 0.0,  // 0
        .IntegralLimit = 0.6,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
        .MaxOut = 1,
        .DeadBand=3,
    },
    pid_yaw_vision_config=
    {
        .Kp = 0.000669999979, // 4.5
        .Ki = 0.00124999997,  // 0
        .Kd = 0.0,  // 0
        .IntegralLimit = 10,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
        .MaxOut = 20,
        .DeadBand=10,
    };
    pid_pitch_vision=malloc(sizeof(PIDInstance));
    pid_yaw_vision=malloc(sizeof(PIDInstance));
    PIDInit(pid_pitch_vision,&pid_pitch_vision_config);
    PIDInit(pid_yaw_vision,&pid_yaw_vision_config);

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
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
        
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode=LOAD_1_BULLET;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 跟随
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode=LOAD_STOP;
        
    }
    //普通模式，不自瞄调试就用这段
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    //     if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],小陀螺
    // {
    //     chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
        
    // }
    // else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    // {
    //     chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // }
    // else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 跟随
    // {
    //     chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
        
    // }
    float pitch_offset=-70;
    float yaw_offset=0;
    if(vision_recv_data->pitch==0 && pid_pitch_vision->Last_Measure==0)
    {
        pitch_offset=0;
    }
    else
    {
        pitch_offset=-70;
    }
    PIDCalculate(pid_pitch_vision,vision_recv_data->pitch,pitch_offset);
    PIDCalculate(pid_yaw_vision,vision_recv_data->yaw,yaw_offset);
    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
        gimbal_cmd_send.yaw += 0.002f * (float)rc_data[TEMP].rc.rocker_l_+pid_yaw_vision->Output;
        gimbal_cmd_send.pitch -= 0.002f * (float)rc_data[TEMP].rc.rocker_l1-pid_pitch_vision->Output;
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw += 0.003f * (float)rc_data[TEMP].rc.rocker_l_-pid_yaw_vision->Output;
        gimbal_cmd_send.pitch -= 0.003f * (float)rc_data[TEMP].rc.rocker_l1-pid_pitch_vision->Output;
    }

    // 云台软件限位
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 35.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 35.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向

    // if(switch_is_down(rc_data[TEMP].rc.switch_left)&&shoot_cmd_send.friction_mode == FRICTION_ON)
    //     {
    //             shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    //     }
    //         else
    //         {
    //             shoot_cmd_send.load_mode = LOAD_STOP;
    //     }


    // // 发射参数
    // if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
    // {
    //     //shoot_cmd_send.load_mode = LOAD_BURSTFIRE;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    // }
    // else
    // {
    //     //shoot_cmd_send.load_mode = LOAD_STOP;
    //     // 弹舱舵机控制,待添加servo_motor模块,关闭

    // }
        
    // 摩擦轮控制,拨轮向上打为负,向下为正
    // if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    // else
    //     ;
        //shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    // if (rc_data[TEMP].rc.dial < -500)
    //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    // else
    //     ;
    //     //shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 15;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * 12000 - rc_data[TEMP].key[KEY_PRESS].s * 12000; // 系数待测
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 12000 - rc_data[TEMP].key[KEY_PRESS].d * 12000;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    // 云台软件限位
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 30;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 30;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3)  
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 10) // C键设置底盘功率
    {//血量优先
    case 0:
        chassis_cmd_send.robot_real_level = 1;
        break;
    case 1:
        chassis_cmd_send.robot_real_level = 2;
        break;
    case 2:
        chassis_cmd_send.robot_real_level = 3;
        break;
    case 3:
        chassis_cmd_send.robot_real_level = 4;
        break;
    case 4:
        chassis_cmd_send.robot_real_level = 5;
        break;
    case 5:
        chassis_cmd_send.robot_real_level = 6;
        break;
    case 6:
        chassis_cmd_send.robot_real_level = 7;
        break;
    case 7:
        chassis_cmd_send.robot_real_level = 8;
        break;
    case 8:
        chassis_cmd_send.robot_real_level = 9;
        break;
    default:
        chassis_cmd_send.robot_real_level = 10;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 2) // X键降低底盘功率为前一级
    {
    case 1:
        {
            if(rc_data[TEMP].key_count[KEY_PRESS][Key_C] > 0)
            {
                rc_data[TEMP].key_count[KEY_PRESS][Key_C]--;
                rc_data[TEMP].key_count[KEY_PRESS][Key_X]++;
            }
            
        } 
        break;
    default:
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
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
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
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
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

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
    //VisionSend(&vision_send_data);
}
