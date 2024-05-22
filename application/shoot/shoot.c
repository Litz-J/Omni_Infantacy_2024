#include "stdbool.h"

#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"



/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

float fric_v_LPF_RC=0.2;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
//冷却计算，结果1表示冷却中
#define IS_HIBERNATED (hibernate_time + dead_time > DWT_GetTimeline_ms())

static bool is_loadstop=true , is_loadenable=false;

static loader_mode_e ShootModGet(void);

loader_mode_e ShootModGet(void)
{
    static loader_mode_e Last_Mode;
    if(shoot_cmd_recv.load_mode != LOAD_STOP && Last_Mode==LOAD_STOP)
    {
        is_loadstop=false;
    }
    else if(shoot_cmd_recv.load_mode != LOAD_STOP && Last_Mode!=LOAD_STOP)
    {
        
    }
    Last_Mode=shoot_cmd_recv.load_mode;
    return shoot_cmd_recv.load_mode;
}

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 2.1, // 20
                .Ki = 15, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.8, // 0.7
                .Ki = 0.125, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 3, // 10
                .Ki = 50,
                .Kd = 0.1,
                .MaxOut = 50000,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 20000,
                .Derivative_LPF_RC =0.0075,
                .DeadBand=750,
            },
            .speed_PID = {
                .Kp = 5, // 10
                .Ki = 0.800000012, // 1
                .Kd = 0.0250000004,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 3000,
                .MaxOut = 9000,
                .Derivative_LPF_RC = 0.04,
            },
            .current_PID = {
                .Kp = 2, // 0.7
                .Ki = 0.3, // 0.1
                .Kd = 0.00079999998,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 3000,
                .MaxOut = 10000,
                .Derivative_LPF_RC = 0.00125,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

float speedref=0;
float torque2006 ;
float fric_v=0;
//48000,31m/s
//44500,26.7m/s

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    torque2006=loader->measure.real_current/16384.0f*36.0f;


    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
        // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        // return;


    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (ShootModGet())
    {
    // 停止拨盘
    case LOAD_STOP:
        //需要测试这个逻辑行不行，先退弹然后关闭电机输出
        if(is_loadstop==false)//退弹
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE);
            is_loadstop=true;
            is_loadenable=false;  
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 400;
            break;
        }
        // if(!IS_HIBERNATED)
        // {
        //     DJIMotorOuterLoop(loader, CURRENT_LOOP);
        //     DJIMotorStop(loader);
        // }
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        //DJIMotorEnable(loader);
        if(!IS_HIBERNATED)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            if(is_loadenable)
            {
                DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE);   // 控制量增加一发弹丸的角度
            }
            else
            {
                DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE*2);   // 控制量补偿先前退掉一发弹丸的角度，并且增加一发弹丸角度
                is_loadenable=true;
            }
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 1000;                                                                    // 完成1发弹丸发射的时间
        }
        
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        //DJIMotorEnable(loader);
        if(!IS_HIBERNATED)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
            DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE);       // 增加3发
            hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
            dead_time = 500;                                                                        // 完成3发弹丸发射的时间
        }
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        //DJIMotorEnable(loader);
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
        // DJIMotorOuterLoop(loader, CURRENT_LOOP);
        //DJIMotorSetRef(loader, speedref);
        
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        //DJIMotorEnable(loader);
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
        
        
    static uint32_t dt_feet_cnt=0;
    
    static float last_fric_v=0;
    float fric_v_dt = DWT_GetDeltaT(&dt_feet_cnt);

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            fric_v=0;
            break;
        case SMALL_AMU_18:
            fric_v=0;
            break;
        case SMALL_AMU_30:
            fric_v=40000;
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            fric_v=40000;
            break;
        }
        
        shoot_feedback_data.shoot_status=SHOOT_READY;
    }
    else // 关闭摩擦轮
    {
        fric_v=0;
        shoot_feedback_data.shoot_status=SHOOT_STOP;
    }

    fric_v = fric_v * fric_v_dt / (fric_v_LPF_RC + fric_v_dt) +
                  last_fric_v * fric_v_LPF_RC / (fric_v_LPF_RC + fric_v_dt);
    last_fric_v = fric_v;

    DJIMotorSetRef(friction_l, fric_v);
    DJIMotorSetRef(friction_r, fric_v);

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}