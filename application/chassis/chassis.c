/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "stdbool.h"

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "user_lib.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DJIMotorInstance *chassis_motor_instance[4];

/* 用于自旋变速策略的时间变量 */
// static float t;
float wz_LPF_RC=0.35;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float output_zoom_coeff = 1.0f;

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.85,   // 4.5
                .Ki = 0.08,   // 0
                .Kd = 0.0002, // 0
                .IntegralLimit = 2000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {//没啥用
                .Kp = 0.8,  // 0.4
                .Ki = 0.08, // 0
                .Kd = 0,
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {.angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED, .outer_loop_type = SPEED_LOOP,
                                           //.close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                                           .close_loop_type = SPEED_LOOP,
                                           .power_limit_flag = POWER_LIMIT_ON},
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);
    chassis_motor_instance[0] = motor_lf;

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    chassis_motor_instance[1] = motor_rf;

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);
    chassis_motor_instance[2] = motor_lb;

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);
    chassis_motor_instance[3] = motor_rb;

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define VWHEEL_2_DPS(v) (v * REDUCTION_RATIO_WHEEL * RAD_2_DEGREE / RADIUS_WHEEL * 1000.0f)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    // 将线速度（m/s）转换为角速度（dps）
    float vx_dps = VWHEEL_2_DPS(chassis_vx);
    float vy_dps = VWHEEL_2_DPS(chassis_vy);
    //轮子线速度±vx±vy-wz*r
    vt_lf = -vx_dps - vy_dps - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -vx_dps + vy_dps - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = vx_dps - vy_dps - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = vx_dps + vy_dps - chassis_cmd_recv.wz * RB_CENTER;
}

#define COSINE45 0.7071068f
#define SINE45 0.7071068f
#define SECANT45 1 / COSINE45
#define COSECANT45 1 / SINE45
/**
 * @brief 全向轮：计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 *        先正交分解计算出临时变量，再应用到底盘上，最后缩放速度
 *        注意正走（四个轮发力）和斜着走（两轮发力）是不同的
 */
static void OmnidirectionalCalculate()
{
    // 将线速度（m/s）转换为角速度（dps）
    float vx_dps = VWHEEL_2_DPS(chassis_vx);
    float vy_dps = VWHEEL_2_DPS(chassis_vy);
    //注意wz单位是度，*LF_CENTER相当于把wz*LF_CENTER换算成弧度并乘上了半径
    vt_lf =(-vx_dps-vy_dps)*COSECANT45 - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf =(-vx_dps+vy_dps)*COSECANT45 - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb =(vx_dps-vy_dps)*COSECANT45 - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb =(vx_dps+vy_dps)*COSECANT45 - chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 设定底盘电机速度参考值
 *
 */
static void ChassisSetRef()
{
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/*这功率限制使用的临时变量*/

static float chassis_pid_output[4];
static float chassis_pid_totaloutput;

static float chassis_power_limit,chassis_input_power,chassis_power_buffer;//裁判系统获取的功率限制值、当前功率值、当前缓冲能量值
static float chassis_power_max;//计算使用的最大功率值
static float chassis_power_offset = -5; // 功率冗余，可修改

//三个系数
float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
float k1 = 1.26e-07;                      // k1，9.50000043e-08
float k2 = 1.95000013e-07;                // k2
float constant_coefficient = 3.5f;

//标示是否处在缓冲能量低状态
bool isLowBuffer = false;

#define CHASSIS_POWER_COFFICIENT (1 - (float)(120 - 45) / (float)(135 - 50)) // 这个量出现是因为我们的电机阻力较大，导致理论值和实际值相差较大，用于补偿

/**
 * @brief 对功率进行缩放，参考西交利物浦的方案
 *
 */
static void LimitChassisOutput()
{
    chassis_pid_totaloutput = 0;
    chassis_power_limit = referee_data->GameRobotState.chassis_power_limit; // 从裁判系统获取的能量限制

    chassis_input_power = referee_data->PowerHeatData.chassis_power;
    chassis_power_buffer = referee_data->PowerHeatData.chassis_power_buffer;

    if (chassis_power_limit >= 100)
    {
        chassis_power_limit = 100;
    }

    // 根据缓冲能量和当前功率限制，计算最大功率值
    chassis_power_offset = -1 * CHASSIS_POWER_COFFICIENT * (chassis_power_limit)-0;

    chassis_power_max = chassis_power_limit + chassis_power_offset;

    if (isLowBuffer)
    {
        chassis_power_max = chassis_power_max - 25;
        if (chassis_power_buffer >= 55.0f)
        {
            isLowBuffer = false;
        }
    }
    else
    {
        // 缓冲能量判断，如果缓冲能量少，则马上减小功率，减少量待测
        if (chassis_power_buffer < 10.0f)
        {
            isLowBuffer = true;
            chassis_power_max = chassis_power_max - 35;
        }
    }

    // 参考西交利物浦
    for (int i = 0; i < 4; i++)
    {
        chassis_pid_output[i] = toque_coefficient * chassis_motor_instance[i]->measure.speed_rpm * chassis_motor_instance[i]->motor_controller.pid_output + k2 * float_Square(chassis_motor_instance[i]->measure.speed_rpm) + k1 * float_Square(chassis_motor_instance[i]->motor_controller.pid_output) + constant_coefficient;
        if (chassis_pid_output[i] < 0)
        {
            continue;
        }
        else
        {
            chassis_pid_totaloutput += chassis_pid_output[i];
        }
    }

    if (chassis_pid_totaloutput > chassis_power_max) // 超出功率
    {
        output_zoom_coeff = chassis_power_max / chassis_pid_totaloutput;
        for (int i = 0; i < 4; i++)
        {
            chassis_pid_output[i] *= output_zoom_coeff;
            if (chassis_pid_output[i] < 0)
            {
                continue;
            }

            //公式法解力矩功率与实际电机功率的一元二次方程
            float a = k1;
            float b = toque_coefficient * chassis_motor_instance[i]->measure.speed_rpm;
            float c = k2 * float_Square(chassis_motor_instance[i]->measure.speed_rpm) - chassis_pid_output[i] + constant_coefficient;
            // k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

            if (chassis_motor_instance[i]->motor_controller.pid_output > 0)
            {
                float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                DJIMotorSetOutputLimit(chassis_motor_instance[i], abs_limit(temp, 13000));
            }
            else
            {
                float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
                DJIMotorSetOutputLimit(chassis_motor_instance[i], abs_limit(temp, 13000));
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
            DJIMotorSetOutputLimit(chassis_motor_instance[i], chassis_motor_instance[i]->motor_controller.pid_output);
    }
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)

    //用于进行底盘旋转缓加速的真实速度值计算
    chassis_feedback_data.real_wz = (-motor_lf->measure.speed_aps - motor_rf->measure.speed_aps - motor_lb->measure.speed_aps - motor_rb->measure.speed_aps) / (LF_CENTER + RF_CENTER + LB_CENTER + RB_CENTER);
    //  ...
}



/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    static float sin_theta, cos_theta;//计算夹角的sin和cos值

    //底盘旋转缓慢加速dt计算变量
    static uint32_t dt_feet_cnt=0;
    static float wz_dt;
    
    static float last_wz=0;//上一次计算时的底盘速度

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    
    //用于底盘旋转速度缓加
    wz_dt = DWT_GetDeltaT(&dt_feet_cnt);

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出

        // 增加角速度判别，如果先前有一定角速度（阈值需要测）（比如在小陀螺，那么就不要反向转回去，这样机动性会更好）
        if (chassis_cmd_recv.offset_angle * chassis_feedback_data.real_wz < 0 && fabs(chassis_feedback_data.real_wz) >= 2500)
        {
            chassis_cmd_recv.offset_angle -= 360;
        }
        chassis_cmd_recv.wz = -1.0f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW_DIAGONAL:

        // 角度回中直接把offset加个45度即可
        chassis_cmd_recv.offset_angle -= 45;

        // 增加角速度判别，如果先前有一定角速度（阈值需要测）（比如在小陀螺，那么就不要反向转回去，这样机动性会更好）
        if (chassis_cmd_recv.offset_angle * chassis_feedback_data.real_wz < 0 && fabs(chassis_feedback_data.real_wz) >= 2500)
        {
            chassis_cmd_recv.offset_angle -= 360;
        }
        chassis_cmd_recv.wz = -1.15f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;

    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = chassis_cmd_recv.wz * wz_dt / (wz_LPF_RC + wz_dt) +
                  last_wz * wz_LPF_RC / (wz_LPF_RC + wz_dt);

        // 在robot_cmd里更改自旋速度，不在这里设置旋转，这里只做滤波
        break;
    case CHASSIS_NO_DIRECTION:
        chassis_cmd_recv.wz = 0;
        cos_theta = 1;
        sin_theta = 0;
        break;
    default:
        break;
    }
    last_wz=chassis_cmd_recv.wz;//记录上次计算时旋转速度用于滤波

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;
    // 底盘向前为y轴正方向,向左为x轴正方向
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    // MecanumCalculate();
    OmnidirectionalCalculate();

    // 设定底盘闭环参考值
    ChassisSetRef();

    // 底盘功率限制
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 2 : 1;

    chassis_feedback_data.real_level = referee_data->GameRobotState.robot_level;
    chassis_feedback_data.chassis_power_limit = referee_data->GameRobotState.chassis_power_limit;

    // UI数据
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.Chassis_Power_Data.chassis_power_mx = chassis_cmd_recv.super_cap.chassis_power_mx;
    ui_data.load_mode = chassis_cmd_recv.load_mode;
    ui_data.shoot_mode = chassis_cmd_recv.shoot_mode;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}