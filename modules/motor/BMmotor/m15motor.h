#ifndef M15_MOTOR_H
#define M15_MOTOR_H
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define M15_MOTOR_CNT 8

#define M15_ECD2Angle 0.0109866f// 360/32767,将编码器值转化为角度制
#define M15_ECD2SPEED 0.01f // 速度值范围：-21000 ~ 21000，对应-210RPM ~ 210RPM
#define M15_ECD2TorqueCurrent 0.00167851f//转矩电流值：-32767 ~ 32767，对应-55A ~ 55A

typedef struct 
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t current;           // 电流反馈
    float torque_current;     // 转矩电流
    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} M15_Motor_Measure_s;

typedef struct 
{
    M15_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    float dt;
} M15MotorInstance;

M15MotorInstance *M15MotorInit(Motor_Init_Config_s *motor_config);
void M15MotorStop(M15MotorInstance *motor);
void M15MotorEnable(M15MotorInstance *motor);
void M15MotorOuterLoop(M15MotorInstance *motor,Closeloop_Type_e type);
void M15MotorSetRef(M15MotorInstance *motor,float ref);
void M15MotorControl();

#endif // !

