#include "m15motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx = 0;

static M15MotorInstance *m15_motor_instance[M15_MOTOR_CNT] = {NULL};

static CANInstance sender_assignment[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x32, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x33, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan2, .txconf.StdId = 0x32, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x33, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

static uint8_t sender_enable_flag[4] = {0};

static void MotorSenderGrouping(M15MotorInstance *motor, CAN_Init_Config_s *can_config)
{
    uint8_t group;
    uint8_t motor_id = can_config->tx_id - 1;
    motor->message_num  = motor_id % 4;
    can_config->rx_id = 0x96 + motor_id + 1;

    if (can_config->can_handle->Instance == CAN1)
    {
        motor->sender_group = (motor_id / 4) + 0;
    }
    else if (can_config->can_handle->Instance == CAN2)
    {
        motor->sender_group = (motor_id / 4) + 2;
    }
    else
    {
        while (1)
            ;
    }
    sender_enable_flag[motor->sender_group] = 1;
}

static void DecodeM15Motor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    M15MotorInstance *motor = (M15MotorInstance *)_instance->id;
    M15_Motor_Measure_s *measure = &motor->measure;

    measure->last_ecd = measure->ecd;
    measure->speed_aps = M15_ECD2SPEED*(float)((int16_t)(rxbuff[0] << 8 | rxbuff[1]));
    measure->current = ((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->torque_current = M15_ECD2TorqueCurrent * (float)measure->current;
    measure->ecd =( (((uint16_t)rxbuff[4]) << 8) | rxbuff[5]) & 32767;
    measure->angle_single_round = M15_ECD2Angle * (float)measure->ecd;
    
    if(measure->ecd - measure->last_ecd > 16384)
    {
        measure->total_round--;
    }
    else if(measure->ecd - measure->last_ecd < -16384)
    {
        measure->total_round++;
    }
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

void M15MotorStop(M15MotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void M15MotorEnable(M15MotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void M15MotorOuterLoop(M15MotorInstance *motor,Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void M15MotorSetRef(M15MotorInstance *motor,float ref)
{
    motor->motor_controller.pid_ref = ref;
}

M15MotorInstance *M15MotorInit(Motor_Init_Config_s *motor_config)
{
    M15MotorInstance *ins = (M15MotorInstance *)malloc(sizeof(M15MotorInstance));
    memset(ins, 0, sizeof(M15MotorInstance));

    ins->motor_settings = motor_config->controller_setting_init_config;

    PIDInit(&ins->motor_controller.angle_PID, &motor_config->controller_param_init_config.angle_PID);
    PIDInit(&ins->motor_controller.speed_PID, &motor_config->controller_param_init_config.speed_PID);
    PIDInit(&ins->motor_controller.current_PID, &motor_config->controller_param_init_config.current_PID);
    ins->motor_controller.other_angle_feedback_ptr = motor_config->controller_param_init_config.other_angle_feedback_ptr;
    ins->motor_controller.other_speed_feedback_ptr = motor_config->controller_param_init_config.other_angle_feedback_ptr;

    MotorSenderGrouping(ins, &motor_config->can_init_config);

    motor_config->can_init_config.can_module_callback = DecodeM15Motor;
    motor_config->can_init_config.id = ins;
    ins->motor_can_instance = CANRegister(&motor_config->can_init_config);

    M15MotorEnable(ins);
    m15_motor_instance[idx++] = ins;
    return ins;
}

void M15MotorControl()
{
    uint8_t group,num;
    int16_t set;
    M15MotorInstance *motor;
    Motor_Control_Setting_s *motor_settings;
    Motor_Controller_s *motor_controller;
    M15_Motor_Measure_s *measure;
    float pid_measure,pid_ref;
    
    for(size_t i = 0;i<idx;++i)
    {
        motor = m15_motor_instance[i];
        motor_settings = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref;
        
        if (motor_settings->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_settings->close_loop_type & ANGLE_LOOP) && motor_settings->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_settings->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_settings->close_loop_type & SPEED_LOOP) && (motor_settings->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_settings->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_settings->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_settings->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_settings->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->current, pid_ref);
        }

        if (motor_settings->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;

        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[num * 2] = (uint8_t)(set >> 8);
        sender_assignment[group].tx_buff[num * 2 + 1] = (uint8_t)(set);
        
        if(motor->stop_flag == MOTOR_STOP)
        {
            sender_assignment[group].tx_buff[num * 2] = 0;
            sender_assignment[group].tx_buff[num * 2 + 1] = 0;
        }
    }

     for (size_t i = 0; i < 4; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}