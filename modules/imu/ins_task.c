/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "user_lib.h"
#include "general_def.h"
#include "master_process.h"
#include "bsp_gpio.h"
#include "task.h"

#include "robot_task.h" //@todo 不应该包括 但是我小小的越一下级 没想到好办法
BMI088Instance *bmi088_test; 
IST8310Instance *ist8310_test;
GPIOInstance *SoftIntgpio_test;

INS_t INS = {0};
static IMU_Param_t IMU_Param;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    BMI088_Data_t raw_data;
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        bmi088_test->spi_acc->spi_work_mode = SPI_BLOCK_MODE;
        bmi088_test->spi_gyro->spi_work_mode = SPI_BLOCK_MODE;
        BMI088Acquire(bmi088_test, &raw_data);
        acc_init[X] += raw_data.acc[X];
        acc_init[Y] += raw_data.acc[Y];
        acc_init[Z] += raw_data.acc[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}
static void SoftIntCallback(GPIOInstance *gpio)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(insTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
}
attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;
    BMI088_Init_Config_s bmi088_config = {
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .work_mode = BMI088_BLOCK_TRIGGER_MODE,
        .spi_acc_config = {
            .spi_handle = &hspi1,
            .GPIOx = GPIOA,
            .cs_pin = GPIO_PIN_4,
            .spi_work_mode = SPI_DMA_MODE,
        },
        .acc_int_config = {
            .GPIOx = GPIOC,
            .GPIO_Pin = GPIO_PIN_4,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
        },
        .spi_gyro_config = {
            .spi_handle = &hspi1,
            .GPIOx = GPIOB,
            .cs_pin = GPIO_PIN_0,
            .spi_work_mode = SPI_DMA_MODE,
        },
        .gyro_int_config = {
            .GPIO_Pin = GPIO_PIN_5,
            .GPIOx = GPIOC,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
        },
        .heat_pwm_config = {
            .htim = &htim10,
            .channel = TIM_CHANNEL_1,
            .period = 1,
        },
        .heat_pid_config = {
            .Kp = 0.5,
            .Ki = 0,
            .Kd = 0,
            .DeadBand = 0.1,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 100,
            .MaxOut = 100,
        },
    };
    bmi088_test = BMI088Register(&bmi088_config);
    IST8310_Init_Config_s ist8310_conf = {
        .gpio_conf_exti = {
            .exti_mode = GPIO_EXTI_MODE_RISING,
            .GPIO_Pin = GPIO_PIN_3,
            .GPIOx = GPIOG,
            .gpio_model_callback = NULL,
        },
        .gpio_conf_rst = {
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .GPIO_Pin = GPIO_PIN_6,
            .GPIOx = GPIOG,
            .gpio_model_callback = NULL,
        },
        .iic_config = {
            .handle = &hi2c3,
            .dev_address = IST8310_IIC_ADDRESS,
            .work_mode = IIC_BLOCK_MODE,
        },
    };
    ist8310_test = IST8310Init(&ist8310_conf);
    GPIO_Init_Config_s SoftIntgpio_conf = {
        .exti_mode = GPIO_EXTI_MODE_FALLING,
        .GPIO_Pin = GPIO_PIN_0,
        .GPIOx = GPIOG,
        .gpio_model_callback = SoftIntCallback,
    };
    SoftIntgpio_test = GPIORegister(&SoftIntgpio_conf);

   
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
bmi088_test->spi_acc->spi_work_mode = SPI_DMA_MODE;
        bmi088_test->spi_gyro->spi_work_mode = SPI_DMA_MODE;
    return (attitude_t *)&INS.Gyro;
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    INS.Accel[X] = bmi088_test->acc[X];
    INS.Accel[Y] = bmi088_test->acc[Y];
    INS.Accel[Z] = bmi088_test->acc[Z];
    INS.Gyro[X] = bmi088_test->gyro[X];
    INS.Gyro[Y] = bmi088_test->gyro[Y];
    INS.Gyro[Z] = bmi088_test->gyro[Z];

    // demo function,用于修正安装误差,可以不管,本demo暂时没用
    //IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
