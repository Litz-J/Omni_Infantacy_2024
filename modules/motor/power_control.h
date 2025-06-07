#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

// 功率控制相关参数定义
// 有关功率控制的知识，移步RM论坛搜索功率控制算法。这里参考的是西交利物浦2023开源功率控制。
//三个系数
#define TORQUE_COEFFICIENT (0.0003662109375f)   // (20/16384)*(0.3)，0.3为转矩系数，20/16384为将最大±20A电流映射编码到±16384
#define POWER_COEFFICIENT (0.005452840944712f)  // (187/3591)/9.55，前项为将电机端转速转换为整体电机转速。使用时乘以τ(N·m)ω(rpm)，即可得到“输入功率”
#define DEFAULT_K1 1.26e-07                     // k1
#define DEFAULT_K2 1.45000013e-07               // k2
#define DEFAULT_CONSTANT_COEFFICIENT 3.8f       // 常数项


#endif //POWER_CONTROL_H
