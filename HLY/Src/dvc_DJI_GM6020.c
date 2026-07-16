//
// Created by 34285 on 2026/6/4.
//

#include "dvc_DJI_GM6020.h"

/**
 * @brief 初始化GM6020电机
 *
 * @param hcan CAN_HandleTypeDef指针
 * @param motor RM_Motor_GM6020结构体指针
 * @param motor_id 电机ID
 * @param control_method 控制方法
 * @param encoder_offset 编码器偏移量
 * @param omega_max 最大角速度
 */
void DJI_GM6020_Init(CAN_HandleTypeDef *hcan, Struct_RM_Motor_GM6020 *motor, Enum_RM_Motor_ID motor_id,
                     Enum_Control_Method control_method, int32_t encoder_offset, float omega_max)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }

    motor->RM_Motor_ID = motor_id;
    motor->Control_Method = control_method;
    motor->Encoder_Offset = encoder_offset;
    motor->Omega_Max = omega_max;
    motor->CAN_Tx_Data = Allocate_Tx_Data(hcan, motor_id);

    // 常量初始化
    motor->Encoder_Num_Per_Round = 8192;
    motor->Output_Max = 16384;

    // 状态变量清零
    motor->Flag = 0;
    motor->Pre_Flag = 0;
    motor->Rx_Encoder = 0;
    motor->Rx_Omega = 0;
    motor->Rx_Torque = 0;
    motor->Rx_Temperature = 0;
    motor->Pre_Encoder = 0;
    motor->Total_Encoder = 0;
    motor->Total_Round = 0;
    motor->RM_Motor_Status = RM_Motor_Status_Disable;
    motor->Now_Angle = 0.0f;
    motor->Now_Omega = 0.0f;
    motor->Now_Torque = 0.0f;
    motor->Now_Temperature = 0;
    motor->Target_Angle = 0.0f;
    motor->Target_Omega = 0.0f;
    motor->Target_Torque = 0.0f;
    motor->Out = 0.0f;
}

/**
 * @brief GM6020电机数据输出到CAN总线发送缓冲区
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 */
void DJI_GM6020_Output(Struct_RM_Motor_GM6020 *motor)
{
    if (motor->CAN_Tx_Data) {
        motor->CAN_Tx_Data[0] = (int16_t)motor->Out >> 8;
        motor->CAN_Tx_Data[1] = (int16_t)motor->Out;
    }
}

/**
 * @brief 获取GM6020电机的最大输出值
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 最大输出值
 */
uint16_t DJI_GM6020_Get_Output_Max(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Output_Max;
}

/**
 * @brief 获取GM6020电机状态
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return GM6020电机状态
 */
Enum_RM_Motor_Status DJI_GM6020_Get_CAN_Motor_Status(Struct_RM_Motor_GM6020 *motor)
{
    return motor->RM_Motor_Status;
}

/**
 * @brief 获取GM6020电机当前角度, rad
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 当前角度, rad
 */
float DJI_GM6020_Get_Now_Angle(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Now_Angle;
}

/**
 * @brief 获取GM6020电机当前角速度, rad/s
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 当前角速度, rad/s
 */
float DJI_GM6020_Get_Now_Omega(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Now_Omega;
}

/**
 * @brief 获取GM6020电机当前扭矩, N*m
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 当前扭矩, N*m
 */
float DJI_GM6020_Get_Now_Torque(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Now_Torque;
}

/**
 * @brief 获取GM6020电机当前温度, ℃
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 当前温度, ℃
 */
uint8_t DJI_GM6020_Get_Now_Temperature(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Now_Temperature;
}

/**
 * @brief 获取电机控制方式
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 电机控制方式
 */
Enum_Control_Method DJI_GM6020_Get_Control_Method(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Control_Method;
}

/**
 * @brief 获取GM6020电机目标角度, rad
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 目标角度, rad
 */
float DJI_GM6020_Get_Target_Angle(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Target_Angle;
}

/**
 * @brief 获取GM6020电机目标角速度, rad/s
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 目标角速度, rad/s
 */
float DJI_GM6020_Get_Target_Omega(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Target_Omega;
}

/**
 * @brief 获取GM6020电机目标扭矩, N*m
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 目标扭矩, N*m
 */
float DJI_GM6020_Get_Target_Torque(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Target_Torque;
}

/**
 * @brief 获取GM6020电机输出值
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @return 输出值
 */
float DJI_GM6020_Get_Out(Struct_RM_Motor_GM6020 *motor)
{
    return motor->Out;
}

/**
 * @brief 设置GM6020电机控制方式
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param control_method 控制方式
 */
void DJI_GM6020_Set_Control_Method(Struct_RM_Motor_GM6020 *motor, Enum_Control_Method control_method)
{
    motor->Control_Method = control_method;
}

/**
 * @brief 设置GM6020电机目标角度, rad
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param target_angle 目标角度, rad
 */
void DJI_GM6020_Set_Target_Angle(Struct_RM_Motor_GM6020 *motor, float target_angle)
{
    motor->Target_Angle = target_angle;
}

/**
 * @brief 设置GM6020电机目标角速度, rad/s
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param target_omega 目标角速度, rad/s
 */
void DJI_GM6020_Set_Target_Omega(Struct_RM_Motor_GM6020 *motor, float target_omega)
{
    motor->Target_Omega = target_omega;
}

/**
 * @brief 设置GM6020电机目标扭矩, N*m
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param target_torque 目标扭矩, N*m
 */
void DJI_GM6020_Set_Target_Torque(Struct_RM_Motor_GM6020 *motor, float target_torque)
{
    motor->Target_Torque = target_torque;
}

/**
 * @brief 设置GM6020电机输出值
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param out 输出值
 */
void DJI_GM6020_Set_Out(Struct_RM_Motor_GM6020 *motor, float out)
{
    motor->Out = out;
}

/**
 * @brief GM6020电机CAN接收完成回调函数, 解析CAN数据并更新电机状态
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 * @param Rx_Data 接收到的CAN数据帧
 */
void DJI_GM6020_CAN_RxCpltCallback(Struct_RM_Motor_GM6020 *motor, uint8_t *Rx_Data)
{
    int16_t delta_encoder;

    motor->Flag++;
    motor->Pre_Encoder = motor->Rx_Encoder;

    motor->Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
    motor->Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
    motor->Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
    motor->Rx_Temperature = Rx_Data[6];

    delta_encoder = motor->Rx_Encoder - motor->Pre_Encoder;
    if (delta_encoder < -4096)
    {
        //正方向转过了一圈
        motor->Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        //反方向转过了一圈
        motor->Total_Round--;
    }

    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round + motor->Rx_Encoder +
                           motor->Encoder_Offset;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * 3.1415926535f;
    motor->Now_Omega = (float)motor->Rx_Omega * RPM_TO_RADPS;
    motor->Now_Torque = motor->Rx_Torque;
    motor->Now_Temperature = motor->Rx_Temperature;
}

/**
 * @brief GM6020电机存活检测回调函数, 根据接收数据更新电机状态并重置PID积分误差
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 */
void DJI_GM6020_TIM_Alive_PeriodElapsedCallback(Struct_RM_Motor_GM6020 *motor)
{
    //判断该时间段内是否接收过电机数据
    if (motor->Flag == motor->Pre_Flag)
    {
        //电机断开连接
        motor->RM_Motor_Status = RM_Motor_Status_Disable;
        PID_Set_I_Error(&motor->PID_Angle, 0.0f);
        PID_Set_I_Error(&motor->PID_Omega, 0.0f);
        PID_Set_I_Error(&motor->PID_Torque, 0.0f);
    }
    else
    {
        //电机保持连接
        motor->RM_Motor_Status = RM_Motor_Status_Enable;
    }
    motor->Pre_Flag = motor->Flag;
}

/**
 * @brief GM6020电机PID控制回调函数, 根据当前控制方式计算输出值并更新CAN发送缓冲区
 *
 * @param motor Struct_RM_Motor_GM6020结构体指针
 */
void DJI_GM6020_TIM_PID_PeriodElapsedCallback(Struct_RM_Motor_GM6020 *motor)
{
    switch (motor->Control_Method)
    {
        //默认开环速度控制
        case Control_Method_OPENLOOP:
            {
                DJI_GM6020_Set_Out(motor, motor->Target_Omega / motor->Omega_Max * motor->Output_Max);
            }
        break;
        //扭矩环
        case Control_Method_TORQUE:
            {
                PID_Set_Target(&motor->PID_Torque, motor->Target_Torque);
                PID_Set_Now(&motor->PID_Torque, motor->Now_Torque);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);
                DJI_GM6020_Set_Out(motor, PID_Get_Out(&motor->PID_Torque));
            }
        break;
        //速度环
        case Control_Method_OMEGA:
            {
                PID_Set_Target(&motor->PID_Omega, motor->Target_Omega);
                PID_Set_Now(&motor->PID_Omega, motor->Now_Omega);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);
                motor->Target_Torque = PID_Get_Out(&motor->PID_Omega);
                PID_Set_Target(&motor->PID_Torque, motor->Target_Torque);
                PID_Set_Now(&motor->PID_Torque, motor->Now_Torque);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);
                DJI_GM6020_Set_Out(motor, PID_Get_Out(&motor->PID_Torque));
            }
        break;
        //角度环
        case Control_Method_ANGLE:
            {
                PID_Set_Target(&motor->PID_Angle, motor->Target_Angle);
                PID_Set_Now(&motor->PID_Angle, motor->Now_Angle);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);
                motor->Target_Omega = PID_Get_Out(&motor->PID_Angle);
                PID_Set_Target(&motor->PID_Omega, motor->Target_Omega);
                PID_Set_Now(&motor->PID_Omega, motor->Now_Omega);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);
                motor->Target_Torque = PID_Get_Out(&motor->PID_Omega);
                PID_Set_Target(&motor->PID_Torque, motor->Target_Torque);
                PID_Set_Now(&motor->PID_Torque, motor->Now_Torque);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);
                DJI_GM6020_Set_Out(motor, PID_Get_Out(&motor->PID_Torque));

            }
        break;
        default:
            {
                DJI_GM6020_Set_Out(motor, 0.0f);
            }
        break;
    }
    DJI_GM6020_Output(motor);
}