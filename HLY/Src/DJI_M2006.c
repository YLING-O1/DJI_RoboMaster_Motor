//
// Created by 34285 on 2026/6/4.
//

#include "DJI_M2006.h"

/**
 * @brief 初始化M2006电机
 *
 * @param hcan CAN_HandleTypeDef指针
 * @param motor RM_Motor_M2006结构体指针
 * @param motor_id 电机ID
 * @param control_method 电机控制方式
 * @param gearbox_rate 减速箱减速比, 默认为原装减速箱36/1, 如拆去减速箱则该值设为1
 * @param torque_max 最大扭矩, 需根据不同负载测量后赋值
 */
void DJI_M2006_Init(CAN_HandleTypeDef *hcan, Struct_RM_Motor_M2006 *motor, Enum_RM_Motor_ID motor_id,
                    Enum_Control_Method control_method, float gearbox_rate, float torque_max)
{
    if (hcan->Instance==CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance==CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }

    motor->RM_Motor_ID = motor_id;
    motor->Control_Method = control_method;
    motor->Gearbox_Rate = gearbox_rate;
    motor->Torque_Max = torque_max;
    motor->CAN_Tx_Data = Allocate_Tx_Data(hcan, motor_id);

    motor->Encoder_Num_Per_Round = 8192;
    motor->Output_Max = 10000;

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
 * @brief M2006电机数据输出到CAN总线发送缓冲区
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 */
void DJI_M2006_Output(Struct_RM_Motor_M2006 *motor)
{
    if (motor->CAN_Tx_Data)
    {
        motor->CAN_Tx_Data[0] = (int16_t)motor->Out >> 8;
        motor->CAN_Tx_Data[1] = (int16_t)motor->Out;
    }
}

//读取电机参数
/**
 * @brief 获取M2006电机的最大输出值, 该值根据电调型号和负载情况测量后赋值, 需根据实际情况调整
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机最大输出值
 */
uint16_t DJI_M2006_Get_Output_Max(Struct_RM_Motor_M2006 *motor)
{
    return motor->Output_Max;
}

/**
 * @brief 获取M2006电机的状态, 根据接收数据更新的Flag变量判断电机是否在线
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机状态
 */
Enum_RM_Motor_Status DJI_M2006_Get_CAN_Motor_Status(Struct_RM_Motor_M2006 *motor)
{
    return motor->RM_Motor_Status;
}

/**
 * @brief 获取M2006电机当前角度, rad
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机当前角度
 */
float DJI_M2006_Get_Now_Angle(Struct_RM_Motor_M2006 *motor)
{
    return motor->Now_Angle;
}

/**
 * @brief 获取M2006电机当前角速度, rad/s
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机当前角速度
 */
float DJI_M2006_Get_Now_Omega(Struct_RM_Motor_M2006 *motor)
{
    return motor->Now_Omega;
}

/**
 * @brief 获取M2006电机当前扭矩, N*m
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机当前扭矩
 */
float DJI_M2006_Get_Now_Torque(Struct_RM_Motor_M2006 *motor)
{
    return motor->Now_Torque;
}

/**
 * @brief 获取M2006电机当前温度, ℃
 *
 * @param motor RM_Motor_M2006结构体指针
 * @return 电机当前温度
 */
uint8_t DJI_M2006_Get_Now_Temperature(Struct_RM_Motor_M2006 *motor)
{
    return motor->Now_Temperature;
}

/**
 * @brief 获取M2006电机控制方式
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机控制方式
 */
Enum_Control_Method DJI_M2006_Get_Control_Method(Struct_RM_Motor_M2006 *motor)
{
    return motor->Control_Method;
}

/**
 * @brief 获取M2006电机目标角度, rad
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机目标角度
 */
float DJI_M2006_Get_Target_Angle(Struct_RM_Motor_M2006 *motor)
{
    return motor->Target_Angle;
}

/**
 * @brief 获取M2006电机目标角速度, rad/s
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机目标角速度
 */
float DJI_M2006_Get_Target_Omega(Struct_RM_Motor_M2006 *motor)
{
    return motor->Target_Omega;
}
/**
 * @brief 获取M2006电机目标扭矩, N*m
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机目标扭矩
 */
float DJI_M2006_Get_Target_Torque(Struct_RM_Motor_M2006 *motor)
{
    return motor->Target_Torque;
}
/**
 * @brief 获取M2006电机输出值
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @return 电机输出值
 */
float DJI_M2006_Get_Out(Struct_RM_Motor_M2006 *motor)
{
    return motor->Out;
}

//设置电机参数
/**
 * @brief 设置M2006电机控制方式
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param control_method 控制方式
 */
void DJI_M2006_Set_Control_Method(Struct_RM_Motor_M2006 *motor, Enum_Control_Method control_method)
{
    motor->Control_Method = control_method;
}

/**
 * @brief 设置M2006电机目标角度, rad
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param target_angle 目标角度
 */
void DJI_M2006_Set_Target_Angle(Struct_RM_Motor_M2006 *motor, float target_angle)
{
    motor->Target_Angle = target_angle;
}

/**
 * @brief 设置M2006电机目标角速度, rad/s
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param target_omega 目标角速度
 */
void DJI_M2006_Set_Target_Omega(Struct_RM_Motor_M2006 *motor, float target_omega)
{
    motor->Target_Omega = target_omega;
}

/**
 * @brief 设置M2006电机目标扭矩, N*m
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param target_torque 目标扭矩
 */
void DJI_M2006_Set_Target_Torque(Struct_RM_Motor_M2006 *motor, float target_torque)
{
    motor->Target_Torque = target_torque;
}

/**
 * @brief 设置M2006电机输出值
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param out 输出值
 */
void DJI_M2006_Set_Out(Struct_RM_Motor_M2006 *motor, float out)
{
    motor->Out = out;
}

/**
 * @brief M2006电机CAN接收回调函数, 根据接收的数据更新电机状态和参数
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 * @param Rx_Data 接收数据指针
 */
void DJI_M2006_CAN_RxCpltCallback(Struct_RM_Motor_M2006 *motor, uint8_t *Rx_Data)
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

    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round + motor->Rx_Encoder;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * 3.1415926535f /
                        motor->Gearbox_Rate;
    motor->Now_Omega = (float)motor->Rx_Omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Now_Torque = motor->Rx_Torque;
    motor->Now_Temperature = motor->Rx_Temperature;
}

/**
 * @brief M2006电机存活检测回调函数, 根据接收数据更新电机状态并重置PID积分误差
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 */
void DJI_M2006_TIM_Alive_PeriodElapsedCallback(Struct_RM_Motor_M2006 *motor)
{
    //判断该时间段内是否接收过电机数据
    if (motor->Flag == motor->Pre_Flag)
    {
        //电机断开连接
        motor->RM_Motor_Status = RM_Motor_Status_Disable;
        PID_Set_I_Error(&motor->PID_Angle, 0.0f);
        PID_Set_I_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        //电机保持连接
        motor->RM_Motor_Status = RM_Motor_Status_Enable;
    }
    motor->Pre_Flag = motor->Flag;
}

/**
 * @brief M2006电机PID控制回调函数, 根据当前控制方式计算输出值并发送到CAN总线
 *
 * @param motor Struct_RM_Motor_M2006结构体指针
 */
void DJI_M2006_TIM_PID_PeriodElapsedCallback(Struct_RM_Motor_M2006 *motor)
{
    switch (motor->Control_Method)
    {
        case Control_Method_OPENLOOP:
            {

            }
        break;
        //默认闭环扭矩控制
        case Control_Method_TORQUE:
            {
                DJI_M2006_Set_Out(motor, motor->Target_Torque / motor->Torque_Max * motor->Output_Max);
            }
        break;
        case Control_Method_OMEGA:
            {
                PID_Set_Target(&motor->PID_Omega, motor->Target_Omega);
                PID_Set_Now(&motor->PID_Omega, motor->Now_Omega);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);
                DJI_M2006_Set_Out(motor, PID_Get_Out(&motor->PID_Omega));
            }
        break;
        case Control_Method_ANGLE:
            {
                PID_Set_Target(&motor->PID_Angle, motor->Target_Angle);
                PID_Set_Now(&motor->PID_Angle, motor->Now_Angle);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);
                motor->Target_Omega = PID_Get_Out(&motor->PID_Angle);
                PID_Set_Target(&motor->PID_Omega, motor->Target_Omega);
                PID_Set_Now(&motor->PID_Omega, motor->Now_Omega);
                PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);
                DJI_M2006_Set_Out(motor, PID_Get_Out(&motor->PID_Omega));
            }
        break;
        default:
            {
                DJI_M2006_Set_Out(motor, 0.0f);
            }
        break;
        }
        DJI_M2006_Output(motor);
}
