//
// Created by 34285 on 2026/6/4.
//

#ifndef MOTOR_DJI_GM6020_H
#define MOTOR_DJI_GM6020_H

#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "alg_PID.h"

/**
 * @brief GM6020无刷电机结构体
 */
typedef struct
{
    Struct_PID PID_Angle;
    Struct_PID PID_Omega;
    Struct_PID PID_Torque;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_RM_Motor_ID RM_Motor_ID;
    uint8_t *CAN_Tx_Data;
    int32_t Encoder_Offset;
    float Omega_Max;

    // 常量 (在Init中赋值)
    uint16_t Encoder_Num_Per_Round;
    uint16_t Output_Max;

    // 内部状态变量
    uint32_t Flag;
    uint32_t Pre_Flag;
    uint16_t Rx_Encoder;
    int16_t Rx_Omega;
    int16_t Rx_Torque;
    uint16_t Rx_Temperature;
    uint16_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;

    // 读变量
    Enum_RM_Motor_Status RM_Motor_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    uint8_t Now_Temperature;

    // 读写变量
    Enum_Control_Method Control_Method;
    float Target_Angle;
    float Target_Omega;
    float Target_Torque;
    float Out;
}Struct_RM_Motor_GM6020;

void DJI_GM6020_Init(CAN_HandleTypeDef *hcan, Struct_RM_Motor_GM6020 *motor, Enum_RM_Motor_ID motor_id,
                     Enum_Control_Method control_method, int32_t encoder_offset, float omega_max);
void DJI_GM6020_Output(Struct_RM_Motor_GM6020 *motor);
uint16_t DJI_GM6020_Get_Output_Max(Struct_RM_Motor_GM6020 *motor);
Enum_RM_Motor_Status DJI_GM6020_Get_CAN_Motor_Status(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Now_Angle(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Now_Omega(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Now_Torque(Struct_RM_Motor_GM6020 *motor);
uint8_t DJI_GM6020_Get_Now_Temperature(Struct_RM_Motor_GM6020 *motor);
Enum_Control_Method DJI_GM6020_Get_Control_Method(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Target_Angle(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Target_Omega(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Target_Torque(Struct_RM_Motor_GM6020 *motor);
float DJI_GM6020_Get_Out(Struct_RM_Motor_GM6020 *motor);
void DJI_GM6020_Set_Control_Method(Struct_RM_Motor_GM6020 *motor, Enum_Control_Method control_method);
void DJI_GM6020_Set_Target_Angle(Struct_RM_Motor_GM6020 *motor, float target_angle);
void DJI_GM6020_Set_Target_Omega(Struct_RM_Motor_GM6020 *motor, float target_omega);
void DJI_GM6020_Set_Target_Torque(Struct_RM_Motor_GM6020 *motor, float target_torque);
void DJI_GM6020_Set_Out(Struct_RM_Motor_GM6020 *motor, float out);
void DJI_GM6020_CAN_RxCpltCallback(Struct_RM_Motor_GM6020 *motor, uint8_t *Rx_Data);
void DJI_GM6020_TIM_Alive_PeriodElapsedCallback(Struct_RM_Motor_GM6020 *motor);
void DJI_GM6020_TIM_PID_PeriodElapsedCallback(Struct_RM_Motor_GM6020 *motor);

#endif //MOTOR_DJI_GM6020_H
