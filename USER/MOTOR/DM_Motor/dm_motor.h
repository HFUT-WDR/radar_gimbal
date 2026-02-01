//
// Created by X. yu on 2025/4/20.
//
/*******************************************************************************
 * @file: dm_4310.c/.h
 * @author: Javen,X.yu
 * @date: 2025年4月20日
 * @brief: DM 4310电机驱动头文件
 * @note: 该文件定义了DM 4310电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2025
 * @license: MIT
 ******************************************************************************/

#ifndef MC02_GENERAL_FRAMEWORK_LITE_DM_MPTOR_H
#define MC02_GENERAL_FRAMEWORK_LITE_DM_MPTOR_H

#include "main.h"
#include "struct_typedef.h"


#define ISFDCAN  0

/**
 * @brief 定义电机控制参数的取值范围
 * @note 以下内容需要与上位机参数保持一致
 */
#define P_MIN (-3.14f)      // 位置最小值，单位：弧度 (rad)
#define P_MAX 3.14f         // 位置最大值，单位：弧度 (rad)
#define V_MIN (-45.0f)      // 速度最小值，单位：弧度/秒 (rad/s)
#define V_MAX 45.0f         // 速度最大值，单位：弧度/秒 (rad/s)
#define KP_MIN 0.0f         // 位置比例系数最小值
#define KP_MAX 500.0f       // 位置比例系数最大值
#define KD_MIN 0.0f         // 位置微分系数最小值
#define KD_MAX 5.0f         // 位置微分系数最大值
#define T_MIN (-18.0f)      // 转矩最小值，单位：牛·米 (Nm)
#define T_MAX 18.0f         // 转矩最大值，单位：牛·米 (Nm)

#define Pitch_Kp   13  //pitch电机Kp
#define Pitch_Kd   0.3 //pithc电机Kd

#define MIT_CTRL          0x000
#define PosSpeed_Ctrl     0x100
#define Speed_Ctrl        0x200
/**
 * @brief 定义电机CAN ID的枚举类型
 */
typedef enum
{
		//此处为fdcan
    FDCAN_DM_M1_SLAVE_ID = 0x05,   // Pitch电机的CAN从机ID
    FDCAN_DM_M1_MASTER_ID = 0x06,	 // Pitch电机的CAN主控ID
    FDCAN_DM_M2_SLAVE_ID = 0x03,   // 拨弹电机的CANID
    FDCAN_DM_M2_MASTER_ID = 0x08,	 // 拨弹电机的Master_ID
    FDCAN_DM_M3_SLAVE_ID = 0x009,  // 电机3的CAN从机ID
    FDCAN_DM_M3_MASTER_ID = 0x00A, // 电机3的CAN主控ID
    FDCAN_DM_M4_SLAVE_ID = 0x00B,  // 电机4的CAN从机ID
    FDCAN_DM_M4_MASTER_ID = 0x00C, // 电机4的CAN主控ID
	
		//此处为标准can
    CAN_DM_M1_SLAVE_ID = 0x01,         //电机1的CAN从机ID
    CAN_DM_M2_SLAVE_ID = 0x02,         //电机2的CAN从机ID
    CAN_DM_M3_SLAVE_ID = 0x03,         //电机3的CAN从机ID
    CAN_DM_M4_SLAVE_ID = 0x04,         //电机4的CAN从机ID

    CAN_DM_M1_MASTER_ID = 0x11,        //电机1的CAN主控ID
    CAN_DM_M2_MASTER_ID = 0x12,        //电机2的CAN主控ID
    CAN_DM_M3_MASTER_ID = 0x13,        //电机3的CAN主控ID
    CAN_DM_M4_MASTER_ID = 0x14,        //电机4的CAN主控ID
} dm_motor_id_e;

/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
    DM_MOTOR_RAW = 0,       // 电机原始值控制模式
    DM_MOTOR_POSITION,      // 电机位置控制模式
    DM_MOTOR_SPEED          // 电机速度控制模式
} dm_motor_mode_e;

/**
 * @brief 定义电机测量数据结构体
 */
typedef struct {
    uint8_t error;       // 错误码，4 位
    uint8_t ID;          // 电机 ID，4 位
    uint16_t p_int;      // 位置原始数据，16 位
    uint16_t v_int;      // 速度原始数据，12 位
    uint16_t t_int;      // 扭矩原始数据，12 位
    float position;      // 解析后的位置，单位为弧度 (rad)
    float velocity;      // 解析后的速度，单位为弧度每秒 (rad/s)
    float torque;        // 解析后的扭矩，单位为牛顿米 (Nm)
    float T_mos;         // MOSFET 温度，单位为摄氏度 (°C)
    float T_motor;       // 电机线圈温度，单位为摄氏度 (°C)
} dm_motor_measure_t;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct
{
    const dm_motor_measure_t* motor_measure;  // 指向电机测量数据的常量指针

    dm_motor_mode_e motor_mode;               // 当前电机控制模式
    dm_motor_mode_e last_motor_mode;          // 上一次电机控制模式

    fp32 position_set;                       // 设定位置，单位：弧度 (rad)
    fp32 speed_set;                          // 设定速度，单位：弧度/秒 (rad/s)
    fp32 current_set;                        // 电流设定值
    int16_t given_current;                   // 实际给定电流值
} dm_motor_t;

/**
 * @brief 获取指定索引的dm_motor_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的dm_motor_measure_t结构体的指针。
 */
const dm_motor_measure_t* get_dm_motor_measure_point(uint8_t i);


/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则调用错误处理函数。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void dm_motor_can_callback(uint32_t can_id, const uint8_t* rx_data);
	
/**
 * @brief 发送标准ID的数据帧。
 *
 * @param hCAN CAN的句柄。
 * @param ID 数据帧ID。
 * @param pData 数组指针。
 * @param Len 字节数0~8。
 */
uint8_t DM_Motor_SendStdData(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t *pData, uint16_t Len);

/**
 * @brief 使能电机。
 */
void DM_MotorEnable(CAN_HandleTypeDef* hcan, uint8_t index);

/**
 * @brief 失能电机。
 */
void DM_MotorDisEnable(CAN_HandleTypeDef* hcan, uint8_t index);

/**
 * @brief 设置零点。
 */
void DM_MotorSetZero(CAN_HandleTypeDef* hcan, uint8_t index);

/**
 * @brief MIT模式控制电机。
 *
 * @param hCAN CAN的句柄。
 * @param id 数据帧的ID。
 * @param pos 位置给定。
 * @param vel 速度给定。
 * @param KP 位置比例系数。
 * @param KD 位置微分系数。
 * @param torq 转矩给定值。
 */
void DM_MIT_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);

void DM_Motor_Init(CAN_HandleTypeDef* hcan, uint8_t index, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id);


/**
 * @brief 位置速度模式控制电机。
 *
 * @param hCAN CAN的句柄。
 * @param id 数据帧的ID。
 * @param _pos 位置给定。
 * @param _vel 速度给定。
 */
void DM_PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, float _pos, float _vel);

/**
 * @brief 速度模式控制电机。
 *
 * @param hCAN CAN的句柄。
 * @param ID 数据帧的ID。
 * @param _vel 速度给定。
 */
void DM_Speed_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t ID, float _vel);


#endif //MC02_GENERAL_FRAMEWORK_LITE_DM_MOTOR_H
