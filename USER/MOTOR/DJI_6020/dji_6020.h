/*******************************************************************************
 * @file: dji_6020.c/.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DJI 6020电机驱动头文件
 * @note: 该文件定义了DJI 6020电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/
#ifndef MC02_GENERAL_FRAMEWORK_LITE_DJI_6020_H
#define MC02_GENERAL_FRAMEWORK_LITE_DJI_6020_H

#include "main.h"
#include "struct_typedef.h"
/**
 * @brief 定义电机CAN ID的枚举类型
 */
typedef enum
{
    CAN_6020_ALL_ID_1 = 0x1FF,  // 0x205-0x208电机的CAN ID
    CAN_6020_ALL_ID_2 = 0x2FF,  // 0x209-0x20B电机的CAN ID
    CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,
    CAN_6020_M5_ID = 0x209,
    CAN_6020_M6_ID = 0x20A,
    CAN_6020_M7_ID = 0x20B,

} motor_6020_id_e;

/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
    DJI_6020_MOTOR_RAW = 0,       // 电机原始值控制模式
    DJI_6020_MOTOR_GYRO,          // 电机陀螺仪角度控制模式
    DJI_6020_MOTOR_ENCONDE        // 电机编码器角度控制模式
} motor_6020_mode_e;

/**
 * @brief 定义电机测量数据结构体
 */
typedef struct
{
    uint16_t ecd;               // 编码器数据
    int16_t speed_rpm;          // 电机转速，单位：转/分钟
    int16_t given_current;      // 给定电流值
    uint8_t temperature;        // 电机温度
    int16_t last_ecd;           // 上一次的编码器数据
} motor_6020_measure_t;

/**
  * @brief          发送电机控制电流（一次可控制四个电机）
  * @param[in]      hcan：CAN的句柄
  * @param[in]      motor_tx_message：定义发送报文
  * @param[in]      motor_can_send_data: 定义发送内容
  * @param[in]      id: 数据帧id
  * @param[in]      motor1: 电机1控制电流, 范围 [-30000,30000]
  * @param[in]      motor2: 电机2控制电流, 范围 [-30000,30000]
  * @param[in]      motor3: 电机3控制电流, 范围 [-30000,30000]
  * @param[in]      motor4: 电机4控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void DJI_6020_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief 获取指定索引的motor_6020_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到7。
 * @return 指向指定索引的motor_6020_measure_t结构体的指针。
 */
const motor_6020_measure_t * get_motor_6020_measure_point(uint8_t i);

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_6020_can_callback(uint32_t can_id, const uint8_t* rx_data);

#endif //MC02_GENERAL_FRAMEWORK_LITE_DJI_6020_H
