//
// Created by X. yu on 2025/4/24.
//
/*******************************************************************************
 * @file: dji_2006.c/.h
 * @author: Javen,X.yu
 * @date: 2025年4月24日
 * @brief: DJI 2006电机驱动
 * @note: 2006电机驱动, 支持CAN通信, 电机数据解析, 电机控制
 *
 * @copyright: Copyright (c) 2025
 * @license: MIT
 ******************************************************************************/
#ifndef DJI_2006_H
#define DJI_2006_H
#include "main.h"
#include "struct_typedef.h"

/**
 * @brief 定义电机CAN ID的枚举类型
 */
typedef enum
{
    CAN_2006_ALL_ID_1 = 0x200,  // 前四个ID的2006电机的CAN ID
    CAN_2006_ALL_ID_2 = 0x1FF,  // 后四个ID的2006电机的CAN ID
    CAN_2006_M1_ID = 0x201,     // 电机1的CAN ID
    CAN_2006_M2_ID = 0x202,     // 电机2的CAN ID
    CAN_2006_M3_ID = 0x203,     // 电机3的CAN ID
    CAN_2006_M4_ID = 0x204,     // 电机4的CAN ID
    CAN_2006_M5_ID = 0x205,     // 电机5的CAN ID
    CAN_2006_M6_ID = 0x206,     // 电机6的CAN ID
    CAN_2006_M7_ID = 0x207,     // 电机7的CAN ID
    CAN_2006_M8_ID = 0x208,     // 电机8的CAN ID
} motor_2006_id_e;

/**
 * @brief 定义电机测量数据结构体
 * @note 该结构体用于存储电机的编码器数据、转速、给定电流、温度以及上一次的编码器数据。
 */
typedef struct __attribute__((packed))
{
    uint16_t ecd;               // 编码器数据
    int16_t speed_rpm;          // 电机转速，单位：转/分钟
    int16_t given_current;      // 给定电流值
    uint8_t temperature;        // 电机温度
    uint16_t last_ecd;          // 上一次的编码器数据
} motor_2006_measure_t;

/**
 * @brief 定义电机控制数据结构体
 * @note 该结构体用于存储电机的测量数据指针、加速度、速度、设定速度、给定电流等信息。
 */
typedef struct
{
    const motor_2006_measure_t* motor_2006_measure;  // 指向电机测量数据的常量指针
    fp32 accel;                                      // 电机加速度
    fp32 speed;                                      // 电机当前速度
    fp32 speed_set;                                  // 电机设定速度
    int16_t give_current;                            // 实际给定电流值
} motor_2006_t;

/**
  * @brief          发送电机控制电流（一次可控制四个电机）
  * @param[in]      hcan：CAN的句柄
  * @param[in]      motor_tx_message：定义发送报文
  * @param[in]      motor_can_send_data: 定义发送内容
  * @param[in]      id: 数据帧id
  * @param[in]      motor1: 电机1控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: 电机2控制电流, 范围 [-10000,10000]
  * @param[in]      motor3: 电机3控制电流, 范围 [-10000,10000]
  * @param[in]      motor4: 电机4控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void DJI_2006_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief 获取指定索引的motor_2006_measure_t结构体指针。
 *
 * 该函数返回指向motor_2006_measure数组中指定索引的结构体指针。
 *
 * @param index 数组索引，范围为0到7
 * @return 指向指定索引的motor_2006_measure_t结构体的指针。
 */
const motor_2006_measure_t* get_motor_2006_measure_point(uint8_t index);

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则不进行处理。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_2006_can_callback(uint32_t can_id, const uint8_t* rx_data);
#endif //DJI_2006_H
