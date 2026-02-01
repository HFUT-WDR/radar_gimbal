/*******************************************************************************
 * @file: dji_6020.c/.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DJI 6020电机驱动
 * @note: 该文件实现了DJI 6020电机的CAN通信解析和数据处理功能。
 *        支持8个电机的数据解析，包括编码器计数、转速、电流和温度。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/
#include "dji_6020.h"

//数组，用于存储8个DJI 6020电机的测量数据
motor_6020_measure_t motor_6020_measure[8] = {0};

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
void DJI_6020_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;
    motor_can_send_data[0] = (motor1 >> 8);
    motor_can_send_data[1] = motor1;
    motor_can_send_data[2] = (motor2 >> 8);
    motor_can_send_data[3] = motor2;
    motor_can_send_data[4] = (motor3 >> 8);
    motor_can_send_data[5] = motor3;
    motor_can_send_data[6] = (motor4 >> 8);
    motor_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

/**
 * @brief 解析CAN数据并填充到motor_6020_measure_t结构体中。
 *
 * 该函数接收一个指向motor_6020_measure_t结构体的指针和一个包含原始CAN数据的字节数组。
 * 从数据中提取编码器计数（ecd）、转速（RPM）、给定电流和温度，并将其存储到结构体中。
 *
 * @param ptr 指向motor_6020_measure_t结构体的指针，用于存储解析后的数据。
 * @param data 指向包含原始CAN数据的字节数组的指针。
 */
void motor_6020_measure_parse(motor_6020_measure_t *ptr, const uint8_t *data){
    ptr->last_ecd = (int16_t)ptr->ecd; // 存储上一次的编码器计数
    ptr->ecd = (uint16_t)((data[0] << 8) | data[1]); // 提取当前编码器计数
    ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]); // 提取转速（RPM） NOLINT(*-narrowing-conversions)
    ptr->given_current = (uint16_t)((data[4] << 8) | data[5]); // 提取给定电流 NOLINT(*-narrowing-conversions)
    ptr->temperature = data[6]; // 提取温度
}

/**
 * @brief 获取指定索引的motor_6020_measure_t结构体指针。
 *
 * 该函数返回指向motor_6020_measure数组中指定索引的结构体指针。
 *
 * @param i 数组索引，范围为0到7。
 * @return 指向指定索引的motor_6020_measure_t结构体的指针。
 */
const motor_6020_measure_t * get_motor_6020_measure_point(uint8_t i){
    return &motor_6020_measure[i];
}

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则调用错误处理函数。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_6020_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
    switch (can_id)
    {
        case CAN_6020_M1_ID:
            motor_6020_measure_parse(&motor_6020_measure[0], rx_data); // 解析Yaw电机的数据
//            detect_hook(YAW_GIMBAL_MOTOR_TOE); // 调用检测钩子函数
            break;
        case CAN_6020_M2_ID:
            motor_6020_measure_parse(&motor_6020_measure[1], rx_data); // 解析Pitch电机的数据
//            detect_hook(PITCH_GIMBAL_MOTOR_TOE); // 调用检测钩子函数
            break;
        default:
            break;
    }
}

