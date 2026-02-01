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

#include "dm_motor.h"

extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;

//数组，用于存储4个DM 4310电机的测量数据
dm_motor_measure_t dm_motor_measure[4] = { 0};

/**
 * @brief 解析CAN数据并填充到dm_motor_measure_t结构体中。
 *
 * @param ptr 指向dm_motor_measure_t结构体的指针。
 * @param data 指向包含原始CAN数据的字节数组的指针。
 */
void dm_motor_measure_parse(dm_motor_measure_t* ptr, const uint8_t* data) {
    ptr->error = (data[0] >> 4);                    // 提取高 4 位作为错误码
    ptr->ID = (data[0] & 0xF);                      // 提取低 4 位作为 ID
    if(ptr->error == 0 || ptr->ID == 0){
        return;
    }
    ptr->p_int = (data[1] << 8) | data[2];          // 组合成 16 位位置整数
    ptr->v_int = (data[3] << 4) | (data[4] >> 4);   // 组合成 12 位速度整数
    ptr->t_int = ((data[4] & 0xF) << 8) | data[5];  // 组合成 12 位扭矩整数
    ptr->position = uint_to_float(ptr->p_int, P_MIN, P_MAX, 16); // 转换为浮点数位置，单位：弧度 (rad)
    ptr->velocity = uint_to_float(ptr->v_int, V_MIN, V_MAX, 12); // 转换为浮点数速度，单位：弧度/秒 (rad/s)
    ptr->torque = uint_to_float(ptr->t_int, T_MIN, T_MAX, 12);   // 转换为浮点数扭矩，单位：牛·米 (N·m)
    ptr->T_mos = (float)data[6];                    // MOSFET 温度，单位：摄氏度 (°C)
    ptr->T_motor = (float)data[7];                  // 电机线圈温度，单位：摄氏度 (°C)
}

/**
 * @brief 获取指定索引的dm_motor_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的dm_motor_measure_t结构体的指针。
 */
const dm_motor_measure_t* get_dm_motor_measure_point(uint8_t i)
{
    return &dm_motor_measure[i];
}

/**
 * @brief 发送标准ID的数据帧。
 *
 * @param hcan CAN的句柄。
 * @param ID 数据帧ID。
 * @param pData 数组指针。
 * @param Len 字节数0~8。
 */
uint8_t DM_Motor_SendStdData(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
	
        /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}

/**
 * @brief 使能电机。
 */
void DM_MotorEnable(CAN_HandleTypeDef* hcan, uint8_t index)
{
    uint8_t Data_Enable[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };

#if ISFDCAN
    switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M1_SLAVE_ID, Data_Enable, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M2_SLAVE_ID, Data_Enable, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M3_SLAVE_ID, Data_Enable, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M4_SLAVE_ID, Data_Enable, 8);
            break;
        default:
            break;
    }
#else
		switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, CAN_DM_M1_SLAVE_ID, Data_Enable, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, CAN_DM_M2_SLAVE_ID, Data_Enable, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, CAN_DM_M3_SLAVE_ID, Data_Enable, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, CAN_DM_M4_SLAVE_ID, Data_Enable, 8);
            break;
        default:
            break;
    }
#endif
}

/**
 * @brief 失能电机。
 */
void DM_MotorDisEnable(CAN_HandleTypeDef* hcan, uint8_t index)
{
    uint8_t Data_Failure[8]={ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

#if ISFDCAN
    switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M1_SLAVE_ID, Data_Failure, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M2_SLAVE_ID, Data_Failure, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M3_SLAVE_ID, Data_Failure, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M4_SLAVE_ID, Data_Failure, 8);
            break;
        default:
            break;
    }
#else
switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, CAN_DM_M1_SLAVE_ID, Data_Failure, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, CAN_DM_M2_SLAVE_ID, Data_Failure, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, CAN_DM_M3_SLAVE_ID, Data_Failure, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, CAN_DM_M4_SLAVE_ID, Data_Failure, 8);
            break;
        default:
            break;
    }
#endif
}

/**
 * @brief 设置零点。
 */
void DM_MotorSetZero(CAN_HandleTypeDef* hcan, uint8_t index)
{
    uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
		
#if ISFDCAN
    switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M1_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M2_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M3_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, FDCAN_DM_M4_SLAVE_ID, Data_Save_zero, 8);
            break;
        default:
            break;
    }
#else 
		switch (index)
    {
        case 0:
            DM_Motor_SendStdData(hcan, CAN_DM_M1_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 1:
            DM_Motor_SendStdData(hcan, CAN_DM_M2_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 2:
            DM_Motor_SendStdData(hcan, CAN_DM_M3_SLAVE_ID, Data_Save_zero, 8);
            break;
        case 3:
            DM_Motor_SendStdData(hcan, CAN_DM_M4_SLAVE_ID, Data_Save_zero, 8);
            break;
        default:
            break;
    }
#endif
}


/**
 * @brief MIT模式控制电机。
 *
 * @param hcan CAN的句柄。
 * @param id 数据帧的ID。
 * @param pos 位置给定。
 * @param _vel 速度给定。
 * @param KP 位置比例系数。
 * @param KD 位置微分系数。
 * @param torq 转矩给定值。
 */
void DM_MIT_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	motor_tx_message.StdId=id + MIT_CTRL;
	motor_tx_message.IDE=CAN_ID_STD;
	motor_tx_message.RTR=CAN_RTR_DATA;
	motor_tx_message.DLC=0x08;
	
	motor_can_send_data[0] = (pos_tmp >> 8);
	motor_can_send_data[1] = pos_tmp;
	motor_can_send_data[2] = (vel_tmp >> 4);
	motor_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	motor_can_send_data[4] = kp_tmp;
	motor_can_send_data[5] = (kd_tmp >> 4);
	motor_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	motor_can_send_data[7] = tor_tmp;
	 
	//寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
    }
   }
}

void DM_Motor_Init(CAN_HandleTypeDef* hcan, uint8_t index, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id)
{
    DM_MotorEnable(hcan, index);
    DM_MIT_CtrlMotor(hcan, motor_tx_message, motor_can_send_data, id, 0.0f, 0.0f, Pitch_Kp, Pitch_Kd, 0.0f);
}

/**
 * @brief 位置速度模式控制电机。
 *
 * @param hcan CAN的句柄。
 * @param id 数据帧的ID。
 * @param _pos 位置给定。
 * @param _vel 速度给定。
 */
void DM_PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t id, float _pos, float _vel)
{
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;

	motor_tx_message.StdId=id + PosSpeed_Ctrl;
	motor_tx_message.IDE=CAN_ID_STD;
	motor_tx_message.RTR=CAN_RTR_DATA;
	motor_tx_message.DLC=0x08;

	motor_can_send_data[0] = *pbuf;;
	motor_can_send_data[1] = *(pbuf+1);
	motor_can_send_data[2] = *(pbuf+2);
	motor_can_send_data[3] = *(pbuf+3);
	motor_can_send_data[4] = *vbuf;
	motor_can_send_data[5] = *(vbuf+1);
	motor_can_send_data[6] = *(vbuf+2);
	motor_can_send_data[7] = *(vbuf+3);

	//找到空的发送邮箱，把数据发送出去
	if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}

/**
 * @brief 速度模式控制电机。
 *
 * @param hcan CAN的句柄。
 * @param ID 数据帧的ID。
 * @param _vel 速度给定。
 */
void DM_Speed_CtrlMotor(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef motor_tx_message, uint8_t motor_can_send_data[8], uint16_t ID, float _vel)
{
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    motor_tx_message.StdId = ID + Speed_Ctrl;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x04;

    motor_can_send_data[0] = *vbuf;
    motor_can_send_data[1] = *(vbuf+1);     
    motor_can_send_data[2] = *(vbuf+2);
    motor_can_send_data[3] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
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
void dm_motor_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
    switch (can_id)
    {
        case FDCAN_DM_M1_MASTER_ID:
        case CAN_DM_M1_MASTER_ID:
            dm_motor_measure_parse(&dm_motor_measure[0], rx_data);
//            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            break;
        case FDCAN_DM_M2_MASTER_ID:
        case CAN_DM_M2_MASTER_ID:
            dm_motor_measure_parse(&dm_motor_measure[1], rx_data);
            break;
        case FDCAN_DM_M3_MASTER_ID:
        case CAN_DM_M3_MASTER_ID:
            dm_motor_measure_parse(&dm_motor_measure[2], rx_data);
            break;
        case FDCAN_DM_M4_MASTER_ID:
        case CAN_DM_M4_MASTER_ID:
            dm_motor_measure_parse(&dm_motor_measure[3], rx_data);
            break;
        default:
            break;
    }
}

