/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


// 头文件内容（函数声明、结构体定义等）
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "referee.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "dm_imu_l1.h"
#include "motor.h"


static uint8_t              referee_data_can_send_data1[8];
static CAN_TxHeaderTypeDef  referee_data_message1;
static uint8_t              referee_data_can_send_data2[8];
static CAN_TxHeaderTypeDef  referee_data_message2;

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(hcan ->Instance == CAN1)
		{
			//dm电机的callback
			//dm_motor_can_callback(rx_header.StdId, (const uint8_t*)rx_data);
			//dji3508的回调
			//shoot_3508_can_callback(rx_header.StdId, (const uint8_t*)rx_data);
			//dji2006的回调
			//motor_2006_can_callback(rx_header.StdId, (const uint8_t*)rx_data);
			//dji6020的回调
			motor_6020_can_callback(rx_header.StdId, (const uint8_t*)rx_data);
            //外置imu的回调
            CAN_DM_imu_callback(DM_IMU1_MASTER_ID, (const uint8_t*)rx_data);
			
		}
		else if(hcan ->Instance == CAN2)
		{
			//dji3508的回调
			motor_3508_can_callback(rx_header.StdId, (const uint8_t*)rx_data);
		}
}

/**
* @brief          发送控制裁判数据1
* @param[in]      power:底盘功率
* @param[in]      buffer：缓冲功率
  * @retval         none
  */
void CAN_cmd_referee_data1(fp32 power, fp32 buffer)
{
    uint32_t send_mail_box;
    referee_data_message1.StdId = REFREE_ID1;
    referee_data_message1.IDE = CAN_ID_STD;
    referee_data_message1.RTR = CAN_RTR_DATA;
    referee_data_message1.DLC = 0x08;
	for(uint8_t i = 0; i < 4; i++)
	{
		referee_data_can_send_data1[i] = *((uint8_t *)(&power) + i);
		referee_data_can_send_data1[i+4] = *((uint8_t *)(&buffer) + i);
	}
    HAL_CAN_AddTxMessage(&hcan2, &referee_data_message1, referee_data_can_send_data1, &send_mail_box);
}

/**
* @brief          发送控制裁判数据2
* @param[in]      power:底盘功率
  * @retval         none
  */
void CAN_cmd_referee_data2(uint16_t power_limit , uint8_t high_speed_flag, uint8_t auto_mode)
{
    uint32_t send_mail_box;
    referee_data_message2.StdId = REFREE_ID2;
    referee_data_message2.IDE = CAN_ID_STD;
    referee_data_message2.RTR = CAN_RTR_DATA;
    referee_data_message2.DLC = 0x08;
		for(uint8_t i = 0; i < 2; i++)
		{
			referee_data_can_send_data2[i] = *((uint8_t *)(&power_limit) + i);
		}
		referee_data_can_send_data2[2] = high_speed_flag;
		referee_data_can_send_data2[3] = auto_mode;
		for(uint8_t i = 0; i < 4; i++) //保留
		{
			referee_data_can_send_data2[i+4] = 0;
		}
	
    HAL_CAN_AddTxMessage(&hcan1, &referee_data_message2, referee_data_can_send_data2, &send_mail_box);
}
