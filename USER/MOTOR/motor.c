/*******************************************************************************
 * @file: motor.c/.h
 * @author: F
 * @date: 2025年10月10日
 * @brief: 电机驱动
 * @note: 调用各种电机库，实现云台及底盘的多电机控制
 *
 * @copyright: Copyright (c) 2025
 ******************************************************************************/
 
 #include "motor.h"
 #include "cmsis_os.h"


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


static motor_measure_t gimbal_motor_measure;
static motor_measure_t shoot_motor_measure;

static motor_measure_t chassis_motor_measure;

/***************此处须根据真实电机配置来修改ID****************/
motor_6020_id_e motor_6020_id = CAN_6020_ALL_ID_1;
motor_6020_id_e yaw_motor_id = CAN_6020_M1_ID;
motor_6020_id_e pitch_motor_id = CAN_6020_M2_ID;

/***************此处须根据真实电机配置来修改代码****************/
/**
  * @brief          发送电机控制电流
  * @param[in]      yaw: yaw电机控制电流
  * @param[in]      pitch: pitch电机控制电流
  * @param[in]      roll: roll电机控制电流
  * @param[in]      rev: 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t roll, int16_t rev)   // float pitch
{
	//现配置为yaw：6020，pitch：6020
	DJI_6020_CtrlMotor(&hcan1, gimbal_tx_message, gimbal_can_send_data, motor_6020_id, yaw, pitch, 0, 0);
}


/***************以下须根据真实电机配置使用对应电机函数指向YAW、PITCH、SHOOT等电机数据****************/
/**
  * @brief          返回yaw电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
 const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
 {
     gimbal_motor_measure.motor_yaw_measure = get_motor_6020_measure_point(0);

     return &gimbal_motor_measure;
 }

/**
  * @brief          返回pitch电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    gimbal_motor_measure.motor_pitch_measure = get_motor_6020_measure_point(1);
	
    return &gimbal_motor_measure;
}






