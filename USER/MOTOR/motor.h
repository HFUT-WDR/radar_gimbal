/*******************************************************************************
 * @file: motor.c/.h
 * @author: F
 * @date: 2025年10月10日
 * @brief: 电机驱动
 * @note: 调用各种电机库，实现云台及底盘的多电机控制
 *
 * @copyright: Copyright (c) 2025
 ******************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

//各种电机库
#include "dji_2006.h"
#include "dji_3508.h"
#include "dji_6020.h"
#include "dm_motor.h"


#define GIMBAL_CAN  &hcan1



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//rm motor data
typedef struct
{
	const motor_6020_measure_t *motor_yaw_measure;
	const motor_2006_measure_t *motor_trigger_measure;
	const motor_3508_measure_t *motor_fric_measure[2];
	const motor_6020_measure_t *motor_pitch_measure;
//    const dm_motor_measure_t *motor_yaw_measure;
	const motor_3508_measure_t *motor_chassis_measure[4];

} motor_measure_t;


/**
  * @brief          发送电机控制电流
  * @param[in]      yaw: yaw电机控制电流
  * @param[in]      pitch: pitch电机控制电流
  * @param[in]      roll: roll电机控制电流
  * @param[in]      rev: 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t roll, int16_t rev);

/**
  * @brief          发送电机控制电流
  * @param[in]      left: left电机控制电流
  * @param[in]      right: right电机控制电流
  * @param[in]      shoot: shoot电机控制电流
  * @param[in]      rev: 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_shoot(int16_t left, int16_t right, int16_t shoot, int16_t rev2);

/**
  * @brief          发送电机控制电流
  * @param[in]      motor1: motor1电机控制电流
  * @param[in]      motor2: motor2电机控制电流
  * @param[in]      motor3: motor3电机控制电流
  * @param[in]      motor4: motor4电机控制电流
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);



/**
  * @brief          返回yaw电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回pitch电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回trigger电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回shoot电机数据指针
  * @param[in]      i:1代表CAN2，0代表CAN1
	* @param[in]      j:第j个电机
  * @retval         电机数据指针
  */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t i, uint8_t j);

/**
  * @brief          返回shoot电机数据指针
  * @param[in]      i:1代表CAN2，0代表CAN1
	* @param[in]      j:第j个电机
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i, uint8_t j);





#endif //MORTOR_H

