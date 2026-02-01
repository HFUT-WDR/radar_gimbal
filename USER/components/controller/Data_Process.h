#ifndef DATE_PROCESS_H
#define DATE_PROCESS_H

#include "struct_typedef.h"

#ifndef PI
#define PI 3.1415926535f
#endif

//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif


float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float rotational_speed_to_rotational_speed(float Target_angular_velocity);
uint16_t radian_to_ecd(float Target_rotational_speed, float Ratio, float Gear_Ratio);
int16_t calculateMotorSpeedFromAngularVelocity(float TargetAngularVelocity, float Ratio, float GearRatio);
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
void ecd_format(uint16_t ecd);
float degrees_to_radians(float degrees);
float radians_to_degrees(float radians);
float normalize_angle_degrees(float angle_deg);
float normalize_angle_radians(float angle_rad);

#endif
