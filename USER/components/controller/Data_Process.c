#include "Data_Process.h" 



/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
	 float span = x_max - x_min;
	 float offset = x_min;
	 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
	 float span = x_max - x_min;
	 float offset = x_min;
	 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


//角速度变为转速
/**
 * @brief  将角速度转换为转速
 * @param  Target_angular_velocity    		角速度
 * @retval Target_rotational_speed      	转速
 */
float rotational_speed_to_rotational_speed(float Target_angular_velocity)   
{                     
		float Target_rotational_speed = 0;
		Target_rotational_speed = Target_angular_velocity / (2 * PI);
		return Target_rotational_speed;
}	     
//转速变为转子转速
/**
 * @brief  将转速转换为转子转速（此函数适用于齿轮传动）
 * @param  Target_rotational_speed      	转速
 * @param  Ratio      	                  齿轮传动比
 * @param  Gear_Ratio      	              电机减速比
 * @retval ecd      	                    转子转速
 */
uint16_t radian_to_ecd(float Target_rotational_speed, float Ratio, float Gear_Ratio)   
{   
		uint16_t ecd = 0;
		ecd = Gear_Ratio * Target_rotational_speed * Ratio;
		return ecd;
}	     

/**
 * @brief  将目标角速度转换为电机转速（此函数适用于齿轮传动）
 * @param  TargetAngularVelocity      目标角速度（rad/s）
 * @param  Ratio                      齿轮传动比
 * @param  GearRatio                  电机减速比
 * @retval MotorSpeed                 电机转速（rpm）
 */
int16_t calculateMotorSpeedFromAngularVelocity(float TargetAngularVelocity, float Ratio, float GearRatio)   
{
    uint16_t MotorSpeed = 0;
    // 将角速度（rad/s）转换为转速（rpm）：乘以 60/(2π)
    float TargetRotationalSpeed = TargetAngularVelocity * (60.0f / (2 * PI));
    // 计算电机转速：目标转速 * 齿轮传动比 * 电机减速比
    MotorSpeed = (int16_t)(GearRatio * TargetRotationalSpeed * Ratio);
    return MotorSpeed;
}


/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          电机编码值规整 0—8191
  * @param[in]      ecd: 电机当前编码
  * @retval         none
  */
void ecd_format(uint16_t ecd)         
{                           
		if ((ecd) > ECD_RANGE)  
		{
			(ecd) -= ECD_RANGE; 
		}
		else if ((ecd) < 0)  
		{
			(ecd) += ECD_RANGE; 
		}			
				
}

/**
 * 将角度值转换为弧度值
 * @param degrees 角度值
 * @return 对应的弧度值
 */
float degrees_to_radians(float degrees) {
    return degrees * PI / 180.0f;
}

/**
 * 将弧度转换为角度
 * @param radians 输入的弧度值
 * @return 对应的角度值
 */
float radians_to_degrees(float radians) {
    return radians * 180.0f / PI;
}


/**
 * @brief  将角度归一化到 [-180°, 180°] 范围内
 * @param  angle_deg 输入的角度值(单位: 度)
 * @return 归一化后的角度值，范围在 [-180.0f, 180.0f] 之间
 * @note   通过循环减去或加上360度，将任意角度值映射到标准范围内
 */
float normalize_angle_degrees(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/**
 * @brief  将弧度归一化到 [-π, π] 范围内
 * @param  angle_rad 输入的弧度值
 * @return 归一化后的弧度值，范围在 [-PI, PI] 之间
 * @note   通过循环减去或加上2π，将任意弧度值映射到标准范围内
 */
// 弧度归一化到 [-π, π] 范围
float normalize_angle_radians(float angle_rad) {
    while (angle_rad > PI) angle_rad -= 2 * PI;
    while (angle_rad < -PI) angle_rad += 2 * PI;
    return angle_rad;
}
