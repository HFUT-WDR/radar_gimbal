//低通滤波处理  解决了线性化处理中的抖动问题
#ifndef AUTO_SHOOT_H
#define AUTO_SHOOT_H

#include "struct_typedef.h"
#include "string.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
//#include "GafProjectileSolver.h"

#ifndef PI
#define PI              3.14159265358979f
#endif
#define RX_BUF_NUM 128u
#define YAW_AUTO_SEN 0.05

/* ================== 新增：斜坡和滤波相关定义 ================== */

// 1. 斜坡步进 (限制最大速度)
// 减小此值 -> 整体上升变慢
#define RAMP_MAX_STEP 0.005f

// 2. 滤波系数 (平滑起步和刹车)
// 减小 -> 更柔和
#define LPF_ALPHA 0.15f

/* ====================================================== */

// 斜坡结构体
typedef struct {
    float current_out;
    float max_step;
} Ramp_t;

// 滤波结构体
typedef struct {
    float current_out;
    float alpha;
} LPF_t;

#define IFNX  0

typedef enum {
    RED = 0,
    BLUE
} ROBOT_TEAM;

#define AUTO_SHOOT_FIFO_BUF_LENGTH 1024
#define DATE_LENGTH sizeof(received_packed_t)

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
}__attribute__((__packed__)) send_packed_t;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
}__attribute__((__packed__)) received_packed_t;

typedef struct {
    received_packed_t *p_header;
    uint16_t      data_len;
    uint8_t       protocol_packet[sizeof(received_packed_t)];
    uint8_t       unpack_step;
    uint16_t      index;
} unpack_autoshoot_data_t;

void auto_task(void const * argument);
received_packed_t *get_AUTOshoot_point(void);
uint8_t *get_auto_mode_flag(void);
uint8_t get_auto_fire_state(void);
void AUTO_init(void);
void Auto_Track(gimbal_control_t *gimbal_control_set);
void send_data_to_upper(void);
uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
extern void AutoShoot_USB_Receive(uint8_t* Buf, uint32_t Len);
uint8_t Find_IS_NOT(void);

// 函数声明
float Ramp_Calc(Ramp_t *ramp, float target);
float LPF_Calc(LPF_t *lpf, float target);

#endif


//线性化处理
// #ifndef AUTO_SHOOT_H
// #define AUTO_SHOOT_H
//
// #include "struct_typedef.h"
// #include "string.h"
// #include "cmsis_os.h"
// #include "gimbal_task.h"
// #include "gimbal_behaviour.h"
// //#include "GafProjectileSolver.h"
//
// #ifndef PI
// #define PI              3.14159265358979f
// #endif
// #define RX_BUF_NUM 128u // 最大接收的数据
// #define YAW_AUTO_SEN 0.05
//
// /* ================== 新增：斜坡相关定义 ================== */
// // 斜坡步进最大值，值越小越平滑，响应越慢。
// // 假设控制频率1kHz，0.005 rad/ms ≈ 286度/秒，可根据实际手感调整
// #define RAMP_MAX_STEP 0.005f
//
// typedef struct {
//     float current_out;  // 当前输出值（平滑后的值）
//     float max_step;     // 单次计算允许的最大步进量
// } Ramp_t;
// /* ====================================================== */
//
// #define IFNX  0//判断上位机是否是nx，如果是nx，即为1，则默认是下位机只需要收发数据，不做任何处理
//
// typedef enum {
//     RED = 0,
//     BLUE
// } ROBOT_TEAM;
//
// #define AUTO_SHOOT_FIFO_BUF_LENGTH 1024 // 最大接收的数据
// #define DATE_LENGTH sizeof(received_packed_t) // 有效数据
//
// typedef struct
// {
//     // 帧头和控制信息
//     uint8_t head[2];  // 帧头 "SP"
//     uint8_t mode;     // 模式 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
//     float q[4];       // 四元数 wxyz
//     float yaw;
//     float yaw_vel;
//     float pitch;
//     float pitch_vel;
//     float bullet_speed;
//     uint16_t bullet_count;
//     uint16_t crc16;  // CRC16校验
// }__attribute__((__packed__)) send_packed_t;
//
// typedef struct
// {
//
//     uint8_t head[2];  // 帧头 "SP"
//     uint8_t mode;     // 模式
//     float yaw;        // 目标偏航角
//     float yaw_vel;    // 目标偏航角速度
//     float yaw_acc;    // 目标偏航角加速度
//     float pitch;      // 目标俯仰角
//     float pitch_vel;  // 目标俯仰角速度
//     float pitch_acc;  // 目标俯仰角加速度
//     uint16_t crc16;  // CRC16校验
//
// }__attribute__((__packed__)) received_packed_t;
//
//
// typedef struct {
//     received_packed_t *p_header;
//     uint16_t      data_len;
//     uint8_t       protocol_packet[sizeof(received_packed_t)];
//     uint8_t       unpack_step;
//     uint16_t      index;
// } unpack_autoshoot_data_t;
//
// typedef struct {
//     float range_yaw;
//     float range_pitch;
// } fire_control_t;  // 控制开火YAW，PITCH范围
//
// void auto_task(void const * argument);
// received_packed_t *get_AUTOshoot_point(void);
// uint8_t *get_auto_mode_flag(void);
// uint8_t get_auto_fire_state(void);
// void AUTO_init(void);
// void Auto_Track(gimbal_control_t *gimbal_control_set);
// void send_data_to_upper(void);
// uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
// float lowPassFilter(float input, float prevOutput);
// extern void AutoShoot_USB_Receive(uint8_t* Buf, uint32_t Len);
// uint8_t Find_IS_NOT(void);
//
// // 新增函数声明
// float Ramp_Calc(Ramp_t *ramp, float target);
//
// #endif


//原始代码
// #ifndef AUTO_SHOOT_H
// #define AUTO_SHOOT_H
//
// #include "struct_typedef.h"
// #include "string.h"
// #include "cmsis_os.h"
// #include "gimbal_task.h"
// #include "gimbal_behaviour.h"
// //#include "GafProjectileSolver.h"
//
// #ifndef PI
// #define PI					3.14159265358979f
// #endif
// #define RX_BUF_NUM 128u // 最大接收的数据
// #define YAW_AUTO_SEN 0.05
//
// #define IFNX  0//判断上位机是否是nx，如果是nx，即为1，则默认是下位机只需要收发数据，不做任何处理
//
// typedef enum {
//     RED = 0,
//     BLUE
// } ROBOT_TEAM;
//
// #define AUTO_SHOOT_FIFO_BUF_LENGTH 1024 // 最大接收的数据
// #define DATE_LENGTH sizeof(received_packed_t) // 有效数据
//
// typedef struct
// {
//     // 帧头和控制信息
//     uint8_t head[2];  // 帧头 "SP"
//     uint8_t mode;     // 模式 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
//     float q[4];       // 四元数 wxyz
//     float yaw;
//     float yaw_vel;
//     float pitch;
//     float pitch_vel;
//     float bullet_speed;
//     uint16_t bullet_count;
//     uint16_t crc16;  // CRC16校验
// }__attribute__((__packed__)) send_packed_t;
//
// typedef struct
// {
//
//     uint8_t head[2];  // 帧头 "SP"
//     uint8_t mode;     // 模式
//     float yaw;        // 目标偏航角
//     float yaw_vel;    // 目标偏航角速度
//     float yaw_acc;    // 目标偏航角加速度
//     float pitch;      // 目标俯仰角
//     float pitch_vel;  // 目标俯仰角速度
//     float pitch_acc;  // 目标俯仰角加速度
//     uint16_t crc16;  // CRC16校验
//
// }__attribute__((__packed__)) received_packed_t;
//
//
// typedef struct {
//     received_packed_t *p_header;
//     uint16_t      data_len;
//     uint8_t       protocol_packet[sizeof(received_packed_t)];
//     uint8_t       unpack_step;
//     uint16_t      index;
// } unpack_autoshoot_data_t;
//
// typedef struct {
//     float range_yaw;
//     float range_pitch;
// } fire_control_t;  // 控制开火YAW，PITCH范围
//
// void auto_task(void const * argument);
// received_packed_t *get_AUTOshoot_point(void);
// uint8_t *get_auto_mode_flag(void);
// uint8_t get_auto_fire_state(void);
// void AUTO_init(void);
// void Auto_Track(gimbal_control_t *gimbal_control_set);
// void send_data_to_upper(void);
// uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
// float lowPassFilter(float input, float prevOutput);
// extern void AutoShoot_USB_Receive(uint8_t* Buf, uint32_t Len);
// uint8_t Find_IS_NOT(void);
//
// #endif
//
