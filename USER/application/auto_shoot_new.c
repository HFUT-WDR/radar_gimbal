#include "auto_shoot_new.h"
#include "referee.h"
#include "arm_math.h"
#include "user_lib.h"
#include "fifo.h"
#include "usbd_cdc_if.h"
#include "INS_task.h"

extern ext_game_robot_state_t robot_state;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;

received_packed_t received_packed;
send_packed_t send_packed;

fifo_s_t auto_shoot_fifo;
uint8_t auto_shoot_fifo_buf[AUTO_SHOOT_FIFO_BUF_LENGTH];
unpack_autoshoot_data_t autoshoot_unpack_obj;
uint8_t flag_received = 0;
uint8_t frame, frame_count = 0;

const uint16_t CRC_INIT_AUTO = 0xffff;

// ================== 定义两套变量 (Yaw/Pitch 各一套) ==================
static Ramp_t yaw_ramp = {0.0f, RAMP_MAX_STEP};
static LPF_t  yaw_lpf  = {0.0f, LPF_ALPHA};

static Ramp_t pitch_ramp = {0.0f, RAMP_MAX_STEP};
static LPF_t  pitch_lpf  = {0.0f, LPF_ALPHA};
// ==================================================================

void AUTO_init(void) {
    fifo_s_init(&auto_shoot_fifo, auto_shoot_fifo_buf, AUTO_SHOOT_FIFO_BUF_LENGTH);
    memset(&received_packed, 0, sizeof(received_packed_t));
    memset(&send_packed, 0, sizeof(send_packed_t));

    // 初始化参数
    yaw_ramp.max_step = RAMP_MAX_STEP;
    yaw_lpf.alpha = LPF_ALPHA;

    pitch_ramp.max_step = RAMP_MAX_STEP;
    pitch_lpf.alpha = LPF_ALPHA;
}

// === 1. 斜坡计算 ===
float Ramp_Calc(Ramp_t *ramp, float target) {
    float err = target - ramp->current_out;
    if (err > ramp->max_step) {
        ramp->current_out += ramp->max_step;
    } else if (err < -ramp->max_step) {
        ramp->current_out -= ramp->max_step;
    } else {
        ramp->current_out = target;
    }
    return ramp->current_out;
}

// === 2. 滤波计算 ===
float LPF_Calc(LPF_t *lpf, float target) {
    lpf->current_out = lpf->current_out * (1.0f - lpf->alpha) + target * lpf->alpha;
    return lpf->current_out;
}

void autoshoot_unpack_fifo_data(void) {
    static uint8_t state = 0;
    static uint16_t bytes_received = 0;
    unpack_autoshoot_data_t *p_obj = &autoshoot_unpack_obj;
    uint16_t data_len = sizeof(received_packed_t);

    while (fifo_s_used(&auto_shoot_fifo) > 0) {
        uint8_t byte = fifo_s_get(&auto_shoot_fifo);
        if (state == 0) {
            if (byte == 'S') {
                p_obj->protocol_packet[0] = byte;
                bytes_received = 1;
                state = 1;
            }
        } else if (state == 1) {
            state = (byte == 'P') ? 2 : (byte == 'S') ? 1 : 0;
            if (byte == 'P') {
                p_obj->protocol_packet[1] = byte;
                bytes_received = 2;
            } else if (byte == 'S') {
                p_obj->protocol_packet[0] = byte;
                bytes_received = 1;
            } else {
                bytes_received = 0;
            }
        } else {
            p_obj->protocol_packet[bytes_received++] = byte;
            if (bytes_received == data_len) {
                uint16_t received_crc = (p_obj->protocol_packet[data_len - 1] << 8) |
                                        p_obj->protocol_packet[data_len - 2];
                uint16_t calculated_crc = Get_CRC16_Check_Sum_Auto(p_obj->protocol_packet,
                                                                   data_len - 2,
                                                                   CRC_INIT_AUTO);
                if (calculated_crc == received_crc) {
                    memcpy(&received_packed, p_obj->protocol_packet, data_len);
                    flag_received = 1;
                    frame_count++;
                }
                state = bytes_received = 0;
            }
        }
    }
}

void auto_task(void const *argument) {
    AUTO_init();
    while (1) {
        autoshoot_unpack_fifo_data();
        send_data_to_upper();
        vTaskDelay(10);
    }
}

void send_data_to_upper(void) {
    uint8_t send_buffer[sizeof(send_packed_t)] = {0};
    const uint8_t head[2] = {'S', 'P'};
    memcpy(send_buffer + 0, head, 2);
    const uint8_t mode = (gimbal_behaviour == GIMBAL_AUTO) ? 1 : 0;
    memcpy(send_buffer + 2, &mode, 1);
    const fp32* quat_data =  get_INS_quat_point();
    memcpy(send_buffer + 3, quat_data, 16);
    const fp32 yaw_data = gimbal_control.gimbal_yaw_motor.absolute_angle;
    const fp32 yaw_vel_data = gimbal_control.gimbal_yaw_motor.motor_gyro;
    const fp32 pitch_data = gimbal_control.gimbal_pitch_motor.absolute_angle;
    const fp32 pitch_vel_data = gimbal_control.gimbal_pitch_motor.motor_gyro;
    memcpy(send_buffer + 19, &yaw_data, 4);
    memcpy(send_buffer + 23, &yaw_vel_data, 4);
    memcpy(send_buffer + 27, &pitch_data, 4);
    memcpy(send_buffer + 31, &pitch_vel_data, 4);
    const fp32 bullet_speed_data = 15.0f;
    const uint16_t bullet_count_data = 0;
    memcpy(send_buffer + 35, &bullet_speed_data, 4);
    memcpy(send_buffer + 39, &bullet_count_data, 2);
    const uint32_t crc_length = sizeof(send_packed_t) - 2;
    const uint16_t crc_data = Get_CRC16_Check_Sum_Auto(send_buffer, crc_length, CRC_INIT_AUTO);
    memcpy(send_buffer + 41, &crc_data, 2);
    CDC_Transmit_FS(send_buffer, sizeof(send_packed_t));
}

// ================== 核心修改：Auto_Track ==================
void Auto_Track(gimbal_control_t *gimbal_control_set) {
    if (gimbal_control_set == NULL) {
        return;
    }

    static uint8_t last_mode = 0;
    uint8_t current_mode = received_packed.mode;

    if (current_mode == 1) {
        // === 1. 复位逻辑 ===
        if (last_mode != 1) {
            // 切换瞬间，必须同时复位 Ramp 和 LPF 的状态
            // 这样才能保证完全无缝衔接
            yaw_ramp.current_out = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
            yaw_lpf.current_out  = gimbal_control_set->gimbal_yaw_motor.absolute_angle;

            pitch_ramp.current_out = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
            pitch_lpf.current_out  = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
        }

        // === 2. 串联处理 (Cascade Control) ===
        // 第一级：斜坡处理 (Yaw)
        float ramp_yaw = Ramp_Calc(&yaw_ramp, received_packed.yaw);
        // 第二级：滤波处理 (Yaw) - 把斜坡的结果喂给滤波
        float final_yaw = LPF_Calc(&yaw_lpf, ramp_yaw);

        // 第一级：斜坡处理 (Pitch)
        float ramp_pitch = Ramp_Calc(&pitch_ramp, received_packed.pitch);
        // 第二级：滤波处理 (Pitch)
        float final_pitch = LPF_Calc(&pitch_lpf, ramp_pitch);

        // === 3. 赋值 ===
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = final_pitch;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = final_yaw;
    }
    else {
        // 非自瞄模式下保持位置
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    }

    last_mode = current_mode;
}
// ========================================================

received_packed_t *get_AUTOshoot_point(void) {
    return &received_packed;
}

uint8_t *get_auto_mode_flag(void) {
    return (uint8_t *)(&flag_received);
}

const uint16_t wCRC_Table_Auto[256] =
        {
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
            0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
            0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
            0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
            0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
            0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
            0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
            0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
            0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
            0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
            0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
            0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
            0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
            0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
            0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
            0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
            0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
            0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
            0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
            0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
            0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
            0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
            0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
            0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
            0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
            0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
            0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
            0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
            0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
            0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
        };

uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_Auto[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


//线性化处理
// #include "auto_shoot_new.h"
// #include "referee.h"
// #include "arm_math.h"
// #include "user_lib.h"
// #include "fifo.h"
// #include "usbd_cdc_if.h"
// #include "INS_task.h"
//
// // 外部结构体指针
// extern ext_game_robot_state_t robot_state;  // 当前机器人的状态
// extern gimbal_control_t gimbal_control;
// extern gimbal_behaviour_e gimbal_behaviour;
//
// received_packed_t received_packed;
// send_packed_t send_packed;
//
// //解包
// fifo_s_t auto_shoot_fifo;
// uint8_t auto_shoot_fifo_buf[AUTO_SHOOT_FIFO_BUF_LENGTH];  // FIFO缓冲区
// unpack_autoshoot_data_t autoshoot_unpack_obj;  // 解包管理结构体
// uint8_t flag_received = 0;  // 数据接收标志位
// uint8_t frame, frame_count = 0;  // 当前接收数据的帧率和帧率计数值
//
// //CRC校验
// const uint16_t CRC_INIT_AUTO = 0xffff;  // CRC校验初始值
//
// // ================== 新增：斜坡变量定义 ==================
// static Ramp_t yaw_ramp = {0.0f, RAMP_MAX_STEP};
// static Ramp_t pitch_ramp = {0.0f, RAMP_MAX_STEP};
// // ======================================================
//
// // 初始化函数，初始化FIFO
// void AUTO_init(void) {
//     fifo_s_init(&auto_shoot_fifo, auto_shoot_fifo_buf, AUTO_SHOOT_FIFO_BUF_LENGTH);
//     memset(&received_packed, 0, sizeof(received_packed_t));
//     memset(&send_packed, 0, sizeof(send_packed_t));
//
//     // 初始化斜坡步进
//     yaw_ramp.max_step = RAMP_MAX_STEP;
//     pitch_ramp.max_step = RAMP_MAX_STEP;
// }
//
// // ================== 新增：斜坡计算函数 ==================
// /**
//  * @brief  斜坡函数计算，将阶跃信号转化为线性信号
//  * @param  ramp: 斜坡结构体指针
//  * @param  target: 原始的目标值
//  * @return 平滑后的目标值
//  */
// float Ramp_Calc(Ramp_t *ramp, float target) {
//     float err = target - ramp->current_out;
//
//     // 如果误差超过了最大步进量，就只走一步
//     if (err > ramp->max_step) {
//         ramp->current_out += ramp->max_step;
//     }
//     else if (err < -ramp->max_step) {
//         ramp->current_out -= ramp->max_step;
//     }
//     else {
//         // 如果误差很小，就直接达到目标
//         ramp->current_out = target;
//     }
//     return ramp->current_out;
// }
// // ======================================================
//
// // 从FIFO中解包数据
// void autoshoot_unpack_fifo_data(void) {
//     // 状态机变量：0-找'S'，1-找'P'，2-读数据
//     static uint8_t state = 0;
//     // 当前已接收的字节数
//     static uint16_t bytes_received = 0;
//     // 获取解包对象指针
//     unpack_autoshoot_data_t *p_obj = &autoshoot_unpack_obj;
//     // 计算完整数据帧的长度（包含帧头、数据和CRC）
//     uint16_t data_len = sizeof(received_packed_t);
//
//     // 只要FIFO中有数据，就持续处理
//     while (fifo_s_used(&auto_shoot_fifo) > 0) {
//         // 从FIFO中取出一个字节
//         uint8_t byte = fifo_s_get(&auto_shoot_fifo);
//
//         /*********************** 状态0：寻找帧头第一个字节'S' ***********************/
//         if (state == 0) {
//             // 检查当前字节是否是帧头第一个字节'S'
//             if (byte == 'S') {
//                 // 存储到协议包缓冲区第0个位置
//                 p_obj->protocol_packet[0] = byte;
//                 // 已接收字节数设为1
//                 bytes_received = 1;
//                 // 切换到状态1：寻找第二个字节'P'
//                 state = 1;
//             }
//             // 如果不是'S'，则继续在状态0循环，等待下一个字节
//         }
//
//             /*********************** 状态1：寻找帧头第二个字节'P' ***********************/
//         else if (state == 1) {
//             /*
//              * 使用嵌套三元运算符更新状态：
//              * 1. 如果当前字节是'P'，状态变为2（开始读数据）
//              * 2. 如果当前字节是'S'，状态保持为1（重新开始找'P'）
//              * 3. 如果既不是'P'也不是'S'，状态重置为0（重新找'S'）
//              */
//             state = (byte == 'P') ? 2 : (byte == 'S') ? 1 : 0;
//
//             // 根据字节内容进行不同处理
//             if (byte == 'P') {
//                 // 找到'P'，存储到协议包缓冲区第1个位置
//                 p_obj->protocol_packet[1] = byte;
//                 // 已接收字节数设为2（已经收到"S"和"P"）
//                 bytes_received = 2;
//             } else if (byte == 'S') {
//                 // 当前字节是'S'，可能是下一帧的开始
//                 // 存储到协议包缓冲区第0个位置（覆盖之前的'S'）
//                 p_obj->protocol_packet[0] = byte;
//                 // 已接收字节数设为1（只有这个'S'）
//                 bytes_received = 1;
//             } else {
//                 // 既不是'P'也不是'S'，重置已接收字节数
//                 bytes_received = 0;
//             }
//         }
//
//             /*********************** 状态2：读取数据部分 ***********************/
//         else {  // state == 2
//             // 将当前字节存储到协议包缓冲区的下一个位置
//             p_obj->protocol_packet[bytes_received++] = byte;
//
//             // 检查是否已收到完整的一帧数据
//             if (bytes_received == data_len) {
//                 // 从协议包的最后两个字节提取接收到的CRC校验值
//                 // 注意：data_len-1是高字节，data_len-2是低字节（小端序）
//                 uint16_t received_crc = (p_obj->protocol_packet[data_len - 1] << 8) |
//                                         p_obj->protocol_packet[data_len - 2];
//
//                 // 计算协议包数据部分的CRC（排除最后2字节的CRC本身）
//                 uint16_t calculated_crc = Get_CRC16_Check_Sum_Auto(p_obj->protocol_packet,
//                                                                    data_len - 2,
//                                                                    CRC_INIT_AUTO);
//
//                 // 校验CRC是否匹配
//                 if (calculated_crc == received_crc) {
//                     // CRC校验通过，复制数据到接收结构体
//                     memcpy(&received_packed, p_obj->protocol_packet, data_len);
//                     // 设置接收成功标志
//                     flag_received = 1;
//                     // 成功接收的帧计数加1
//                     frame_count++;
//                 }
//
//                 // 无论CRC是否通过，都重置状态机，准备接收下一帧
//                 state = bytes_received = 0;
//             }
//         }
//     }
// }
//
// // 自动任务函数，循环执行解包和发送数据操作
// void auto_task(void const *argument) {
//     AUTO_init();
//     while (1) {
//         autoshoot_unpack_fifo_data();
//         send_data_to_upper();
//         vTaskDelay(10);
//     }
// }
//
// // 发送数据给上位机
// void send_data_to_upper(void) {
//
//     uint8_t send_buffer[sizeof(send_packed_t)] = {0};
//
//     // 填充帧头
//     const uint8_t head[2] = {'S', 'P'};
//     memcpy(send_buffer + 0, head, 2);
//
//     // 填充模式信息
//     const uint8_t mode = (gimbal_behaviour == GIMBAL_AUTO) ? 1 : 0;
//     memcpy(send_buffer + 2, &mode, 1);
//
//     // 填充四元数 - 使用固定值测试用
//     // fp32 quat_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
//     const fp32* quat_data =  get_INS_quat_point();
//     memcpy(send_buffer + 3, quat_data, 16);  // 4个float = 16字节
//
//     // 填充云台状态
//     const fp32 yaw_data = gimbal_control.gimbal_yaw_motor.absolute_angle;
//     const fp32 yaw_vel_data = gimbal_control.gimbal_yaw_motor.motor_gyro;
//     const fp32 pitch_data = gimbal_control.gimbal_pitch_motor.absolute_angle;
//     const fp32 pitch_vel_data = gimbal_control.gimbal_pitch_motor.motor_gyro;
//     memcpy(send_buffer + 19, &yaw_data, 4);
//     memcpy(send_buffer + 23, &yaw_vel_data, 4);
//     memcpy(send_buffer + 27, &pitch_data, 4);
//     memcpy(send_buffer + 31, &pitch_vel_data, 4);
//
//     // 填充射击信息
//     const fp32 bullet_speed_data = 15.0f;
//     const uint16_t bullet_count_data = 0;
//     memcpy(send_buffer + 35, &bullet_speed_data, 4);
//     memcpy(send_buffer + 39, &bullet_count_data, 2);
//
//     // 计算CRC16校验
//     const uint32_t crc_length = sizeof(send_packed_t) - 2; // 减去CRC16本身的2字节
//     const uint16_t crc_data = Get_CRC16_Check_Sum_Auto(send_buffer, crc_length, CRC_INIT_AUTO);
//     memcpy(send_buffer + 41, &crc_data, 2);
//
//     // 发送数据
//     CDC_Transmit_FS(send_buffer, sizeof(send_packed_t));
// }
//
// // 自动跟踪函数
// void Auto_Track(gimbal_control_t *gimbal_control_set) {
//     if (gimbal_control_set == NULL) {
//         return;
//     }
//
//     // 记录上一次的模式，用于检测切换边沿
//     static uint8_t last_mode = 0;
//     uint8_t current_mode = received_packed.mode;
//
//     if (current_mode == 1) {
//         // ========== 新增逻辑：检测到从其他模式切换到自瞄模式 ==========
//         if (last_mode != 1) {
//             // 在切换瞬间，将斜坡的当前输出强制复位为电机当前的实际角度
//             // 这样Ramp函数就会从"现在的位置"开始起跑，实现无扰切换
//             yaw_ramp.current_out = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//             pitch_ramp.current_out = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//         }
//         // ==========================================================
//
//         // 使用斜坡函数计算平滑后的目标值
//         float smooth_pitch = Ramp_Calc(&pitch_ramp, received_packed.pitch);
//         float smooth_yaw = Ramp_Calc(&yaw_ramp, received_packed.yaw);
//
//         // 将平滑后的值赋给设定值
//         gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = smooth_pitch;
//         gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = smooth_yaw;
//     }
//     else {
//         // 非自瞄模式下，保持当前位置
//         gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//         gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//
//         // 可选：在非自瞄模式下，也持续更新斜坡内部状态跟随实际值，作为双重保险
//         // yaw_ramp.current_out = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//         // pitch_ramp.current_out = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//     }
//
//     // 更新历史模式状态
//     last_mode = current_mode;
// }
//
// // 返回结构体指针
// received_packed_t *get_AUTOshoot_point(void) {
//     return &received_packed;
// }
//
// // 返回自动模式标志位指针
// uint8_t *get_auto_mode_flag(void) {
//     return (uint8_t *)(&flag_received);
// }
//
// const uint16_t wCRC_Table_Auto[256] =
//         {
//             0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//             0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//             0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//             0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//             0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//             0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//             0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//             0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//             0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//             0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//             0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//             0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//             0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//             0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//             0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//             0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//             0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//             0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//             0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//             0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//             0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//             0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//             0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//             0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//             0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//             0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//             0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//             0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//             0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//             0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//             0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//             0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//         };
//
// /*
// ** Descriptions: CRC16 checksum function
// ** Input: Data to check,Stream length, initialized checksum
// ** Output: CRC checksum
// */
// uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
// {
//     uint8_t chData;
//     if (pchMessage == NULL)
//     {
//         return 0xFFFF;
//     }
//     while(dwLength--)
//     {
//         chData = *pchMessage++;
//         (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_Auto[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
//     }
//     return wCRC;
// }



//原始版本
// #include "auto_shoot_new.h"
// #include "referee.h"
// #include "arm_math.h"
// #include "user_lib.h"
// #include "fifo.h"
// #include "usbd_cdc_if.h"
// #include "INS_task.h"
//
// // 外部结构体指针
// extern ext_game_robot_state_t robot_state;  // 当前机器人的状态
// extern gimbal_control_t gimbal_control;
// extern gimbal_behaviour_e gimbal_behaviour;
//
// received_packed_t received_packed;
// send_packed_t send_packed;
//
// //解包
// fifo_s_t auto_shoot_fifo;
// uint8_t auto_shoot_fifo_buf[AUTO_SHOOT_FIFO_BUF_LENGTH];  // FIFO缓冲区
// unpack_autoshoot_data_t autoshoot_unpack_obj;  // 解包管理结构体
// uint8_t flag_received = 0;  // 数据接收标志位
// uint8_t frame, frame_count = 0;  // 当前接收数据的帧率和帧率计数值
//
// //CRC校验
// const uint16_t CRC_INIT_AUTO = 0xffff;  // CRC校验初始值
//
// // 初始化函数，初始化FIFO
// void AUTO_init(void) {
//     fifo_s_init(&auto_shoot_fifo, auto_shoot_fifo_buf, AUTO_SHOOT_FIFO_BUF_LENGTH);
//     memset(&received_packed, 0, sizeof(received_packed_t));
//     memset(&send_packed, 0, sizeof(send_packed_t));
// }
//
// // 从FIFO中解包数据
// void autoshoot_unpack_fifo_data(void) {
//     // 状态机变量：0-找'S'，1-找'P'，2-读数据
//     static uint8_t state = 0;
//     // 当前已接收的字节数
//     static uint16_t bytes_received = 0;
//     // 获取解包对象指针
//     unpack_autoshoot_data_t *p_obj = &autoshoot_unpack_obj;
//     // 计算完整数据帧的长度（包含帧头、数据和CRC）
//     uint16_t data_len = sizeof(received_packed_t);
//
//     // 只要FIFO中有数据，就持续处理
//     while (fifo_s_used(&auto_shoot_fifo) > 0) {
//         // 从FIFO中取出一个字节
//         uint8_t byte = fifo_s_get(&auto_shoot_fifo);
//
//         /*********************** 状态0：寻找帧头第一个字节'S' ***********************/
//         if (state == 0) {
//             // 检查当前字节是否是帧头第一个字节'S'
//             if (byte == 'S') {
//                 // 存储到协议包缓冲区第0个位置
//                 p_obj->protocol_packet[0] = byte;
//                 // 已接收字节数设为1
//                 bytes_received = 1;
//                 // 切换到状态1：寻找第二个字节'P'
//                 state = 1;
//             }
//             // 如果不是'S'，则继续在状态0循环，等待下一个字节
//         }
//
//             /*********************** 状态1：寻找帧头第二个字节'P' ***********************/
//         else if (state == 1) {
//             /*
//              * 使用嵌套三元运算符更新状态：
//              * 1. 如果当前字节是'P'，状态变为2（开始读数据）
//              * 2. 如果当前字节是'S'，状态保持为1（重新开始找'P'）
//              * 3. 如果既不是'P'也不是'S'，状态重置为0（重新找'S'）
//              */
//             state = (byte == 'P') ? 2 : (byte == 'S') ? 1 : 0;
//
//             // 根据字节内容进行不同处理
//             if (byte == 'P') {
//                 // 找到'P'，存储到协议包缓冲区第1个位置
//                 p_obj->protocol_packet[1] = byte;
//                 // 已接收字节数设为2（已经收到"S"和"P"）
//                 bytes_received = 2;
//             } else if (byte == 'S') {
//                 // 当前字节是'S'，可能是下一帧的开始
//                 // 存储到协议包缓冲区第0个位置（覆盖之前的'S'）
//                 p_obj->protocol_packet[0] = byte;
//                 // 已接收字节数设为1（只有这个'S'）
//                 bytes_received = 1;
//             } else {
//                 // 既不是'P'也不是'S'，重置已接收字节数
//                 bytes_received = 0;
//             }
//         }
//
//             /*********************** 状态2：读取数据部分 ***********************/
//         else {  // state == 2
//             // 将当前字节存储到协议包缓冲区的下一个位置
//             p_obj->protocol_packet[bytes_received++] = byte;
//
//             // 检查是否已收到完整的一帧数据
//             if (bytes_received == data_len) {
//                 // 从协议包的最后两个字节提取接收到的CRC校验值
//                 // 注意：data_len-1是高字节，data_len-2是低字节（小端序）
//                 uint16_t received_crc = (p_obj->protocol_packet[data_len - 1] << 8) |
//                                         p_obj->protocol_packet[data_len - 2];
//
//                 // 计算协议包数据部分的CRC（排除最后2字节的CRC本身）
//                 uint16_t calculated_crc = Get_CRC16_Check_Sum_Auto(p_obj->protocol_packet,
//                                                                    data_len - 2,
//                                                                    CRC_INIT_AUTO);
//
//                 // 校验CRC是否匹配
//                 if (calculated_crc == received_crc) {
//                     // CRC校验通过，复制数据到接收结构体
//                     memcpy(&received_packed, p_obj->protocol_packet, data_len);
//                     // 设置接收成功标志
//                     flag_received = 1;
//                     // 成功接收的帧计数加1
//                     frame_count++;
//                 }
//
//                 // 无论CRC是否通过，都重置状态机，准备接收下一帧
//                 state = bytes_received = 0;
//             }
//         }
//     }
// }
//
// // 自动任务函数，循环执行解包和发送数据操作
// void auto_task(void const *argument) {
//     AUTO_init();
//     while (1) {
//         autoshoot_unpack_fifo_data();
//         send_data_to_upper();
//         vTaskDelay(10);
//     }
// }
//
// // 发送数据给上位机
// void send_data_to_upper(void) {
//
//     uint8_t send_buffer[sizeof(send_packed_t)] = {0};
//
//     // 填充帧头
//     const uint8_t head[2] = {'S', 'P'};
//     memcpy(send_buffer + 0, head, 2);
//
//     // 填充模式信息
//     const uint8_t mode = (gimbal_behaviour == GIMBAL_AUTO) ? 1 : 0;
//     memcpy(send_buffer + 2, &mode, 1);
//
//     // 填充四元数 - 使用固定值测试用
//     // fp32 quat_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
//     const fp32* quat_data =  get_INS_quat_point();
//     memcpy(send_buffer + 3, quat_data, 16);  // 4个float = 16字节
//
//     // 填充云台状态
//     const fp32 yaw_data = gimbal_control.gimbal_yaw_motor.absolute_angle;
//     const fp32 yaw_vel_data = gimbal_control.gimbal_yaw_motor.motor_gyro;
//     const fp32 pitch_data = gimbal_control.gimbal_pitch_motor.absolute_angle;
//     const fp32 pitch_vel_data = gimbal_control.gimbal_pitch_motor.motor_gyro;
//     memcpy(send_buffer + 19, &yaw_data, 4);
//     memcpy(send_buffer + 23, &yaw_vel_data, 4);
//     memcpy(send_buffer + 27, &pitch_data, 4);
//     memcpy(send_buffer + 31, &pitch_vel_data, 4);
//
//     // 填充射击信息
//     const fp32 bullet_speed_data = 15.0f;
//     const uint16_t bullet_count_data = 0;
//     memcpy(send_buffer + 35, &bullet_speed_data, 4);
//     memcpy(send_buffer + 39, &bullet_count_data, 2);
//
//     // 计算CRC16校验
//     const uint32_t crc_length = sizeof(send_packed_t) - 2; // 减去CRC16本身的2字节
//     const uint16_t crc_data = Get_CRC16_Check_Sum_Auto(send_buffer, crc_length, CRC_INIT_AUTO);
//     memcpy(send_buffer + 41, &crc_data, 2);
//
//     // 发送数据
//     CDC_Transmit_FS(send_buffer, sizeof(send_packed_t));
// }
//
// // 自动跟踪函数
// void Auto_Track(gimbal_control_t *gimbal_control_set) {
//     if (gimbal_control_set == NULL) {
//         return;
//     }
//
//     if (received_packed.mode == 1) {
//
//         gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = received_packed.pitch;
//         gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = received_packed.yaw;
//     }
//     else {
//         gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//         gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//     }
// }
//
// // 返回结构体指针
// received_packed_t *get_AUTOshoot_point(void) {
//     return &received_packed;
// }
//
// // 返回自动模式标志位指针
// uint8_t *get_auto_mode_flag(void) {
//     return (uint8_t *)(&flag_received);
// }
//
// const uint16_t wCRC_Table_Auto[256] =
//         {
//             0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//             0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//             0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//             0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//             0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//             0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//             0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//             0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//             0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//             0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//             0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//             0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//             0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//             0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//             0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//             0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//             0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//             0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//             0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//             0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//             0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//             0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//             0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//             0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//             0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//             0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//             0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//             0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//             0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//             0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//             0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//             0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//         };
//
// /*
// ** Descriptions: CRC16 checksum function
// ** Input: Data to check,Stream length, initialized checksum
// ** Output: CRC checksum
// */
// uint16_t Get_CRC16_Check_Sum_Auto(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
// {
//     uint8_t chData;
//     if (pchMessage == NULL)
//     {
//         return 0xFFFF;
//     }
//     while(dwLength--)
//     {
//         chData = *pchMessage++;
//         (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_Auto[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
//     }
//     return wCRC;
// }