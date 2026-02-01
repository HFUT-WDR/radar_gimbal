//#ifndef __SYS_ID_TASK_H__
//#define __SYS_ID_TASK_H__
//
//#include "cmsis_os.h"
//#include <stdint.h>
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//// 系统辨识命令枚举
//typedef enum {
//    SYS_ID_CMD_START = 0,    // 开始系统辨识
//    SYS_ID_CMD_STOP,         // 停止系统辨识
//    SYS_ID_CMD_SET_PARAMS    // 设置扫频参数
//} sys_id_cmd_t;
//
//// 扫频信号参数结构
//typedef struct {
//    float start_freq_hz;     // 起始频率 (Hz)
//    float end_freq_hz;       // 结束频率 (Hz)
//    uint32_t duration_ms;    // 扫频持续时间 (ms)
//    int32_t base_amplitude;  // 基础幅值
//    uint8_t axis_select;     // 轴选择 (0: Yaw, 1: Pitch)
//} chirp_params_t;
//
//// 系统辨识命令消息结构
//typedef struct {
//    sys_id_cmd_t cmd;        // 命令类型
//    chirp_params_t params;   // 参数设置
//} sys_id_message_t;
//
//// 控制指令结构（发送给云台任务）
//typedef struct {
//    int32_t voltage_cmd;     // 电压指令值
//    uint8_t source_sysid;   // 指令来源标记
//    uint8_t axis_target;    // 目标轴 (0: Yaw, 1: Pitch)
//} control_cmd_t;
//
//// 数据记录结构
//typedef struct {
//    uint32_t timestamp_ms;   // 时间戳 (ms)
//    int32_t voltage_cmd;     // 电压指令
//    float imu_angle;        // IMU角度 (rad)
//    float imu_velocity;     // IMU角速度 (rad/s)
//    uint8_t experiment_active; // 实验状态
//} log_data_t;
//
//// 队列句柄声明
//extern osMessageQId xSysIdCmdQueue;     // 命令输入队列
//extern osMessageQId xSysIdControlQueue; // 控制指令输出队列
//extern osMessageQId xSysIdLogQueue;     // 数据记录队列
//
//// 函数声明
//void sys_id_task(void const * argument);
//void sys_id_send_command(sys_id_cmd_t cmd);
//void sys_id_set_params(chirp_params_t* params);
//uint8_t sys_id_is_running(void);
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* __SYS_ID_TASK_H__ */