//#include "sys_id_task.h"
//#include "main.h"
//#include <math.h>
//#include <stdio.h>
//
//// 队列定义
//osMessageQDef(sysIdCmdQueue, 5, sys_id_message_t);
//osMessageQId xSysIdCmdQueue;
//
//osMessageQDef(sysIdControlQueue, 2, control_cmd_t);
//osMessageQId xSysIdControlQueue;
//
//osMessageQDef(sysIdLogQueue, 100, log_data_t);
//osMessageQId xSysIdLogQueue;
//
//// 任务状态变量
//static uint8_t experiment_active = 0;
//static float current_phase = 0.0f;
//static uint32_t experiment_start_tick = 0;
//
//// 默认扫频参数
//static chirp_params_t current_params = {
//        .start_freq_hz = 0.5f,
//        .end_freq_hz = 15.0f,
//        .duration_ms = 20000,
//        .base_amplitude = 5000,
//        .axis_select = 0  // 默认Yaw轴
//};
//
//// 幅值调度函数
//static float get_scheduled_amplitude(float current_freq_hz) {
//    float amplitude = current_params.base_amplitude;
//
//    // 低频时增大幅值克服静摩擦
//    if (current_freq_hz <= 2.0f) {
//        amplitude *= 1.5f;
//    }
//        // 高频时减小幅值避免饱和
//    else if (current_freq_hz >= 10.0f) {
//        amplitude *= 0.7f;
//    }
//
//    // 限幅保护
//    if (amplitude > 20000) amplitude = 20000;
//    if (amplitude < 1000) amplitude = 1000;
//
//    return amplitude;
//}
//
//// 系统辨识任务主函数
//void sys_id_task(void const * argument)
//{
//    sys_id_message_t received_msg;
//    osEvent evt;
//    uint32_t last_wake_time = osKernelSysTick();
//
//    // 初始化队列
//    xSysIdCmdQueue = osMessageCreate(osMessageQ(sysIdCmdQueue), NULL);
//    xSysIdControlQueue = osMessageCreate(osMessageQ(sysIdControlQueue), NULL);
//    xSysIdLogQueue = osMessageCreate(osMessageQ(sysIdLogQueue), NULL);
//
//    printf("System Identification Task Started\r\n");
//    printf("Default Parameters: Start=%.1fHz, End=%.1fHz, Duration=%lums, Amp=%ld\r\n",
//           current_params.start_freq_hz, current_params.end_freq_hz,
//           current_params.duration_ms, current_params.base_amplitude);
//
//    // 等待系统稳定
//    osDelay(1000);
//
//    for(;;)
//    {
//        // 精确1ms周期延迟 - 确保1kHz执行频率
//        osDelayUntil(&last_wake_time, 1);
//
//        uint32_t current_tick = osKernelSysTick();
//        int32_t voltage_cmd_yaw = 0;
//        int32_t voltage_cmd_pitch = 0;
//
//        // 1. 处理命令（非阻塞方式）
//        evt = osMessageGet(xSysIdCmdQueue, 0);
//        if (evt.status == osEventMessage) {
//            received_msg = *(sys_id_message_t*)evt.value.p;
//
//            switch (received_msg.cmd) {
//                case SYS_ID_CMD_START:
//                    experiment_active = 1;
//                    experiment_start_tick = current_tick;
//                    current_phase = 0.0f;
//                    if (received_msg.params.base_amplitude > 0) {
//                        current_params = received_msg.params;
//                    }
//                    printf("System ID Experiment Started on Axis %d\r\n", current_params.axis_select);
//                    printf("Params: %.1fHz->%.1fHz, %lums, Amp=%ld\r\n",
//                           current_params.start_freq_hz, current_params.end_freq_hz,
//                           current_params.duration_ms, current_params.base_amplitude);
//                    break;
//
//                case SYS_ID_CMD_STOP:
//                    experiment_active = 0;
//                    printf("System ID Experiment Stopped\r\n");
//                    break;
//
//                case SYS_ID_CMD_SET_PARAMS:
//                    current_params = received_msg.params;
//                    printf("System ID Parameters Updated\r\n");
//                    break;
//            }
//        }
//
//        // 2. 实验运行逻辑
//        if (experiment_active) {
//            uint32_t elapsed_ticks = current_tick - experiment_start_tick;
//            float elapsed_time_sec = elapsed_ticks * 0.001f;
//
//            if (elapsed_time_sec <= (current_params.duration_ms * 0.001f)) {
//                // 计算当前瞬时频率（线性扫频）
//                float freq_slope = (current_params.end_freq_hz - current_params.start_freq_hz)
//                                   / (current_params.duration_ms * 0.001f);
//                float instantaneous_freq = current_params.start_freq_hz + freq_slope * elapsed_time_sec;
//
//                // 幅值调度
//                float scheduled_amp = get_scheduled_amplitude(instantaneous_freq);
//
//                // 更新相位并生成信号
//                float phase_increment = 2.0f * (float)M_PI * instantaneous_freq * 0.001f;
//                current_phase += phase_increment;
//
//                // 相位规范化 [0, 2π]
//                if (current_phase > 2 * M_PI) {
//                    current_phase -= 2 * M_PI;
//                }
//
//                int32_t signal_output = (int32_t)(scheduled_amp * sinf(current_phase));
//
//                // 根据选择的轴分配信号
//                if (current_params.axis_select == 0) {
//                    voltage_cmd_yaw = signal_output;    // Yaw轴
//                } else {
//                    voltage_cmd_pitch = signal_output;  // Pitch轴
//                }
//
//            } else {
//                // 实验时间到，自动停止
//                experiment_active = 0;
//                printf("System ID Experiment Completed\r\n");
//            }
//        }
//
//        // 3. 发送控制指令到云台任务
//        if (xSysIdControlQueue != NULL) {
//            // 发送Yaw轴指令
//            if (voltage_cmd_yaw != 0) {
//                control_cmd_t ctrl_msg = {
//                        .voltage_cmd = voltage_cmd_yaw,
//                        .source_sysid = experiment_active,
//                        .axis_target = 0  // Yaw轴
//                };
//                osMessagePut(xSysIdControlQueue, (uint32_t)&ctrl_msg, 0);
//            }
//
//            // 发送Pitch轴指令
//            if (voltage_cmd_pitch != 0) {
//                control_cmd_t ctrl_msg = {
//                        .voltage_cmd = voltage_cmd_pitch,
//                        .source_sysid = experiment_active,
//                        .axis_target = 1  // Pitch轴
//                };
//                osMessagePut(xSysIdControlQueue, (uint32_t)&ctrl_msg, 0);
//            }
//        }
//
//        // 4. 发送数据到记录队列（用于调试和数据分析）
//        if (xSysIdLogQueue != NULL) {
//            // 这里需要您从IMU获取实际的角度和角速度数据
//            // 以下为示例数据，请替换为您的IMU读取代码
//            float imu_angle = 0.0f;    // 需要替换为实际IMU角度
//            float imu_velocity = 0.0f; // 需要替换为实际IMU角速度
//
//            log_data_t log_msg = {
//                    .timestamp_ms = current_tick,
//                    .voltage_cmd = (voltage_cmd_yaw != 0) ? voltage_cmd_yaw : voltage_cmd_pitch,
//                    .imu_angle = imu_angle,
//                    .imu_velocity = imu_velocity,
//                    .experiment_active = experiment_active
//            };
//
//            // 非阻塞方式发送，队列满时丢弃最旧数据
//            osMessagePut(xSysIdLogQueue, (uint32_t)&log_msg, 0);
//        }
//    }
//}
//
//// 发送命令接口函数
//void sys_id_send_command(sys_id_cmd_t cmd)
//{
//    sys_id_message_t msg = {.cmd = cmd};
//    if (xSysIdCmdQueue != NULL) {
//        osMessagePut(xSysIdCmdQueue, (uint32_t)&msg, osWaitForever);
//    }
//}
//
//// 设置参数接口函数
//void sys_id_set_params(chirp_params_t* params)
//{
//    sys_id_message_t msg = {
//            .cmd = SYS_ID_CMD_SET_PARAMS,
//            .params = *params
//    };
//    if (xSysIdCmdQueue != NULL) {
//        osMessagePut(xSysIdCmdQueue, (uint32_t)&msg, osWaitForever);
//    }
//}
//
//// 获取实验状态
//uint8_t sys_id_is_running(void)
//{
//    return experiment_active;
//}