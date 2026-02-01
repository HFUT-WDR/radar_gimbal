/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"


#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"

#include "sys_id_task.h"

static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
//        osDelay(1000);
//        usb_printf(
//"******************************\r\n\
//voltage percentage:%d%% \r\n\
//DBUS:%s\r\n\
//chassis motor1:%s\r\n\
//chassis motor2:%s\r\n\
//chassis motor3:%s\r\n\
//chassis motor4:%s\r\n\
//yaw motor:%s\r\n\
//pitch motor:%s\r\n\
//trigger motor:%s\r\n\
//gyro sensor:%s\r\n\
//accel sensor:%s\r\n\
//mag sensor:%s\r\n\
//referee usart:%s\r\n\
//******************************\r\n",
//            status[error_list_usb_local[DBUS_TOE].error_exist],
//            status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//            status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//            status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//            status[error_list_usb_local[REFEREE_TOE].error_exist]);
//        if (size == 0) return;
//
//        chirp_params_t params;
//
//        switch (cmd[0]) {
//            case 's': // 开始系统辨识实验
//                sys_id_send_command(SYS_ID_CMD_START);
//                printf("System ID: Start command sent\r\n");
//                break;
//
//            case 't': // 停止系统辨识实验
//                sys_id_send_command(SYS_ID_CMD_STOP);
//                printf("System ID: Stop command sent\r\n");
//                break;
//
//            case 'y': // 设置Yaw轴辨识
//                params.start_freq_hz = 0.5f;
//                params.end_freq_hz = 12.0f;
//                params.duration_ms = 15000;
//                params.base_amplitude = 4000;
//                params.axis_select = 0; // Yaw轴
//                sys_id_set_params(&params);
//                printf("System ID: Yaw axis parameters set\r\n");
//                break;
//
//            case 'p': // 设置Pitch轴辨识
//                params.start_freq_hz = 0.3f;
//                params.end_freq_hz = 8.0f;
//                params.duration_ms = 20000;
//                params.base_amplitude = 3000;
//                params.axis_select = 1; // Pitch轴
//                sys_id_set_params(&params);
//                printf("System ID: Pitch axis parameters set\r\n");
//                break;
//
//            case 'h': // 帮助信息
//                printf("System ID Commands:\r\n");
//                printf("  s - Start experiment\r\n");
//                printf("  t - Stop experiment\r\n");
//                printf("  y - Set Yaw axis params\r\n");
//                printf("  p - Set Pitch axis params\r\n");
//                printf("  h - Show this help\r\n");
//                break;
//
//            default:
//                printf("Unknown command. Send 'h' for help.\r\n");
//                break;
//        }
//
    }

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}
