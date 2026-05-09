/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Attitude/Accel_Calibration.h"
#include "Attitude/IMU_TOP.h"
#include "control/control.h"
#include "encoder/encoder_control.h"
#include "menu/menu_config.h"
#include "motor/motor.h"
#include "odometer/odometer.h"
#include "target_follow/target_follow.h"
#include "target_follow/target_follow_config.h"
#include "uwb/ALX_AOA.h"
#include "uwb/uwb_follow.h"
#include "wireless_control/wireless_control.h"
#include "wifi/wifi_core.h"
#include "wifi/wifi_justfloat/wifi_justfloat.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

volatile uint8_t timer_10ms_flag = 0;
volatile uint8_t timer_20ms_flag = 0;
volatile uint8_t timer_40ms_flag = 0;
volatile uint16 g_tick_1000HZ = 0U;
static float yaw_angle_target = 0.0f;
static float remote_forward_target = 0.0f;
static float remote_strafe_target = 0.0f;
static float remote_rotate_target = 0.0f;
static uint8_t remote_last_rotate_active = 0U;
static uint8_t yaw_angle_hold_active = 0U;
static uint32 telemetry_timestamp_count = 0U;
static uint32 system_time_ms = 0U;
static uint8_t last_control_mode = WIRELESS_CONTROL_MODE_REMOTE;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    control_cascade_init();
    uart_receiver_init();
    wireless_control_init();
    wifi_core_Init();
    ALX_AOA_Init();
    uwb_follow_init();
    target_follow_init();
    target_follow_load_default_targets();
    pit_init(PIT_CH0, 1000);

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        uint16 imu_tick_guard = 0U;

        while((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
        {
            g_tick_1000HZ--;
            IMU_Update_1000HZ();
            imu_tick_guard++;
        }
        // 此处编写需要循环执行的代码
        if(timer_40ms_flag)
        {
            timer_40ms_flag = 0;
            (void)ALX_AOA_Update(system_time_ms);

            wireless_control_task();
            if(last_control_mode != g_wireless_control_state.mode)
            {
                control_cascade_reset();
                uwb_follow_reset();
                target_follow_reset();
                remote_last_rotate_active = 0U;
                yaw_angle_hold_active = 0U;
                yaw_angle_target = control_get_current_yaw_angle();
                last_control_mode = g_wireless_control_state.mode;
            }

            if(0U != g_wireless_control_state.control_enabled)
            {
                if(WIRELESS_CONTROL_MODE_UWB_FOLLOW == g_wireless_control_state.mode)
                {
                    target_follow_update(system_time_ms);
                    remote_forward_target = (0U != g_target_follow_state.output_valid) ?
                                            g_target_follow_state.forward_target : 0.0f;
                    remote_strafe_target = (0U != g_target_follow_state.output_valid) ?
                                           g_target_follow_state.strafe_target : 0.0f;
                    remote_rotate_target = 0.0f;
                }
                else
                {
                    remote_forward_target = (float)g_wireless_control_state.forward_speed;
                    remote_strafe_target = (float)g_wireless_control_state.strafe_speed;
                    remote_rotate_target = (float)g_wireless_control_state.rotate_speed;
                }

                if(0.0f != remote_rotate_target)
                {
                    yaw_angle_target = control_get_current_yaw_angle();
                    yaw_angle_hold_active = 0U;
                }
                else
                {
                    if((0U == yaw_angle_hold_active) || (0U != remote_last_rotate_active))
                    {
                        yaw_angle_target = control_get_current_yaw_angle();
                        control_yaw_angle_loop_reset();
                        yaw_angle_hold_active = 1U;
                    }
                    control_yaw_angle_loop_update(yaw_angle_target);
                }
                remote_last_rotate_active = (0.0f != remote_rotate_target) ? 1U : 0U;
            }
            else
            {
                remote_forward_target = 0.0f;
                remote_strafe_target = 0.0f;
                remote_rotate_target = 0.0f;
                remote_last_rotate_active = 0U;
                yaw_angle_target = control_get_current_yaw_angle();
                yaw_angle_hold_active = 0U;
                uwb_follow_reset();
                target_follow_reset();
            }
        }

        if(timer_20ms_flag)
        {
            timer_20ms_flag = 0;

            if((0U != g_wireless_control_state.control_enabled) && (0.0f != remote_rotate_target))
            {
                control_yaw_rate_loop_update(remote_rotate_target);
            }
            else
            {
                control_yaw_rate_loop_update(control_yaw_rate_target);
            }
        }

        if(timer_10ms_flag)
        {
            float imu_acc_x_g = 0.0f;
            float imu_acc_y_g = 0.0f;
            float imu_acc_z_g = 0.0f;
            float imu_gyro_x_dps = 0.0f;
            float imu_gyro_y_dps = 0.0f;
            float imu_gyro_z_dps = 0.0f;

            timer_10ms_flag = 0;

            encoder_update();
            odometer_update();
            telemetry_timestamp_count++;
            system_time_ms = telemetry_timestamp_count * 10U;

            if(0U != g_wireless_control_state.control_enabled)
            {
                control_cascade_speed_loop_update(remote_forward_target, remote_strafe_target);
            }
            else
            {
                control_cascade_stop();
            }

            imu_acc_x_g = g_imufilter_1000hz.accx;
            imu_acc_y_g = g_imufilter_1000hz.accy;
            imu_acc_z_g = g_imufilter_1000hz.accz;
            imu_gyro_x_dps = g_imufilter_1000hz.gyrox;
            imu_gyro_y_dps = g_imufilter_1000hz.gyroy;
            imu_gyro_z_dps = g_imufilter_1000hz.gyroz;
            wifi_justfloat((float)system_time_ms,
                           imu_acc_x_g,
                           imu_acc_y_g,
                           imu_acc_z_g,
                           imu_gyro_x_dps,
                           imu_gyro_y_dps,
                           imu_gyro_z_dps,
                           g_euler.roll,
                           g_euler.pitch,
                           g_euler.yaw,
                           g_odometer.strafe_distance,
                           g_odometer.forward_distance,
                           encoder_get_left_front_filtered_count(),
                           encoder_get_right_front_filtered_count(),
                           encoder_get_left_rear_filtered_count(),
                           encoder_get_right_rear_filtered_count());
        }
        wifi_core_Poll();

        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
