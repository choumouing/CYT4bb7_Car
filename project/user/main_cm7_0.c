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
volatile uint16 g_tick_1000HZ = 0U;
static uint32_t speed_profile_10ms_count = 0U;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    control_cascade_init();
    wifi_core_Init();
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
        if(timer_10ms_flag)
        {
            uint8_t yaw_rate_update_flag;
            uint32 irq_state = interrupt_global_disable();

            yaw_rate_update_flag = timer_20ms_flag;
            timer_10ms_flag = 0;
            timer_20ms_flag = 0;
            interrupt_global_enable(irq_state);

            float target_speed = 0.0f;

            if(speed_profile_10ms_count < 500U)
            {
                target_speed = 0.0f;
            }
            else if(speed_profile_10ms_count < 1000U)
            {
                target_speed = 1.0f;
            }
            else if(speed_profile_10ms_count < 1500U)
            {
                target_speed = 2.0f;
            }
            else if(speed_profile_10ms_count < 2000U)
            {
                target_speed = -1.0f;
            }
            else
            {
                target_speed = 0.0f;
            }
            speed_profile_10ms_count++;

            if(yaw_rate_update_flag)
            {
                control_yaw_rate_loop_update(target_speed);
            }

            encoder_update();
            control_cascade_speed_loop_update(0.0f, 0.0f);
            wifi_justfloat(control_yaw_rate_raw, control_yaw_rate_current,
                           yaw_rate_pid.p_term, yaw_rate_pid.i_term,
                           yaw_rate_pid.d_term, 0.0f,
                           0.0f, 0.0f,
                           0.0f, 0.0f,
                           0.0f, 0.0f,
                           0.0f, 0.0f,
                           0.0f, 0.0f);
        }
        wifi_core_Poll();

        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
