/* 主循环调度模块
 *
 * 各频率任务职责：
 *   1000HZ：IMU数据更新（不能阻塞，必须快速返回）
 *   100HZ：编码器/里程计/速度环/通信/菜单刷新（核心控制频率）
 *   50HZ：角速度环PID更新
 *   25HZ：UWB更新/遥控器/模式管理/航向保持
 *
 * 数据流：car_loop_init初始化→PIT中断置标志→car_loop_poll轮询执行
 */
#include "car_loop.h"

volatile uint8_t timer_100HZ_flag = 0U;
volatile uint8_t timer_50HZ_flag = 0U;
volatile uint8_t timer_25HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;

/* 全局控制目标（模式层写入，控制层读取） */
float car_forward_target = 0.0f;
float car_strafe_target = 0.0f;
float car_rotate_target = 0.0f;
uint8 car_control_enabled = 0U;       // 0=禁止控制（安全状态）
uint8 car_emergency_stop_active = 1U; // 默认紧急停（上电安全）

static uint32 s_telemetry_timestamp_count = 0U; // 100HZ滴答计数
static uint32 s_system_time_ms = 0U;            // 系统时间（ms，10ms递增）

volatile float g_air_tof1_height_mm;
volatile float g_air_tof2_height_mm;
volatile float g_air_tof3_height_mm;
volatile float g_air_tof4_height_mm;
volatile float g_air_imufilter_1000hz_accx;
volatile float g_air_imufilter_1000hz_accy;
volatile float g_air_imufilter_1000hz_accz;
volatile float g_air_imufilter_1000hz_gyrox;
volatile float g_air_imufilter_1000hz_gyroy;
volatile float g_air_imufilter_1000hz_gyroz;

static void on_air_data(const float *data, uint8 count)
{
    if (count >= 10)
    {
        g_air_tof1_height_mm = data[0];
        g_air_tof2_height_mm = data[1];
        g_air_tof3_height_mm = data[2];
        g_air_tof4_height_mm = data[3];
        g_air_imufilter_1000hz_accx  = data[4];
        g_air_imufilter_1000hz_accy  = data[5];
        g_air_imufilter_1000hz_accz  = data[6];
        g_air_imufilter_1000hz_gyrox = data[7];
        g_air_imufilter_1000hz_gyroy = data[8];
        g_air_imufilter_1000hz_gyroz = data[9];
    }
}

static void car_loop_runtime_reset(void)
{
    timer_100HZ_flag = 0U;
    timer_50HZ_flag = 0U;
    timer_25HZ_flag = 0U;
    g_tick_1000HZ = 0U;

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_rotate_target = 0.0f;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    menu_init();
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    camera_spi_init();
    beacon_detection_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    control_cascade_init();
    uart_receiver_init();
    sbus_init();
    wireless_control_init();
    car_start_sbus_init();
    car_mode_init();
    wifi_core_Init();
    ALX_AOA_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    pit_init(PIT_CH0, 1000);
}

/* 1000HZ任务：IMU原始数据读取+滤波
 * 注意：此函数在PIT中断中调用，不能阻塞，不能操作Flash/SPI
 */
static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
}

/* 100HZ任务：核心控制频率
 * 职责：
 *   1. 编码器读取+滤波 → 里程计积分
 *   2. 摄像头SPI通信 + 信标检测
 *   3. AirComm通信 + Air参数同步
 *   4. 菜单按键处理+屏幕刷新
 *   5. 四轮速度环PID（控制使能时）或安全停机
 *   6. WiFi调试数据发送
 */
static void car_loop_100HZ(void)
{
    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    camera_spi_update_100HZ(s_system_time_ms);
    if ((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        menu_air_stop_param_sync();
    }
    air_comm_car_update_100HZ();
    beacon_detection_update_100HZ();

    if ((car_control_enabled == 0U) || (car_emergency_stop_active != 0U))
    {
        menu_air_update_100HZ();
        menu_update_100HZ();
    }
    else
    {
        menu_discard_key_events();
    }

    /* 控制使能：执行速度环；否则安全停机 */
    if (0U != car_control_enabled)
    {
        control_cascade_speed_loop_update_100HZ(car_forward_target, car_strafe_target);
    }
    else
    {
        control_cascade_stop();
        control_yaw_hold_reset();
    }


    

    float car_data[10];
    car_data[0] = encoder_left_front.count_raw;
    car_data[1] = encoder_right_front.count_raw;
    car_data[2] = encoder_left_rear.count_raw;
    car_data[3] = encoder_right_rear.count_raw;
    car_data[4] = g_imufilter_1000hz.accx;
    car_data[5] = g_imufilter_1000hz.accy;
    car_data[6] = g_imufilter_1000hz.accz;
    car_data[7] = g_imufilter_1000hz.gyrox;
    car_data[8] = g_imufilter_1000hz.gyroy;
    car_data[9] = g_imufilter_1000hz.gyroz;
    air_comm_send_run_data(car_data, 10);

    wifi_justfloat(g_air_tof1_height_mm,
                   g_air_tof2_height_mm,
                   g_air_tof3_height_mm,
                   g_air_tof4_height_mm,
                   g_air_imufilter_1000hz_accx,
                   g_air_imufilter_1000hz_accy,
                   g_air_imufilter_1000hz_accz,
                   g_air_imufilter_1000hz_gyrox,
                   g_air_imufilter_1000hz_gyroy,
                   g_air_imufilter_1000hz_gyroz);
}

/* 50HZ任务：角速度环PID更新
 * 逻辑：
 *   - 控制使能 + 有旋转输入：直接用遥控器角速度（rad/s）
 *   - 控制使能 + 无旋转输入：用角度环输出的角速度目标（航向保持）
 *   - 控制禁止：输出0（安全）
 * 注意：此任务独立于速度环（100HZ），两者频率不同
 */
static void car_loop_50HZ(void)
{
    if (0U != car_control_enabled)
    {
        if (0.0f != car_rotate_target)
        {
            control_yaw_rate_loop_update_50HZ(car_rotate_target);
        }
        else
        {
            control_yaw_rate_loop_update_50HZ(control_yaw_rate_target);
        }
    }
    else
    {
        control_yaw_rate_loop_update_50HZ(0.0f);
    }
}

/* 25HZ任务：上层逻辑频率
 * 职责：
 *   1. UWB AoA数据更新
 *   2. SBUS遥控器数据解析
 *   3. 无线控制状态机（wireless_control）
 *   4. 小车启动/模式状态机（car_start_sbus）
 *   5. 模式分发（car_mode_update → mode0/1/2）
 *   6. 航向保持逻辑（松开旋转摇杆时锁定朝向）
 */
static void car_loop_25HZ(void)
{
    ALX_AOA_Update_25HZ(s_system_time_ms);
    sbus_update_25HZ();
    wireless_control_update_25HZ();
    car_start_sbus_update_25HZ();
    car_mode_update_25HZ(s_system_time_ms);

    if (0U != car_control_enabled)
    {
        control_yaw_hold_update_25HZ(car_rotate_target);
    }
    else
    {
        control_yaw_hold_reset();
    }
}

/* 主循环轮询入口
 * 执行顺序：1000HZ（追赶）→ 25HZ → 50HZ → 100HZ
 * 注意：1000HZ有防堆积保护（最多追100次），防止IMU落后太多时卡死
 * 最后处理非周期任务：WiFi轮询、摄像头SPI轮询、AirComm轮询
 */
void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    /* 1000HZ任务：在主循环中追赶中断累积的tick */
    while ((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_loop_1000HZ();
        imu_tick_guard++;
    }

    if (timer_25HZ_flag)
    {
        timer_25HZ_flag = 0U;
        car_loop_25HZ();
    }

    if (timer_50HZ_flag)
    {
        timer_50HZ_flag = 0U;
        car_loop_50HZ();
    }

    if (timer_100HZ_flag)
    {
        timer_100HZ_flag = 0U;
        car_loop_100HZ();
    }

    wifi_core_Poll();
    camera_spi_poll();
    air_comm_car_poll();
}
