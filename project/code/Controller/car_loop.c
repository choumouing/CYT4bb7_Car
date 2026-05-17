/* 主循环调度模块
 *
 * 各频率任务职责：
 *   1000HZ：IMU数据更新/高自由度里程计陀螺采样/WiFi遥测
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

static uint32 s_telemetry_timestamp_count = 0U; // 1ms计数时间戳
static uint32 s_system_time_ms = 0U;            // 系统时间（ms，1ms递增）

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

#define WIFI_HIGH_ODO_UPDATE_DT_S                (0.01f)
#define WIFI_HIGH_ODO_STARTUP_HOLD_TICKS         (50U)
#define WIFI_HIGH_ODO_FORWARD_COUNT_PER_METER    (11287.0f)
#define WIFI_HIGH_ODO_STRAFE_COUNT_PER_METER_ABS (12100.0f)

#define WIFI_HIGH_ODO_DEG_TO_RAD                 (0.017453292519943295f)
#define WIFI_HIGH_ODO_PI                         (3.14159265358979323846f)
#define WIFI_HIGH_ODO_TWO_PI                     (6.28318530717958647692f)
#define WIFI_HIGH_ODO_EPSILON                    (1.0e-6f)

#define WIFI_HIGH_ODO_YAW_INTEGRAL_GAIN          (0.10f)
#define WIFI_HIGH_ODO_AXIS_SPEED_BREAK_MPS       (0.35f)
#define WIFI_HIGH_ODO_AXIS_SPEED_RANGE_MPS       (1.00f)
#define WIFI_HIGH_ODO_DUAL_RATIO_BREAK           (0.15f)
#define WIFI_HIGH_ODO_DUAL_RATIO_RANGE           (0.50f)
#define WIFI_HIGH_ODO_YAW_RATE_REF_DPS           (80.0f)

#define WIFI_HIGH_ODO_FWD_FP_BASE                (0.87647697f)
#define WIFI_HIGH_ODO_FWD_FN_BASE                (0.90167164f)
#define WIFI_HIGH_ODO_FWD_SP_BASE                (-0.09383003f)
#define WIFI_HIGH_ODO_FWD_SN_BASE                (-0.05782720f)
#define WIFI_HIGH_ODO_FWD_FP_AXIS                (-0.06856631f)
#define WIFI_HIGH_ODO_FWD_FN_AXIS                (-0.00171054f)
#define WIFI_HIGH_ODO_FWD_SP_AXIS                (0.33128398f)
#define WIFI_HIGH_ODO_FWD_SN_AXIS                (0.10364321f)
#define WIFI_HIGH_ODO_FWD_FP_DUAL                (-0.06949153f)
#define WIFI_HIGH_ODO_FWD_FN_DUAL                (0.23064157f)
#define WIFI_HIGH_ODO_FWD_SP_DUAL                (0.09346553f)
#define WIFI_HIGH_ODO_FWD_SN_DUAL                (-0.09552139f)
#define WIFI_HIGH_ODO_FWD_FP_YAWRATE             (1.44317809f)
#define WIFI_HIGH_ODO_FWD_FN_YAWRATE             (-1.01732248f)
#define WIFI_HIGH_ODO_FWD_SP_YAWRATE             (-0.51637106f)
#define WIFI_HIGH_ODO_FWD_SN_YAWRATE             (1.10436543f)

#define WIFI_HIGH_ODO_STRAFE_FP_BASE             (0.26635523f)
#define WIFI_HIGH_ODO_STRAFE_FN_BASE             (-0.08793618f)
#define WIFI_HIGH_ODO_STRAFE_SP_BASE             (0.67135251f)
#define WIFI_HIGH_ODO_STRAFE_SN_BASE             (1.16741196f)
#define WIFI_HIGH_ODO_STRAFE_FP_AXIS             (-0.41095207f)
#define WIFI_HIGH_ODO_STRAFE_FN_AXIS             (0.13405844f)
#define WIFI_HIGH_ODO_STRAFE_SP_AXIS             (0.42152586f)
#define WIFI_HIGH_ODO_STRAFE_SN_AXIS             (-0.21177676f)
#define WIFI_HIGH_ODO_STRAFE_FP_DUAL             (0.18528411f)
#define WIFI_HIGH_ODO_STRAFE_FN_DUAL             (-0.47628442f)
#define WIFI_HIGH_ODO_STRAFE_SP_DUAL             (-0.39473401f)
#define WIFI_HIGH_ODO_STRAFE_SN_DUAL             (0.24036549f)
#define WIFI_HIGH_ODO_STRAFE_FP_YAWRATE          (-0.10017021f)
#define WIFI_HIGH_ODO_STRAFE_FN_YAWRATE          (0.23634134f)
#define WIFI_HIGH_ODO_STRAFE_SP_YAWRATE          (-0.86158350f)
#define WIFI_HIGH_ODO_STRAFE_SN_YAWRATE          (-1.64313940f)

typedef struct
{
    float forward;
    float strafe;
} wifi_high_odo_vec2_t;

typedef struct
{
    float forward_distance;
    float strafe_distance;
    float travel_distance;
} wifi_high_odo_data_t;

typedef struct
{
    float yaw_delta_rad;
    float gyro_sum_dps[3];
    uint16 gyro_sample_count;
    uint16 startup_hold_ticks;
} wifi_high_odo_state_t;

static wifi_high_odo_data_t s_wifi_high_odo = {0.0f, 0.0f, 0.0f};
static wifi_high_odo_state_t s_wifi_high_odo_state;

static float wifi_high_odo_vec_norm(wifi_high_odo_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
}

static float wifi_high_odo_normalize_angle(float angle)
{
    while (angle > WIFI_HIGH_ODO_PI)
    {
        angle -= WIFI_HIGH_ODO_TWO_PI;
    }

    while (angle < -WIFI_HIGH_ODO_PI)
    {
        angle += WIFI_HIGH_ODO_TWO_PI;
    }

    return angle;
}

static float wifi_high_odo_positive_part(float value)
{
    return (value > 0.0f) ? value : 0.0f;
}

static float wifi_high_odo_negative_part(float value)
{
    return (value < 0.0f) ? value : 0.0f;
}

static float wifi_high_odo_axis_speed_weight(float speed_abs_mps)
{
    return car_math_clampf((speed_abs_mps - WIFI_HIGH_ODO_AXIS_SPEED_BREAK_MPS) /
                           WIFI_HIGH_ODO_AXIS_SPEED_RANGE_MPS,
                           0.0f,
                           1.0f);
}

static float wifi_high_odo_dual_axis_weight(wifi_high_odo_vec2_t velocity)
{
    float speed_norm;
    float dual_ratio;

    speed_norm = wifi_high_odo_vec_norm(velocity);
    dual_ratio = car_math_minf(car_math_absf(velocity.forward),
                               car_math_absf(velocity.strafe)) /
                 (speed_norm + WIFI_HIGH_ODO_EPSILON);
    return car_math_clampf((dual_ratio - WIFI_HIGH_ODO_DUAL_RATIO_BREAK) /
                           WIFI_HIGH_ODO_DUAL_RATIO_RANGE,
                           0.0f,
                           1.0f);
}

static wifi_high_odo_vec2_t wifi_high_odo_get_encoder_delta_count(void)
{
    wifi_high_odo_vec2_t count;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    count.forward = (left_front + right_front + left_rear + right_rear) * 0.25f;
    count.strafe = (-left_front + right_front + left_rear - right_rear) * 0.25f;
    return count;
}

static wifi_high_odo_vec2_t wifi_high_odo_get_encoder_velocity(wifi_high_odo_vec2_t delta_count)
{
    wifi_high_odo_vec2_t velocity;

    velocity.forward = delta_count.forward /
                       WIFI_HIGH_ODO_FORWARD_COUNT_PER_METER /
                       WIFI_HIGH_ODO_UPDATE_DT_S;
    velocity.strafe = delta_count.strafe /
                      WIFI_HIGH_ODO_STRAFE_COUNT_PER_METER_ABS /
                      WIFI_HIGH_ODO_UPDATE_DT_S;
    return velocity;
}

static wifi_high_odo_vec2_t wifi_high_odo_compensate_encoder_velocity(wifi_high_odo_vec2_t velocity,
                                                                      float yaw_rate_abs_dps)
{
    wifi_high_odo_vec2_t compensated;
    float fp;
    float fn;
    float sp;
    float sn;
    float fw;
    float sw;
    float dw;
    float yw;

    fp = wifi_high_odo_positive_part(velocity.forward);
    fn = wifi_high_odo_negative_part(velocity.forward);
    sp = wifi_high_odo_positive_part(velocity.strafe);
    sn = wifi_high_odo_negative_part(velocity.strafe);
    fw = wifi_high_odo_axis_speed_weight(car_math_absf(velocity.forward));
    sw = wifi_high_odo_axis_speed_weight(car_math_absf(velocity.strafe));
    dw = wifi_high_odo_dual_axis_weight(velocity);
    yw = car_math_clampf(yaw_rate_abs_dps / WIFI_HIGH_ODO_YAW_RATE_REF_DPS, 0.0f, 1.0f);

    compensated.forward =
        (WIFI_HIGH_ODO_FWD_FP_BASE * fp) +
        (WIFI_HIGH_ODO_FWD_FN_BASE * fn) +
        (WIFI_HIGH_ODO_FWD_SP_BASE * sp) +
        (WIFI_HIGH_ODO_FWD_SN_BASE * sn) +
        (WIFI_HIGH_ODO_FWD_FP_AXIS * fp * fw) +
        (WIFI_HIGH_ODO_FWD_FN_AXIS * fn * fw) +
        (WIFI_HIGH_ODO_FWD_SP_AXIS * sp * sw) +
        (WIFI_HIGH_ODO_FWD_SN_AXIS * sn * sw) +
        (WIFI_HIGH_ODO_FWD_FP_DUAL * fp * dw) +
        (WIFI_HIGH_ODO_FWD_FN_DUAL * fn * dw) +
        (WIFI_HIGH_ODO_FWD_SP_DUAL * sp * dw) +
        (WIFI_HIGH_ODO_FWD_SN_DUAL * sn * dw) +
        (WIFI_HIGH_ODO_FWD_FP_YAWRATE * fp * yw) +
        (WIFI_HIGH_ODO_FWD_FN_YAWRATE * fn * yw) +
        (WIFI_HIGH_ODO_FWD_SP_YAWRATE * sp * yw) +
        (WIFI_HIGH_ODO_FWD_SN_YAWRATE * sn * yw);

    compensated.strafe =
        (WIFI_HIGH_ODO_STRAFE_FP_BASE * fp) +
        (WIFI_HIGH_ODO_STRAFE_FN_BASE * fn) +
        (WIFI_HIGH_ODO_STRAFE_SP_BASE * sp) +
        (WIFI_HIGH_ODO_STRAFE_SN_BASE * sn) +
        (WIFI_HIGH_ODO_STRAFE_FP_AXIS * fp * fw) +
        (WIFI_HIGH_ODO_STRAFE_FN_AXIS * fn * fw) +
        (WIFI_HIGH_ODO_STRAFE_SP_AXIS * sp * sw) +
        (WIFI_HIGH_ODO_STRAFE_SN_AXIS * sn * sw) +
        (WIFI_HIGH_ODO_STRAFE_FP_DUAL * fp * dw) +
        (WIFI_HIGH_ODO_STRAFE_FN_DUAL * fn * dw) +
        (WIFI_HIGH_ODO_STRAFE_SP_DUAL * sp * dw) +
        (WIFI_HIGH_ODO_STRAFE_SN_DUAL * sn * dw) +
        (WIFI_HIGH_ODO_STRAFE_FP_YAWRATE * fp * yw) +
        (WIFI_HIGH_ODO_STRAFE_FN_YAWRATE * fn * yw) +
        (WIFI_HIGH_ODO_STRAFE_SP_YAWRATE * sp * yw) +
        (WIFI_HIGH_ODO_STRAFE_SN_YAWRATE * sn * yw);

    return compensated;
}

static void wifi_high_odo_clear_gyro_average(void)
{
    s_wifi_high_odo_state.gyro_sum_dps[0] = 0.0f;
    s_wifi_high_odo_state.gyro_sum_dps[1] = 0.0f;
    s_wifi_high_odo_state.gyro_sum_dps[2] = 0.0f;
    s_wifi_high_odo_state.gyro_sample_count = 0U;
}

static void wifi_high_odo_get_gyro_average_dps(float gyro_avg_dps[3])
{
    float inv_count;

    if ((gyro_avg_dps == 0) || (0U == s_wifi_high_odo_state.gyro_sample_count))
    {
        if (gyro_avg_dps != 0)
        {
            gyro_avg_dps[0] = 0.0f;
            gyro_avg_dps[1] = 0.0f;
            gyro_avg_dps[2] = 0.0f;
        }
        return;
    }

    inv_count = 1.0f / (float)s_wifi_high_odo_state.gyro_sample_count;
    gyro_avg_dps[0] = s_wifi_high_odo_state.gyro_sum_dps[0] * inv_count;
    gyro_avg_dps[1] = s_wifi_high_odo_state.gyro_sum_dps[1] * inv_count;
    gyro_avg_dps[2] = s_wifi_high_odo_state.gyro_sum_dps[2] * inv_count;
}

static void wifi_high_odo_reset(void)
{
    s_wifi_high_odo.forward_distance = 0.0f;
    s_wifi_high_odo.strafe_distance = 0.0f;
    s_wifi_high_odo.travel_distance = 0.0f;

    s_wifi_high_odo_state.yaw_delta_rad = 0.0f;
    s_wifi_high_odo_state.startup_hold_ticks = WIFI_HIGH_ODO_STARTUP_HOLD_TICKS;
    wifi_high_odo_clear_gyro_average();
}

static void wifi_high_odo_update_1000HZ(void)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    s_wifi_high_odo_state.gyro_sum_dps[0] += gyro_x_dps;
    s_wifi_high_odo_state.gyro_sum_dps[1] += gyro_y_dps;
    s_wifi_high_odo_state.gyro_sum_dps[2] += gyro_z_dps;
    if (s_wifi_high_odo_state.gyro_sample_count < 1000U)
    {
        s_wifi_high_odo_state.gyro_sample_count++;
    }
}

static void wifi_high_odo_update_100HZ(void)
{
    wifi_high_odo_vec2_t delta_count;
    wifi_high_odo_vec2_t raw_velocity;
    wifi_high_odo_vec2_t compensated_velocity;
    wifi_high_odo_vec2_t distance_delta;
    float gyro_avg_dps[3];
    float yaw_step_rad;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;

    delta_count = wifi_high_odo_get_encoder_delta_count();
    raw_velocity = wifi_high_odo_get_encoder_velocity(delta_count);

    wifi_high_odo_get_gyro_average_dps(gyro_avg_dps);
    wifi_high_odo_clear_gyro_average();

    compensated_velocity =
        wifi_high_odo_compensate_encoder_velocity(raw_velocity, car_math_absf(gyro_avg_dps[2]));

    if (s_wifi_high_odo_state.startup_hold_ticks > 0U)
    {
        s_wifi_high_odo.forward_distance = 0.0f;
        s_wifi_high_odo.strafe_distance = 0.0f;
        s_wifi_high_odo.travel_distance = 0.0f;
        s_wifi_high_odo_state.yaw_delta_rad = 0.0f;
        s_wifi_high_odo_state.startup_hold_ticks--;
        return;
    }

    yaw_step_rad = gyro_avg_dps[2] * WIFI_HIGH_ODO_DEG_TO_RAD * WIFI_HIGH_ODO_UPDATE_DT_S;
    s_wifi_high_odo_state.yaw_delta_rad =
        wifi_high_odo_normalize_angle(s_wifi_high_odo_state.yaw_delta_rad + yaw_step_rad);

    yaw_for_projection = s_wifi_high_odo_state.yaw_delta_rad * WIFI_HIGH_ODO_YAW_INTEGRAL_GAIN;
    cos_yaw = cosf(yaw_for_projection);
    sin_yaw = sinf(yaw_for_projection);
    distance_delta.forward = ((cos_yaw * compensated_velocity.forward) -
                              (sin_yaw * compensated_velocity.strafe)) *
                             WIFI_HIGH_ODO_UPDATE_DT_S;
    distance_delta.strafe = ((sin_yaw * compensated_velocity.forward) +
                             (cos_yaw * compensated_velocity.strafe)) *
                            WIFI_HIGH_ODO_UPDATE_DT_S;

    s_wifi_high_odo.forward_distance += distance_delta.forward;
    s_wifi_high_odo.strafe_distance += distance_delta.strafe;
    s_wifi_high_odo.travel_distance += wifi_high_odo_vec_norm(distance_delta);
}

static void car_loop_send_wifi_telemetry_1000HZ(void)
{
    wifi_justfloat((float)s_telemetry_timestamp_count,
                   g_odometer.forward_distance,
                   g_odometer.strafe_distance,
                   s_wifi_high_odo.forward_distance,
                   s_wifi_high_odo.strafe_distance,
                   (float)encoder_left_front.count_raw,
                   (float)encoder_right_front.count_raw,
                   (float)encoder_left_rear.count_raw,
                   (float)encoder_right_rear.count_raw,
                   g_euler.roll,
                   g_euler.pitch,
                   g_euler.yaw,
                   g_imufilter_1000hz.accx,
                   g_imufilter_1000hz.accy,
                   g_imufilter_1000hz.accz);
}

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
    wifi_high_odo_reset();
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

/* 1000HZ任务：IMU原始数据读取+滤波、高自由度里程计采样、WiFi遥测
 * 注意：此函数由主循环追赶1ms节拍调用，不能阻塞
 */
static void car_loop_1000HZ(void)
{
    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count;

    IMU_Update_1000HZ();
    control_yaw_rate_average_update_1000HZ();
    wifi_high_odo_update_1000HZ();
    car_loop_send_wifi_telemetry_1000HZ();
}

/* 100HZ任务：核心控制频率
 * 职责：
 *   1. 编码器读取+滤波 → 里程计积分
 *   2. 摄像头SPI通信 + 信标检测
 *   3. AirComm通信 + Air参数同步
 *   4. 菜单按键处理+屏幕刷新
 *   5. 四轮速度环PID（控制使能时）或安全停机
 */
static void car_loop_100HZ(void)
{
    encoder_update_100HZ();
    odometer_update_100HZ();
    wifi_high_odo_update_100HZ();
    camera_spi_update_100HZ(s_system_time_ms);
    if ((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        menu_air_stop_param_sync();
    }
    air_comm_car_update_100HZ();
    menu_air_command_update_100HZ();
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
