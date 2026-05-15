#include "ALX_AOA.h"
#include "math.h"
#include "string.h"

/*
Position frame layout from vendor STM32F1 demo, big-endian wire fields:
FF FF FF FF | PacketLength | SequenceID | RequestCommand | VersionID |
AnchorID | TagID | Distance | Azimuth | Elevation | TagStatus |
BatchSn | Reserve | Xor

RequestCommand is at byte offset 8, not 6.
*/

#define ALX_AOA_OFFSET_PACKET_LENGTH        (4)
#define ALX_AOA_OFFSET_COMMAND              (8)
#define ALX_AOA_OFFSET_VERSION              (10)
#define ALX_AOA_OFFSET_ANCHOR_ID            (12)
#define ALX_AOA_OFFSET_TAG_ID               (16)
#define ALX_AOA_OFFSET_DISTANCE             (20)
#define ALX_AOA_OFFSET_AZIMUTH              (24)
#define ALX_AOA_OFFSET_ELEVATION            (26)
#define ALX_AOA_COMMAND_FIELD_SIZE          (2)
#define ALX_AOA_POSITION_VERSION            (0x0102)
#define ALX_AOA_ANGLE_SCALE                 (1.0f)
#define ALX_AOA_PI                          (3.14159265358979323846f)
#define ALX_AOA_DEG_TO_RAD                  (ALX_AOA_PI / 180.0f)

static volatile uint16 alx_aoa_rx_head = 0;
static volatile uint16 alx_aoa_rx_tail = 0;
static volatile uint8  alx_aoa_rx_buffer[ALX_AOA_RX_BUFFER_SIZE];

static uint8  alx_aoa_frame_buffer[ALX_AOA_MAX_FRAME_SIZE];
static uint16 alx_aoa_frame_length = 0;

static ALX_AOA_Position_t  alx_aoa_latest_position;

float alx_aoa_x_cm = 0.0f;
float alx_aoa_y_cm = 0.0f;
float alx_aoa_debug_accepted = 0.0f;
float alx_aoa_debug_jump_cm = 0.0f;
float alx_aoa_debug_gate_cm = 0.0f;
uint32 alx_aoa_debug_rx_bytes = 0;
uint32 alx_aoa_debug_rx_overflow = 0;
uint32 alx_aoa_debug_frame_headers = 0;
uint32 alx_aoa_debug_frame_ok = 0;
uint32 alx_aoa_debug_frame_error = 0;

static float  alx_aoa_filt_x_cm     = 0.0f;
static float  alx_aoa_filt_y_cm     = 0.0f;
static float  alx_aoa_raw_x_hist[ALX_AOA_MEDIAN_SIZE] = {0.0f};
static float  alx_aoa_raw_y_hist[ALX_AOA_MEDIAN_SIZE] = {0.0f};
static uint8  alx_aoa_raw_hist_count = 0;
static uint8  alx_aoa_raw_hist_index = 0;
static uint8  alx_aoa_filt_init     = 0;
static float  alx_aoa_last_obs_x_cm = 0.0f;
static float  alx_aoa_last_obs_y_cm = 0.0f;
static uint8  alx_aoa_last_obs_init = 0;
static float  alx_aoa_reacquire_x_cm = 0.0f;
static float  alx_aoa_reacquire_y_cm = 0.0f;
static uint8  alx_aoa_reacquire_count = 0;
static uint32 alx_aoa_last_filt_ms  = 0;

static uint16 alx_aoa_next_index (uint16 index)
{
    index ++;
    if(ALX_AOA_RX_BUFFER_SIZE <= index)
    {
        index = 0;
    }
    return index;
}

static uint16 alx_aoa_rx_count (void)
{
    uint16 head = alx_aoa_rx_head;
    uint16 tail = alx_aoa_rx_tail;

    if(head >= tail)
    {
        return (uint16)(head - tail);
    }

    return (uint16)(ALX_AOA_RX_BUFFER_SIZE - tail + head);
}

static uint8 alx_aoa_pop_byte (uint8 *dat)
{
    if(alx_aoa_rx_head == alx_aoa_rx_tail)
    {
        return 0;
    }

    *dat = alx_aoa_rx_buffer[alx_aoa_rx_tail];
    alx_aoa_rx_tail = alx_aoa_next_index(alx_aoa_rx_tail);

    return 1;
}

static uint16 alx_aoa_read_be16 (const uint8 *dat)
{
    return (uint16)(((uint16)dat[0] << 8) | dat[1]);
}

static uint32 alx_aoa_read_be32 (const uint8 *dat)
{
    return (((uint32)dat[0] << 24) |
            ((uint32)dat[1] << 16) |
            ((uint32)dat[2] << 8 ) |
            ((uint32)dat[3]));
}

static int16 alx_aoa_read_s16 (const uint8 *dat)
{
    uint16 value = alx_aoa_read_be16(dat);

    if(0 != (value & 0x8000))
    {
        return (int16)(-1 - (int16)(0xFFFF - value));
    }

    return (int16)value;
}

static int32 alx_aoa_float_to_s32 (float value)
{
    if(value >= 0.0f)
    {
        return (int32)(value + 0.5f);
    }

    return (int32)(value - 0.5f);
}

static float alx_aoa_angle_to_rad (int16 angle)
{
    return (((float)angle) / ALX_AOA_ANGLE_SCALE) * ALX_AOA_DEG_TO_RAD;
}

static float alx_aoa_median (const float *data, uint8 count)
{
    float sorted[ALX_AOA_MEDIAN_SIZE];
    uint8 i;
    uint8 j;

    if(0 == count)
    {
        return 0.0f;
    }

    if(count > ALX_AOA_MEDIAN_SIZE)
    {
        count = ALX_AOA_MEDIAN_SIZE;
    }

    for(i = 0; i < count; i ++)
    {
        sorted[i] = data[i];
    }

    for(i = 1; i < count; i ++)
    {
        float value = sorted[i];

        j = i;
        while((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j --;
        }
        sorted[j] = value;
    }

    return sorted[count / 2U];
}

static void alx_aoa_calculate_xy (ALX_AOA_Position_t *position)
{
    float azimuth_rad = alx_aoa_angle_to_rad(position->azimuth_deg);
    float elevation_rad = alx_aoa_angle_to_rad(position->elevation_deg);
    float distance = (float)position->distance_cm;
    float uwb_y_from_elevation;
    float x_uwb;
    float y_uwb;

    uwb_y_from_elevation = -distance * sinf(elevation_rad);
    x_uwb = distance * cosf(elevation_rad) * sinf(azimuth_rad);
    y_uwb = uwb_y_from_elevation;

    position->x_cm = alx_aoa_float_to_s32(x_uwb);
    position->y_cm = alx_aoa_float_to_s32(y_uwb);
    position->z_cm = 0;
}

static uint8 alx_aoa_raw_valid (const ALX_AOA_Position_t *p)
{
    if(0 == p->distance_cm)
    {
        return 0;
    }

    if(255 == p->azimuth_deg)
    {
        return 0;
    }

    if(p->elevation_deg > ALX_AOA_EL_MAX_DEG || p->elevation_deg < -ALX_AOA_EL_MAX_DEG)
    {
        return 0;
    }

    if(p->elevation_deg >= ALX_AOA_EL_GATE_DEG || p->elevation_deg <= -ALX_AOA_EL_GATE_DEG)
    {
        return 0;
    }

    if(p->distance_cm < ALX_AOA_DIST_MIN_CM || p->distance_cm > ALX_AOA_DIST_MAX_CM)
    {
        return 0;
    }

    return 1;
}

static void alx_aoa_raw_history_reset (float raw_x, float raw_y)
{
    uint8 i;

    for(i = 0; i < ALX_AOA_MEDIAN_SIZE; i ++)
    {
        alx_aoa_raw_x_hist[i] = raw_x;
        alx_aoa_raw_y_hist[i] = raw_y;
    }

    alx_aoa_raw_hist_count = ALX_AOA_MEDIAN_SIZE;
    alx_aoa_raw_hist_index = 0;
}

static void alx_aoa_raw_history_push (float raw_x, float raw_y)
{
    alx_aoa_raw_x_hist[alx_aoa_raw_hist_index] = raw_x;
    alx_aoa_raw_y_hist[alx_aoa_raw_hist_index] = raw_y;

    alx_aoa_raw_hist_index ++;
    if(alx_aoa_raw_hist_index >= ALX_AOA_MEDIAN_SIZE)
    {
        alx_aoa_raw_hist_index = 0;
    }

    if(alx_aoa_raw_hist_count < ALX_AOA_MEDIAN_SIZE)
    {
        alx_aoa_raw_hist_count ++;
    }
}

static void alx_aoa_raw_history_get_median (float *x_cm, float *y_cm)
{
    uint8 count = alx_aoa_raw_hist_count;

    if(0 == count)
    {
        *x_cm = 0.0f;
        *y_cm = 0.0f;
        return;
    }

    if(count > ALX_AOA_MEDIAN_SIZE)
    {
        count = ALX_AOA_MEDIAN_SIZE;
    }

    *x_cm = alx_aoa_median(alx_aoa_raw_x_hist, count);
    *y_cm = alx_aoa_median(alx_aoa_raw_y_hist, count);
}

static float alx_aoa_distance_xy (float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;

    return sqrtf((dx * dx) + (dy * dy));
}

static void alx_aoa_reacquire_reset (void)
{
    alx_aoa_reacquire_x_cm = 0.0f;
    alx_aoa_reacquire_y_cm = 0.0f;
    alx_aoa_reacquire_count = 0;
}

static void alx_aoa_accept_observation (float obs_x, float obs_y)
{
    alx_aoa_last_obs_x_cm = obs_x;
    alx_aoa_last_obs_y_cm = obs_y;
    alx_aoa_last_obs_init = 1;
    alx_aoa_reacquire_reset();
}

static uint8 alx_aoa_reacquire_update (float obs_x, float obs_y, float *target_x, float *target_y)
{
    float stable_jump;

    if((0U == alx_aoa_reacquire_count) ||
       (alx_aoa_distance_xy(alx_aoa_reacquire_x_cm,
                            alx_aoa_reacquire_y_cm,
                            obs_x,
                            obs_y) > ALX_AOA_REACQUIRE_STABLE_CM))
    {
        alx_aoa_reacquire_x_cm = obs_x;
        alx_aoa_reacquire_y_cm = obs_y;
        alx_aoa_reacquire_count = 1;
        return 0;
    }

    stable_jump = (float)alx_aoa_reacquire_count;
    alx_aoa_reacquire_x_cm =
        ((alx_aoa_reacquire_x_cm * stable_jump) + obs_x) / (stable_jump + 1.0f);
    alx_aoa_reacquire_y_cm =
        ((alx_aoa_reacquire_y_cm * stable_jump) + obs_y) / (stable_jump + 1.0f);

    if(alx_aoa_reacquire_count < ALX_AOA_REACQUIRE_COUNT)
    {
        alx_aoa_reacquire_count ++;
    }

    if(alx_aoa_reacquire_count >= ALX_AOA_REACQUIRE_COUNT)
    {
        *target_x = alx_aoa_reacquire_x_cm;
        *target_y = alx_aoa_reacquire_y_cm;
        alx_aoa_accept_observation(*target_x, *target_y);
        return 1;
    }

    return 0;
}

static uint8 alx_aoa_select_observation (float obs_x, float obs_y, float *target_x, float *target_y)
{
    float jump_2d;

    if(0U == alx_aoa_last_obs_init)
    {
        *target_x = obs_x;
        *target_y = obs_y;
        alx_aoa_accept_observation(obs_x, obs_y);
        return 1;
    }

    jump_2d = alx_aoa_distance_xy(alx_aoa_last_obs_x_cm,
                                  alx_aoa_last_obs_y_cm,
                                  obs_x,
                                  obs_y);

    if(jump_2d <= ALX_AOA_OBS_STEP_GATE_CM)
    {
        *target_x = obs_x;
        *target_y = obs_y;
        alx_aoa_accept_observation(obs_x, obs_y);
        return 1;
    }

    if(0U != alx_aoa_reacquire_update(obs_x, obs_y, target_x, target_y))
    {
        return 1;
    }

    *target_x = alx_aoa_last_obs_x_cm;
    *target_y = alx_aoa_last_obs_y_cm;
    return 0;
}

static float alx_aoa_lpf_alpha (float dt)
{
    float tau;
    float alpha;

    tau = 1.0f / (2.0f * ALX_AOA_PI * ALX_AOA_LPF_CUTOFF_HZ);
    alpha = dt / (tau + dt);

    if(alpha < 0.0f)
    {
        alpha = 0.0f;
    }

    if(alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    return alpha;
}

static void alx_aoa_apply_output_slew_xy (float target_x, float target_y, float dt)
{
    float dx;
    float dy;
    float step;
    float max_step;

    dx = target_x - alx_aoa_filt_x_cm;
    dy = target_y - alx_aoa_filt_y_cm;
    step = sqrtf((dx * dx) + (dy * dy));
    max_step = ALX_AOA_OUTPUT_SLEW_CM_S * dt;

    if((step > max_step) && (step > 0.0f) && (max_step > 0.0f))
    {
        target_x = alx_aoa_filt_x_cm + ((dx / step) * max_step);
        target_y = alx_aoa_filt_y_cm + ((dy / step) * max_step);
    }

    alx_aoa_filt_x_cm = target_x;
    alx_aoa_filt_y_cm = target_y;
}

static void alx_aoa_lpf_update_xy (float obs_x, float obs_y, float dt)
{
    float alpha;
    float target_x;
    float target_y;

    alpha = alx_aoa_lpf_alpha(dt);
    target_x = alx_aoa_filt_x_cm + (alpha * (obs_x - alx_aoa_filt_x_cm));
    target_y = alx_aoa_filt_y_cm + (alpha * (obs_y - alx_aoa_filt_y_cm));
    alx_aoa_apply_output_slew_xy(target_x, target_y, dt);
}

static void alx_aoa_filter_xy (const ALX_AOA_Position_t *p, uint32 now_ms)
{
    float raw_x;
    float raw_y;
    float obs_x;
    float obs_y;
    float target_x;
    float target_y;
    float dt;
    uint8 obs_ok;

    raw_x = (float)p->x_cm;
    raw_y = (float)p->y_cm;

    obs_ok = alx_aoa_raw_valid(p);
    alx_aoa_debug_accepted = 0.0f;
    alx_aoa_debug_jump_cm = 0.0f;
    alx_aoa_debug_gate_cm = 0.0f;

    if(!alx_aoa_filt_init)
    {
        if(obs_ok)
        {
            alx_aoa_raw_history_reset(raw_x, raw_y);
            alx_aoa_filt_x_cm     = raw_x;
            alx_aoa_filt_y_cm     = raw_y;
            alx_aoa_filt_init     = 1;
            alx_aoa_accept_observation(raw_x, raw_y);
            alx_aoa_last_filt_ms  = now_ms;
            alx_aoa_debug_accepted = 1.0f;
        }

        alx_aoa_x_cm = alx_aoa_filt_x_cm;
        alx_aoa_y_cm = alx_aoa_filt_y_cm;
        return;
    }

    if(0 != alx_aoa_last_filt_ms)
    {
        dt = (float)((uint32)(now_ms - alx_aoa_last_filt_ms)) * 0.001f;
        if(dt < 0.005f)
        {
            dt = 0.005f;
        }
        if(dt > 0.100f)
        {
            dt = 0.100f;
        }
    }
    else
    {
        dt = 0.02f;
    }

    alx_aoa_last_filt_ms = now_ms;

    if(0 == obs_ok)
    {
        alx_aoa_reacquire_reset();
        alx_aoa_x_cm = alx_aoa_filt_x_cm;
        alx_aoa_y_cm = alx_aoa_filt_y_cm;
        return;
    }

    alx_aoa_raw_history_push(raw_x, raw_y);
    alx_aoa_raw_history_get_median(&obs_x, &obs_y);

    if(0U != alx_aoa_select_observation(obs_x, obs_y, &target_x, &target_y))
    {
        alx_aoa_debug_accepted = 1.0f;
    }
    alx_aoa_debug_jump_cm = alx_aoa_distance_xy(alx_aoa_last_obs_x_cm,
                                                alx_aoa_last_obs_y_cm,
                                                obs_x,
                                                obs_y);
    alx_aoa_debug_gate_cm = ALX_AOA_OBS_STEP_GATE_CM;
    alx_aoa_lpf_update_xy(target_x, target_y, dt);

    alx_aoa_x_cm = alx_aoa_filt_x_cm;
    alx_aoa_y_cm = alx_aoa_filt_y_cm;
}

static void alx_aoa_drop_frame_bytes (uint16 length)
{
    if(length >= alx_aoa_frame_length)
    {
        alx_aoa_frame_length = 0;
        return;
    }

    memmove(&alx_aoa_frame_buffer[0],
            &alx_aoa_frame_buffer[length],
            (uint32)(alx_aoa_frame_length - length));
    alx_aoa_frame_length = (uint16)(alx_aoa_frame_length - length);
}

static void alx_aoa_keep_header_prefix (void)
{
    uint16 keep = 0;
    uint16 i;

    while((keep < alx_aoa_frame_length) &&
          (keep < 3) &&
          (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[alx_aoa_frame_length - 1 - keep]))
    {
        keep ++;
    }

    if(keep != alx_aoa_frame_length)
    {
        for(i = 0; i < keep; i ++)
        {
            alx_aoa_frame_buffer[i] = alx_aoa_frame_buffer[alx_aoa_frame_length - keep + i];
        }
        alx_aoa_frame_length = keep;
    }
}

static uint8 alx_aoa_has_header_at (uint16 index)
{
    return ((ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 1]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 2]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 3]));
}

static uint8 alx_aoa_seek_header (void)
{
    uint16 index;

    if(alx_aoa_frame_length < 4)
    {
        alx_aoa_keep_header_prefix();
        return 0;
    }

    for(index = 0; (uint16)(index + 3) < alx_aoa_frame_length; index ++)
    {
        if(alx_aoa_has_header_at(index))
        {
            if(0 != index)
            {
                alx_aoa_drop_frame_bytes(index);
            }
            return 1;
        }
    }

    alx_aoa_keep_header_prefix();
    return 0;
}

static void alx_aoa_parse_position (const uint8 *frame, uint32 now_ms)
{
    alx_aoa_latest_position.base_id        = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_ANCHOR_ID]);
    alx_aoa_latest_position.tag_id         = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_TAG_ID]);
    alx_aoa_latest_position.distance_cm    = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_DISTANCE]);
    alx_aoa_latest_position.azimuth_deg    = alx_aoa_read_s16(&frame[ALX_AOA_OFFSET_AZIMUTH]);
    alx_aoa_latest_position.elevation_deg  = alx_aoa_read_s16(&frame[ALX_AOA_OFFSET_ELEVATION]);
    alx_aoa_calculate_xy(&alx_aoa_latest_position);
    alx_aoa_filter_xy(&alx_aoa_latest_position, now_ms);
    alx_aoa_latest_position.last_position_ms = now_ms;
    alx_aoa_latest_position.valid            = 1;
}

static uint8 alx_aoa_parse_stream (uint32 now_ms)
{
    uint16 packet_length;
    uint16 command;
    uint8 parsed_position = 0;

    while(1)
    {
        if(!alx_aoa_seek_header())
        {
            break;
        }

        if(alx_aoa_frame_length < (ALX_AOA_OFFSET_COMMAND + ALX_AOA_COMMAND_FIELD_SIZE))
        {
            break;
        }

        packet_length = alx_aoa_read_be16(&alx_aoa_frame_buffer[ALX_AOA_OFFSET_PACKET_LENGTH]);
        command = alx_aoa_read_be16(&alx_aoa_frame_buffer[ALX_AOA_OFFSET_COMMAND]);
        alx_aoa_debug_frame_headers ++;

        if(ALX_AOA_CMD_POSITION == command)
        {
            if(alx_aoa_frame_length < ALX_AOA_POSITION_FRAME_SIZE)
            {
                break;
            }

            if(ALX_AOA_POSITION_FRAME_SIZE != packet_length)
            {
                alx_aoa_debug_frame_error ++;
                alx_aoa_drop_frame_bytes(1);
                continue;
            }

            if(ALX_AOA_POSITION_VERSION != alx_aoa_read_be16(&alx_aoa_frame_buffer[ALX_AOA_OFFSET_VERSION]))
            {
                alx_aoa_debug_frame_error ++;
                alx_aoa_drop_frame_bytes(ALX_AOA_POSITION_FRAME_SIZE);
                continue;
            }

            alx_aoa_parse_position(alx_aoa_frame_buffer, now_ms);
            alx_aoa_debug_frame_ok ++;
            parsed_position = 1;
            alx_aoa_drop_frame_bytes(ALX_AOA_POSITION_FRAME_SIZE);
        }
        else
        {
            alx_aoa_debug_frame_error ++;
            alx_aoa_drop_frame_bytes(1);
        }
    }

    return parsed_position;
}

static uint8 alx_aoa_feed_parser (uint8 dat, uint32 now_ms)
{
    if(ALX_AOA_MAX_FRAME_SIZE <= alx_aoa_frame_length)
    {
        alx_aoa_drop_frame_bytes(1);
    }

    alx_aoa_frame_buffer[alx_aoa_frame_length] = dat;
    alx_aoa_frame_length ++;

    return alx_aoa_parse_stream(now_ms);
}

void ALX_AOA_Reset (void)
{
    alx_aoa_rx_head = 0;
    alx_aoa_rx_tail = 0;
    alx_aoa_frame_length = 0;
    memset((void *)alx_aoa_rx_buffer, 0, sizeof(alx_aoa_rx_buffer));
    memset(&alx_aoa_frame_buffer[0], 0, sizeof(alx_aoa_frame_buffer));
    memset(&alx_aoa_latest_position, 0, sizeof(alx_aoa_latest_position));

    alx_aoa_x_cm          = 0.0f;
    alx_aoa_y_cm          = 0.0f;
    alx_aoa_debug_accepted = 0.0f;
    alx_aoa_debug_jump_cm  = 0.0f;
    alx_aoa_debug_gate_cm  = 0.0f;
    alx_aoa_debug_rx_bytes = 0;
    alx_aoa_debug_rx_overflow = 0;
    alx_aoa_debug_frame_headers = 0;
    alx_aoa_debug_frame_ok = 0;
    alx_aoa_debug_frame_error = 0;
    alx_aoa_filt_x_cm     = 0.0f;
    alx_aoa_filt_y_cm     = 0.0f;
    memset(&alx_aoa_raw_x_hist[0], 0, sizeof(alx_aoa_raw_x_hist));
    memset(&alx_aoa_raw_y_hist[0], 0, sizeof(alx_aoa_raw_y_hist));
    alx_aoa_raw_hist_count = 0;
    alx_aoa_raw_hist_index = 0;
    alx_aoa_filt_init     = 0;
    alx_aoa_last_obs_x_cm = 0.0f;
    alx_aoa_last_obs_y_cm = 0.0f;
    alx_aoa_last_obs_init = 0;
    alx_aoa_reacquire_reset();
    alx_aoa_last_filt_ms  = 0;
}

void ALX_AOA_Init (void)
{
    ALX_AOA_Reset();
    uart_init(ALX_AOA_UART_INDEX, ALX_AOA_UART_BAUDRATE, ALX_AOA_UART_TX_PIN, ALX_AOA_UART_RX_PIN);
    uart_rx_interrupt(ALX_AOA_UART_INDEX, 1);
}

void ALX_AOA_InputByte (uint8 dat)
{
    uint16 next = alx_aoa_next_index(alx_aoa_rx_head);

    if(next == alx_aoa_rx_tail)
    {
        alx_aoa_debug_rx_overflow ++;
        return;
    }

    alx_aoa_rx_buffer[alx_aoa_rx_head] = dat;
    alx_aoa_rx_head = next;
    alx_aoa_debug_rx_bytes ++;
}

uint8 ALX_AOA_Update (uint32 now_ms)
{
    uint16 pending = alx_aoa_rx_count();
    uint8 dat;
    uint8 parsed_position = 0;

    while(0 != pending)
    {
        if(!alx_aoa_pop_byte(&dat))
        {
            break;
        }

        if(alx_aoa_feed_parser(dat, now_ms))
        {
            parsed_position = 1;
        }

        pending --;
    }

    return parsed_position;
}

uint8 ALX_AOA_GetLatest (ALX_AOA_Position_t *data)
{
    if((0 == data) || (!alx_aoa_latest_position.valid))
    {
        return 0;
    }

    *data = alx_aoa_latest_position;
    return 1;
}

uint8 ALX_AOA_GetFilteredXY (float *x_cm, float *y_cm)
{
    if((0 == x_cm) || (0 == y_cm))
    {
        return 0;
    }

    *x_cm = alx_aoa_x_cm;
    *y_cm = alx_aoa_y_cm;

    return alx_aoa_filt_init;
}

uint8 ALX_AOA_IsTagOnline (uint32 now_ms, uint32 timeout_ms)
{
    if(!alx_aoa_latest_position.valid)
    {
        return 0;
    }

    return (((uint32)(now_ms - alx_aoa_latest_position.last_position_ms)) <= timeout_ms);
}
