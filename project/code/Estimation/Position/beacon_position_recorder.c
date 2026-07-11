#include "beacon_position_recorder.h"
#include "Controller/car_loop.h"

#define BEACON_POSITION_RECORDER_FLASH_MAGIC   (0x42505243UL)
#define BEACON_POSITION_RECORDER_FLASH_VERSION (2U)

#define BEACON_MAP_CELL_PIXELS                 (16)
#define BEACON_MAP_LEFT_CELLS                  (3)
#define BEACON_MAP_RIGHT_CELLS                 (4)
#define BEACON_MAP_UP_CELLS                    (7)
#define BEACON_MAP_START_X                     (8)
#define BEACON_MAP_BOTTOM_Y                    (120)
#define BEACON_MAP_ORIGIN_X                    \
    (BEACON_MAP_START_X + (BEACON_MAP_LEFT_CELLS * BEACON_MAP_CELL_PIXELS))
#define BEACON_MAP_TOP_Y                       \
    (BEACON_MAP_BOTTOM_Y - (BEACON_MAP_UP_CELLS * BEACON_MAP_CELL_PIXELS))
#define BEACON_MAP_END_X                       \
    (BEACON_MAP_ORIGIN_X + (BEACON_MAP_RIGHT_CELLS * BEACON_MAP_CELL_PIXELS))

#if (BEACON_POSITION_RECORDER_FLASH_PAGE >= FLASH_PAGE_NUM)
#error "Beacon position recorder Flash page exceeds Work Flash range"
#endif

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 point_count;
    uint8 full;
    uint8 reserved[3];
    float position[BEACON_POSITION_RECORDER_AXIS_NUM];
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
    uint32 checksum;
} beacon_position_recorder_flash_data_t;

typedef char beacon_position_recorder_flash_data_must_be_word_aligned[
    ((sizeof(beacon_position_recorder_flash_data_t) % sizeof(uint32)) == 0U) ? 1 : -1];
typedef char beacon_position_recorder_flash_data_must_fit_page[
    (sizeof(beacon_position_recorder_flash_data_t) <= FLASH_PAGE_SIZE) ? 1 : -1];

beacon_position_recorder_t g_beacon_position_recorder = {0};

static uint8 s_last_ch8_high;
static uint8 s_ch8_synced;
static uint8 s_save_pending;
static uint8 s_menu_built;

static beacon_position_recorder_flash_data_t s_map_data;
static uint8 s_map_data_valid;
static float s_edit_position[BEACON_POSITION_RECORDER_AXIS_NUM];
static float s_edit_points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
static menu_external_param_config_t s_edit_param_config[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
static menu_item_t s_beacon_menu[4U];
static menu_item_t s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS + 2U];
static menu_item_t s_point_menu[BEACON_POSITION_RECORDER_MAX_POINTS][4U];

static void beacon_position_recorder_menu_record(void);
static void beacon_position_recorder_menu_map(void);
static void beacon_position_recorder_menu_edit(void);
static void beacon_position_recorder_menu_save(void);
static void beacon_position_recorder_render_record(void);
static void beacon_position_recorder_render_map(void);

static uint8 beacon_position_recorder_point_valid(
    const float point[BEACON_POSITION_RECORDER_AXIS_NUM])
{
    if(point == NULL)
    {
        return 0U;
    }

    return ((point[x] != BEACON_POSITION_RECORDER_INVALID_COORD) &&
            (point[y] != BEACON_POSITION_RECORDER_INVALID_COORD)) ? 1U : 0U;
}

static void beacon_position_recorder_fill_invalid(
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        points[index][x] = BEACON_POSITION_RECORDER_INVALID_COORD;
        points[index][y] = BEACON_POSITION_RECORDER_INVALID_COORD;
    }
}

static uint16 beacon_position_recorder_count_valid(
    const float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;
    uint16 count = 0U;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        if(beacon_position_recorder_point_valid(points[index]) != 0U)
        {
            count++;
        }
    }
    return count;
}

static uint32 beacon_position_recorder_checksum_mix(uint32 checksum, uint32 value)
{
    checksum ^= value;
    checksum = (checksum << 7) | (checksum >> 25);
    checksum += 0x9E3779B9UL;
    return checksum;
}

static uint32 beacon_position_recorder_calc_checksum(
    const beacon_position_recorder_flash_data_t *data)
{
    uint32 checksum = 0x13579BDFUL;
    uint32 value_bits;
    uint16 point;
    uint8 axis;

    if(data == NULL)
    {
        return 0U;
    }

    checksum = beacon_position_recorder_checksum_mix(checksum, data->magic);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->version);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->point_count);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->full);

    for(axis = 0U; axis < BEACON_POSITION_RECORDER_AXIS_NUM; axis++)
    {
        memcpy(&value_bits, &data->position[axis], sizeof(value_bits));
        checksum = beacon_position_recorder_checksum_mix(checksum, value_bits);
    }

    for(point = 0U; point < BEACON_POSITION_RECORDER_MAX_POINTS; point++)
    {
        for(axis = 0U; axis < BEACON_POSITION_RECORDER_AXIS_NUM; axis++)
        {
            memcpy(&value_bits, &data->points[point][axis], sizeof(value_bits));
            checksum = beacon_position_recorder_checksum_mix(checksum, value_bits);
        }
    }

    return checksum;
}

static uint8 beacon_position_recorder_flash_data_valid(
    const beacon_position_recorder_flash_data_t *data)
{
    if((data == NULL) ||
       (data->magic != BEACON_POSITION_RECORDER_FLASH_MAGIC) ||
       (data->version != BEACON_POSITION_RECORDER_FLASH_VERSION) ||
       (data->point_count > BEACON_POSITION_RECORDER_MAX_POINTS) ||
       (data->full > 1U))
    {
        return 0U;
    }

    return (data->checksum == beacon_position_recorder_calc_checksum(data)) ? 1U : 0U;
}

static uint8 beacon_position_recorder_read_flash(
    beacon_position_recorder_flash_data_t *data)
{
    uint32 words;

    if((data == NULL) ||
       (flash_check(0U, BEACON_POSITION_RECORDER_FLASH_PAGE) == 0U))
    {
        return 0U;
    }

    words = (uint32)(sizeof(*data) / sizeof(uint32));
    memset(data, 0, sizeof(*data));
    flash_read_page(0U,
                    BEACON_POSITION_RECORDER_FLASH_PAGE,
                    (uint32 *)data,
                    words);
    return beacon_position_recorder_flash_data_valid(data);
}

static uint8 beacon_position_recorder_write_flash(
    beacon_position_recorder_flash_data_t *data)
{
    beacon_position_recorder_flash_data_t verify;
    uint32 words;

    if(data == NULL)
    {
        return 0U;
    }

    data->magic = BEACON_POSITION_RECORDER_FLASH_MAGIC;
    data->version = BEACON_POSITION_RECORDER_FLASH_VERSION;
    data->checksum = beacon_position_recorder_calc_checksum(data);
    words = (uint32)(sizeof(*data) / sizeof(uint32));

    flash_write_page(0U,
                     BEACON_POSITION_RECORDER_FLASH_PAGE,
                     (const uint32 *)data,
                     words);

    memset(&verify, 0, sizeof(verify));
    flash_read_page(0U,
                    BEACON_POSITION_RECORDER_FLASH_PAGE,
                    (uint32 *)&verify,
                    words);
    return beacon_position_recorder_flash_data_valid(&verify);
}

static void beacon_position_recorder_build_flash_data(
    beacon_position_recorder_flash_data_t *data,
    const float position[BEACON_POSITION_RECORDER_AXIS_NUM],
    const float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;

    memset(data, 0, sizeof(*data));
    memcpy(data->position, position, sizeof(data->position));
    beacon_position_recorder_fill_invalid(data->points);

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        if(beacon_position_recorder_point_valid(points[index]) != 0U)
        {
            data->points[index][x] = points[index][x];
            data->points[index][y] = points[index][y];
        }
    }

    data->point_count = beacon_position_recorder_count_valid(data->points);
    data->full = (data->point_count >= BEACON_POSITION_RECORDER_MAX_POINTS) ? 1U : 0U;
}

static void beacon_position_recorder_apply_flash_data(
    const beacon_position_recorder_flash_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    memcpy(g_beacon_position_recorder.position,
           data->position,
           sizeof(g_beacon_position_recorder.position));
    memcpy(g_beacon_position_recorder.points,
           data->points,
           sizeof(g_beacon_position_recorder.points));
    g_beacon_position_recorder.point_count = data->point_count;
    g_beacon_position_recorder.full = data->full;
    g_beacon_position_recorder.active = 0U;
}

static uint8 beacon_position_recorder_save_current(void)
{
    beacon_position_recorder_flash_data_t data;

    beacon_position_recorder_build_flash_data(&data,
                                               g_beacon_position_recorder.position,
                                               g_beacon_position_recorder.points);
    if(beacon_position_recorder_write_flash(&data) == 0U)
    {
        return 0U;
    }

    beacon_position_recorder_apply_flash_data(&data);
    return 1U;
}

static void beacon_position_recorder_try_save(void)
{
    if((s_save_pending == 0U) || (car_menu_is_runtime_locked() != 0U))
    {
        return;
    }

    (void)beacon_position_recorder_save_current();
    s_save_pending = 0U;
}

static uint8 beacon_position_recorder_ch8_high(void)
{
    return (g_air_crsf_std_ch8 > 0.5f) ? 1U : 0U;
}

static void beacon_position_recorder_record(void)
{
    uint16 index = g_beacon_position_recorder.point_count;

    if(index >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
        return;
    }

    g_beacon_position_recorder.points[index][x] = g_beacon_position_recorder.position[x];
    g_beacon_position_recorder.points[index][y] = g_beacon_position_recorder.position[y];
    g_beacon_position_recorder.point_count++;
    if(g_beacon_position_recorder.point_count >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
    }
}

static void beacon_position_recorder_invalidate_edit_point(uint8 index)
{
    if(index >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        return;
    }

    s_edit_points[index][x] = BEACON_POSITION_RECORDER_INVALID_COORD;
    s_edit_points[index][y] = BEACON_POSITION_RECORDER_INVALID_COORD;
}

#define BEACON_DEFINE_INVALID_FUNCTION(number, index) \
    static void beacon_position_recorder_invalid_##number(void) \
    { \
        beacon_position_recorder_invalidate_edit_point(index); \
    }

BEACON_DEFINE_INVALID_FUNCTION(1, 0U)
BEACON_DEFINE_INVALID_FUNCTION(2, 1U)
BEACON_DEFINE_INVALID_FUNCTION(3, 2U)
BEACON_DEFINE_INVALID_FUNCTION(4, 3U)
BEACON_DEFINE_INVALID_FUNCTION(5, 4U)
BEACON_DEFINE_INVALID_FUNCTION(6, 5U)
BEACON_DEFINE_INVALID_FUNCTION(7, 6U)
BEACON_DEFINE_INVALID_FUNCTION(8, 7U)
BEACON_DEFINE_INVALID_FUNCTION(9, 8U)
BEACON_DEFINE_INVALID_FUNCTION(10, 9U)

static void (*const s_invalid_functions[BEACON_POSITION_RECORDER_MAX_POINTS])(void) =
{
    beacon_position_recorder_invalid_1,
    beacon_position_recorder_invalid_2,
    beacon_position_recorder_invalid_3,
    beacon_position_recorder_invalid_4,
    beacon_position_recorder_invalid_5,
    beacon_position_recorder_invalid_6,
    beacon_position_recorder_invalid_7,
    beacon_position_recorder_invalid_8,
    beacon_position_recorder_invalid_9,
    beacon_position_recorder_invalid_10
};

static void beacon_position_recorder_build_menu(void)
{
    uint8 index;

    if(s_menu_built != 0U)
    {
        return;
    }

    memset(s_beacon_menu, 0, sizeof(s_beacon_menu));
    memset(s_edit_menu, 0, sizeof(s_edit_menu));
    memset(s_point_menu, 0, sizeof(s_point_menu));
    memset(s_edit_param_config, 0, sizeof(s_edit_param_config));

    strcpy(s_beacon_menu[0].name, "C_BeaconRec");
    s_beacon_menu[0].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[0].function = beacon_position_recorder_menu_record;
    strcpy(s_beacon_menu[1].name, "Map");
    s_beacon_menu[1].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[1].function = beacon_position_recorder_menu_map;
    strcpy(s_beacon_menu[2].name, "Edit Data");
    s_beacon_menu[2].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[2].function = beacon_position_recorder_menu_edit;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        snprintf(s_edit_menu[index].name,
                 sizeof(s_edit_menu[index].name),
                 "Beacon%u",
                 (unsigned int)(index + 1U));
        s_edit_menu[index].type = MENU_TYPE_SUBMENU;
        s_edit_menu[index].submenu = s_point_menu[index];

        s_edit_param_config[index][x].variable = &s_edit_points[index][x];
        s_edit_param_config[index][x].step = 0.1f;
        s_edit_param_config[index][x].min_val = -3.0f;
        s_edit_param_config[index][x].max_val = 4.0f;
        s_edit_param_config[index][y].variable = &s_edit_points[index][y];
        s_edit_param_config[index][y].step = 0.1f;
        s_edit_param_config[index][y].min_val = 0.0f;
        s_edit_param_config[index][y].max_val = 7.0f;

        strcpy(s_point_menu[index][0].name, "X");
        s_point_menu[index][0].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_point_menu[index][0].external_param = &s_edit_param_config[index][x];
        strcpy(s_point_menu[index][1].name, "Y");
        s_point_menu[index][1].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_point_menu[index][1].external_param = &s_edit_param_config[index][y];
        strcpy(s_point_menu[index][2].name, "Set Invalid");
        s_point_menu[index][2].type = MENU_TYPE_FUNCTION;
        s_point_menu[index][2].function = s_invalid_functions[index];
    }

    strcpy(s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].name, "Save");
    s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].type = MENU_TYPE_FUNCTION;
    s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].function = beacon_position_recorder_menu_save;
    s_menu_built = 1U;
}

static int32 beacon_position_recorder_round_to_pixel(float value)
{
    float scaled = value * (float)BEACON_MAP_CELL_PIXELS;

    return (scaled >= 0.0f) ? (int32)(scaled + 0.5f) : (int32)(scaled - 0.5f);
}

static void beacon_position_recorder_draw_marker(uint16 px, uint16 py, uint16 color)
{
    ips114_draw_line((uint16)(px - 2U), py, (uint16)(px + 2U), py, color);
    ips114_draw_line(px, (uint16)(py - 2U), px, (uint16)(py + 2U), color);
}

static void beacon_position_recorder_render_record(void)
{
    char text[32];
    uint16 last_index;

    /* 车辆运行时避免整屏 SPI 刷新占用控制周期，停车后自动恢复显示更新。 */
    if((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        return;
    }

    ips114_clear();
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);

    ips114_show_string(0, 0, "Beacon Recorder");
    sprintf(text, "Active:%u Full:%u",
            (unsigned int)g_beacon_position_recorder.active,
            (unsigned int)g_beacon_position_recorder.full);
    ips114_show_string(0, 16, text);
    sprintf(text, "Count:%u/%u",
            (unsigned int)g_beacon_position_recorder.point_count,
            (unsigned int)BEACON_POSITION_RECORDER_MAX_POINTS);
    ips114_show_string(0, 32, text);
    sprintf(text, "Pos X:%8.3f", (double)g_beacon_position_recorder.position[x]);
    ips114_show_string(0, 48, text);
    sprintf(text, "Pos Y:%8.3f", (double)g_beacon_position_recorder.position[y]);
    ips114_show_string(0, 64, text);

    if(g_beacon_position_recorder.point_count > 0U)
    {
        last_index = g_beacon_position_recorder.point_count - 1U;
        sprintf(text, "Last X:%7.3f",
                (double)g_beacon_position_recorder.points[last_index][x]);
        ips114_show_string(0, 80, text);
        sprintf(text, "Last Y:%7.3f",
                (double)g_beacon_position_recorder.points[last_index][y]);
        ips114_show_string(0, 96, text);
    }
    else
    {
        ips114_show_string(0, 80, "Last X: --");
        ips114_show_string(0, 96, "Last Y: --");
    }

    ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);
    ips114_show_string(0, 112, "Hold Back Exit");
}

static void beacon_position_recorder_render_map(void)
{
    uint8 line;
    uint16 index;
    int32 px;
    int32 py;
    char text[16];

    ips114_clear();
    ips114_set_font(IPS114_6X8_FONT);

    for(line = 0U; line <= (BEACON_MAP_LEFT_CELLS + BEACON_MAP_RIGHT_CELLS); line++)
    {
        uint16 grid_x = (uint16)(BEACON_MAP_START_X + (line * BEACON_MAP_CELL_PIXELS));
        ips114_draw_line(grid_x, BEACON_MAP_TOP_Y, grid_x, BEACON_MAP_BOTTOM_Y, RGB565_WHITE);
    }
    for(line = 0U; line <= BEACON_MAP_UP_CELLS; line++)
    {
        uint16 grid_y = (uint16)(BEACON_MAP_BOTTOM_Y - (line * BEACON_MAP_CELL_PIXELS));
        ips114_draw_line(BEACON_MAP_START_X, grid_y, BEACON_MAP_END_X, grid_y, RGB565_WHITE);
    }

    ips114_draw_line(BEACON_MAP_ORIGIN_X,
                     BEACON_MAP_TOP_Y,
                     BEACON_MAP_ORIGIN_X,
                     BEACON_MAP_BOTTOM_Y,
                     RGB565_GREEN);
    ips114_draw_line(BEACON_MAP_START_X,
                     BEACON_MAP_BOTTOM_Y,
                     BEACON_MAP_END_X,
                     BEACON_MAP_BOTTOM_Y,
                     RGB565_GREEN);

    if(s_map_data_valid != 0U)
    {
        for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
        {
            if(beacon_position_recorder_point_valid(s_map_data.points[index]) == 0U)
            {
                continue;
            }
            if((s_map_data.points[index][x] < -3.0f) ||
               (s_map_data.points[index][x] > 4.0f) ||
               (s_map_data.points[index][y] < 0.0f) ||
               (s_map_data.points[index][y] > 7.0f))
            {
                continue;
            }

            px = BEACON_MAP_ORIGIN_X +
                 beacon_position_recorder_round_to_pixel(s_map_data.points[index][x]);
            py = BEACON_MAP_BOTTOM_Y -
                 beacon_position_recorder_round_to_pixel(s_map_data.points[index][y]);
            beacon_position_recorder_draw_marker((uint16)px, (uint16)py, RGB565_RED);
            sprintf(text, "%u", (unsigned int)(index + 1U));
            ips114_set_color(RGB565_YELLOW, UI_COLOR_BG);
            ips114_show_string((uint16)(px + 3), (uint16)(py > 7 ? py - 7 : py + 3), text);
        }
    }

    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_show_string(132, 8, "Beacon Map");
    sprintf(text, "Count:%u", (unsigned int)((s_map_data_valid != 0U) ? s_map_data.point_count : 0U));
    ips114_show_string(132, 24, text);
    ips114_show_string(132, 40, "Grid 7x7");
    ips114_show_string(132, 56, "X -3..4");
    ips114_show_string(132, 72, "Y 0..7");
    if(s_map_data_valid == 0U)
    {
        ips114_set_color(UI_COLOR_ERROR, UI_COLOR_BG);
        ips114_show_string(132, 88, "No Data");
    }
    ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);
    ips114_show_string(132, 112, "Back Exit");
}

static const menu_external_view_config_t s_record_view_config =
{
    beacon_position_recorder_render_record,
    beacon_position_recorder_exit,
    1U,
    1U,
    1U
};

static const menu_external_view_config_t s_map_view_config =
{
    beacon_position_recorder_render_map,
    NULL,
    0U,
    0U,
    0U
};

static void beacon_position_recorder_menu_record(void)
{
    beacon_position_recorder_enter();
    if(menu_enter_external_view(&s_record_view_config) != 0U)
    {
        g_beacon_position_recorder.active = 0U;
    }
}

static void beacon_position_recorder_menu_map(void)
{
    memset(&s_map_data, 0, sizeof(s_map_data));
    s_map_data_valid = beacon_position_recorder_read_flash(&s_map_data);
    (void)menu_enter_external_view(&s_map_view_config);
}

static void beacon_position_recorder_menu_edit(void)
{
    beacon_position_recorder_flash_data_t data;

    memset(s_edit_position, 0, sizeof(s_edit_position));
    beacon_position_recorder_fill_invalid(s_edit_points);
    if(beacon_position_recorder_read_flash(&data) != 0U)
    {
        memcpy(s_edit_position, data.position, sizeof(s_edit_position));
        memcpy(s_edit_points, data.points, sizeof(s_edit_points));
    }
    menu_enter_submenu(s_edit_menu);
}

static void beacon_position_recorder_menu_save(void)
{
    beacon_position_recorder_flash_data_t data;

    beacon_position_recorder_build_flash_data(&data, s_edit_position, s_edit_points);
    if(beacon_position_recorder_write_flash(&data) == 0U)
    {
        menu_show_error("Beacon Save Fail");
        return;
    }

    memcpy(s_edit_points, data.points, sizeof(s_edit_points));
    beacon_position_recorder_apply_flash_data(&data);
    menu_show_success("Beacon Save OK");
}

void beacon_position_recorder_init(void)
{
    beacon_position_recorder_flash_data_t data;

    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    beacon_position_recorder_fill_invalid(g_beacon_position_recorder.points);
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;
    s_save_pending = 0U;
    beacon_position_recorder_build_menu();

    if(beacon_position_recorder_read_flash(&data) != 0U)
    {
        beacon_position_recorder_apply_flash_data(&data);
    }
}

void beacon_position_recorder_enter(void)
{
    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    beacon_position_recorder_fill_invalid(g_beacon_position_recorder.points);
    g_beacon_position_recorder.active = 1U;

    if(air_comm_car_is_run_data_fresh() != 0U)
    {
        s_last_ch8_high = beacon_position_recorder_ch8_high();
        s_ch8_synced = 1U;
    }
    else
    {
        s_last_ch8_high = 0U;
        s_ch8_synced = 0U;
    }
}

void beacon_position_recorder_exit(void)
{
    uint8 was_active = g_beacon_position_recorder.active;

    g_beacon_position_recorder.active = 0U;
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;

    if(was_active != 0U)
    {
        s_save_pending = 1U;
        beacon_position_recorder_try_save();
    }
}

void beacon_position_recorder_update_100HZ(void)
{
    uint8 ch8_high;

    if(g_beacon_position_recorder.active == 0U)
    {
        beacon_position_recorder_try_save();
        return;
    }

    g_beacon_position_recorder.position[x] +=
        g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_beacon_position_recorder.position[y] +=
        g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;

    if(air_comm_car_is_run_data_fresh() == 0U)
    {
        s_ch8_synced = 0U;
        return;
    }

    ch8_high = beacon_position_recorder_ch8_high();
    if(s_ch8_synced == 0U)
    {
        s_last_ch8_high = ch8_high;
        s_ch8_synced = 1U;
        return;
    }

    if((ch8_high != 0U) && (s_last_ch8_high == 0U))
    {
        beacon_position_recorder_record();
    }

    s_last_ch8_high = ch8_high;
}

uint8 beacon_position_recorder_is_active(void)
{
    return g_beacon_position_recorder.active;
}

struct menu_item *beacon_position_recorder_get_menu(void)
{
    beacon_position_recorder_build_menu();
    return s_beacon_menu;
}
