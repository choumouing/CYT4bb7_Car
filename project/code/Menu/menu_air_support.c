/* Air参数远程同步模块 - 实现
 *
 * Flash存档布局：页80-87（4个slot，每个2页/4KB）
 * 同步协议：车端发送set_param(name, value) → Air回复ACK(ok/fail)
 * 每次只同步一个参数，100HZ轮询逐个清除dirty
 */
#include "menu_air_support.h"

#define MENU_AIR_SLOT_BASE_PAGE             (80U)       // Air存档起始页（避开车端72-79）
#define MENU_AIR_SLOT_COUNT                 (4U)        // 存档槽数量
#define MENU_AIR_SLOT_SIZE                  (2U)        // 每个槽占用页数
#define MENU_AIR_MAGIC_NUMBER               (0x41495250UL)  // "AIRP"魔数
#define MENU_AIR_VERSION                    (1U)        // 存档版本
#define MENU_AIR_MAX_PARAMS                 (16U)       // 最大参数数量
#define MENU_AIR_SYNC_INVALID_INDEX         (0xFFU)     // 无效索引标记
#define MENU_AIR_ACK_TYPE_SET_PARAM         (0x01U)     // set_param命令的ACK类型

typedef struct
{
    uint32 magic;
    uint8 version;
    uint8 slot_id;
    uint16 param_count;
    uint32 checksum;
} menu_air_slot_header_t;

float air_min_area = 5.0f;
float air_hold_ms = 30.0f;
float air_x_bias = 0.0f;
float air_y_bias = 0.0f;

static menu_air_param_config_t s_air_params[MENU_AIR_MAX_PARAMS];
static uint8 s_air_param_count;
static uint8 s_air_param_dirty[MENU_AIR_MAX_PARAMS];
static menu_air_sync_status_t s_air_sync_status;

static uint8 menu_air_dirty_count(void)
{
    uint8 index;
    uint8 count = 0U;

    for(index = 0U; index < s_air_param_count; index++)
    {
        if(s_air_param_dirty[index] != 0U)
        {
            count++;
        }
    }

    return count;
}

static uint8 menu_air_find_dirty_index(void)
{
    uint8 index;

    for(index = 0U; index < s_air_param_count; index++)
    {
        if(s_air_param_dirty[index] != 0U)
        {
            return index;
        }
    }

    return MENU_AIR_SYNC_INVALID_INDEX;
}

/* 标记指定参数为dirty（需要同步到Air） */
static void menu_air_mark_dirty(uint8 index)
{
    if(index < s_air_param_count)
    {
        s_air_param_dirty[index] = 1U;
        s_air_sync_status.dirty_count = menu_air_dirty_count();
    }
}

static uint32 menu_air_calc_checksum(uint8 count)
{
    uint8 index;
    uint32 checksum = 0x13572468UL;
    float value;
    uint32 value_bits;

    for(index = 0U; index < count; index++)
    {
        value = *(s_air_params[index].variable);
        memcpy(&value_bits, &value, sizeof(value_bits));
        checksum ^= value_bits;
        checksum = (checksum << 5) | (checksum >> 27);
        checksum += (uint32)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint32 menu_air_calc_buffer_checksum(uint8 count, uint32 offset)
{
    uint8 index;
    uint32 checksum = 0x13572468UL;
    uint32 value_bits;

    for(index = 0U; index < count; index++)
    {
        value_bits = flash_union_buffer[offset + index].uint32_type;
        checksum ^= value_bits;
        checksum = (checksum << 5) | (checksum >> 27);
        checksum += (uint32)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint8 menu_air_slot_valid(uint8 slot, menu_air_slot_header_t *out_header)
{
    uint32 page;
    menu_air_slot_header_t *header;

    if(slot >= MENU_AIR_SLOT_COUNT)
    {
        return 0U;
    }

    page = MENU_AIR_SLOT_BASE_PAGE + ((uint32)slot * MENU_AIR_SLOT_SIZE);
    if((page + MENU_AIR_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        return 0U;
    }

    if(flash_check(0U, page) == 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(0U, page, FLASH_PAGE_LENGTH);
    header = (menu_air_slot_header_t *)flash_union_buffer;

    if((header->magic != MENU_AIR_MAGIC_NUMBER) ||
       (header->version != MENU_AIR_VERSION) ||
       (header->slot_id != slot) ||
       (header->param_count > MENU_AIR_MAX_PARAMS))
    {
        return 0U;
    }

    if(header->checksum != menu_air_calc_buffer_checksum(header->param_count,
        (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U)))
    {
        return 0U;
    }

    if(out_header != NULL)
    {
        *out_header = *header;
    }

    return 1U;
}

void menu_air_support_init(void)
{
    s_air_param_count = 0U;
    memset(s_air_param_dirty, 0, sizeof(s_air_param_dirty));
    memset(&s_air_sync_status, 0, sizeof(s_air_sync_status));
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.last_failed_index = MENU_AIR_SYNC_INVALID_INDEX;

    menu_register_param_air("air_min_area", &air_min_area, 1.0f, 0.0f, 500.0f);
    menu_register_param_air("air_hold_ms", &air_hold_ms, 5.0f, 0.0f, 200.0f);
    menu_register_param_air("air_x_bias", &air_x_bias, 1.0f, -40.0f, 40.0f);
    menu_register_param_air("air_y_bias", &air_y_bias, 1.0f, -40.0f, 40.0f);

    if(menu_air_slot_valid(0U, NULL) != 0U)
    {
        (void)menu_load_air_slot(0U);
    }
}

void menu_register_param_air(const char *name, float *var, float step, float min, float max)
{
    uint8 name_len;

    if((name == NULL) || (var == NULL) || (s_air_param_count >= MENU_AIR_MAX_PARAMS))
    {
        return;
    }

    name_len = (uint8)strlen(name);
    if((name_len == 0U) || (name_len >= sizeof(s_air_params[0].name)))
    {
        return;
    }

    memset(s_air_params[s_air_param_count].name, 0, sizeof(s_air_params[s_air_param_count].name));
    memcpy(s_air_params[s_air_param_count].name, name, name_len);
    s_air_params[s_air_param_count].variable = var;
    s_air_params[s_air_param_count].step = step;
    s_air_params[s_air_param_count].min_val = min;
    s_air_params[s_air_param_count].max_val = max;
    *(s_air_params[s_air_param_count].variable) =
        car_math_clampf(*(s_air_params[s_air_param_count].variable), min, max);
    s_air_param_count++;
}

uint8 menu_get_air_param_count(void)
{
    return s_air_param_count;
}

float menu_get_air_param_by_index(uint8 index)
{
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 0.0f;
    }

    return *(s_air_params[index].variable);
}

uint8 menu_set_air_param_by_index(uint8 index, float value)
{
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        return 1U;
    }

    value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);
    *(s_air_params[index].variable) = value;
    menu_air_mark_dirty(index);

    return 0U;
}

const menu_air_param_config_t *menu_get_air_param_config(uint8 index)
{
    if(index >= s_air_param_count)
    {
        return NULL;
    }

    return &s_air_params[index];
}

uint8 menu_is_air_connected(void)
{
    return air_comm_car_is_online();
}

uint8 menu_can_edit_air_params(void)
{
    if(menu_is_air_connected() == 0U)
    {
        return 0U;
    }

    return ((car_control_enabled == 0U) ||
            (car_emergency_stop_active != 0U)) ? 1U : 0U;
}

uint8 menu_sync_all_air_params(void)
{
    uint8 index;

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ?
                        "Air Offline" : "Car Active");
        return 1U;
    }

    for(index = 0U; index < s_air_param_count; index++)
    {
        s_air_param_dirty[index] = 1U;
    }

    s_air_sync_status.dirty_count = menu_air_dirty_count();

    return 0U;
}

/* 100HZ同步轮询
 * 逻辑：
 *   1. 正在发送 → 等ACK回来，处理结果（成功清dirty，失败也清dirty避免死循环）
 *   2. 空闲 → 找第一个dirty参数 → 发送set_param命令
 *   3. 发送失败 → 立即清dirty + 记录失败
 * 注意：每次只处理一个参数，100HZ逐个同步
 */
void menu_air_update_100HZ(void)
{
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;
    uint8 dirty_index;

    if(s_air_sync_status.sending != 0U)
    {
        (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
        if((air_comm_car_has_pending_ack() == 0U) &&
           (ack_type == MENU_AIR_ACK_TYPE_SET_PARAM) &&
           (ack_result != AIR_COMM_ACK_RESULT_NONE))
        {
            s_air_sync_status.last_result = ack_result;
            s_air_sync_status.last_status = ack_status;
            if((ack_result == AIR_COMM_ACK_RESULT_OK) &&
               (ack_status == AIR_COMM_STATUS_OK) &&
               (s_air_sync_status.active_index < s_air_param_count))
            {
                s_air_param_dirty[s_air_sync_status.active_index] = 0U;
                s_air_sync_status.ok_count++;
            }
            else
            {
                if(s_air_sync_status.active_index < s_air_param_count)
                {
                    s_air_param_dirty[s_air_sync_status.active_index] = 0U;
                }
                s_air_sync_status.last_failed_index = s_air_sync_status.active_index;
                s_air_sync_status.fail_count++;
            }

            s_air_sync_status.sending = 0U;
            s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
            s_air_sync_status.dirty_count = menu_air_dirty_count();
        }

        return;
    }

    if((menu_can_edit_air_params() == 0U) || (air_comm_car_has_pending_ack() != 0U))
    {
        s_air_sync_status.dirty_count = menu_air_dirty_count();
        return;
    }

    dirty_index = menu_air_find_dirty_index();
    if(dirty_index == MENU_AIR_SYNC_INVALID_INDEX)
    {
        s_air_sync_status.dirty_count = 0U;
        return;
    }

    if(air_comm_car_set_param(s_air_params[dirty_index].name,
                              *(s_air_params[dirty_index].variable)) == 0U)
    {
        s_air_sync_status.sending = 1U;
        s_air_sync_status.active_index = dirty_index;
        s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
        s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_sync_status.send_count++;
    }
    else
    {
        s_air_param_dirty[dirty_index] = 0U;
        s_air_sync_status.last_failed_index = dirty_index;
        s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_ERROR;
        s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_sync_status.fail_count++;
    }

    s_air_sync_status.dirty_count = menu_air_dirty_count();
}

void menu_get_air_sync_status(menu_air_sync_status_t *status)
{
    if(status == NULL)
    {
        return;
    }

    s_air_sync_status.dirty_count = menu_air_dirty_count();
    *status = s_air_sync_status;
}

uint8 menu_load_air_slot(uint8 slot)
{
    menu_air_slot_header_t header;
    uint32 offset;
    uint8 index;
    uint8 count;
    float value;

    if(menu_air_slot_valid(slot, &header) == 0U)
    {
        menu_show_error("No Air Data");
        return 1U;
    }

    count = (header.param_count < s_air_param_count) ? header.param_count : s_air_param_count;
    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);

    for(index = 0U; index < count; index++)
    {
        value = flash_union_buffer[offset + index].float_type;
        value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);
        *(s_air_params[index].variable) = value;
    }

    return 0U;
}

uint8 menu_save_air_slot(uint8 slot)
{
    uint32 page;
    uint32 offset;
    uint8 index;
    menu_air_slot_header_t *header;

    if(slot >= MENU_AIR_SLOT_COUNT)
    {
        menu_show_error("Air Slot Err");
        return 1U;
    }

    page = MENU_AIR_SLOT_BASE_PAGE + ((uint32)slot * MENU_AIR_SLOT_SIZE);
    if((page + MENU_AIR_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_error("Air Page Err");
        return 1U;
    }

    flash_buffer_clear();
    header = (menu_air_slot_header_t *)flash_union_buffer;
    header->magic = MENU_AIR_MAGIC_NUMBER;
    header->version = MENU_AIR_VERSION;
    header->slot_id = slot;
    header->param_count = s_air_param_count;
    header->checksum = menu_air_calc_checksum(s_air_param_count);

    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);
    for(index = 0U; index < s_air_param_count; index++)
    {
        flash_union_buffer[offset + index].float_type = *(s_air_params[index].variable);
    }

    flash_erase_page(0U, page);
    (void)flash_write_page_from_buffer(0U, page, offset + s_air_param_count);

    return 0U;
}
