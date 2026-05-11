#include "menu_air_support.h"

#define MENU_AIR_SLOT_BASE_PAGE             (80U)
#define MENU_AIR_SLOT_COUNT                 (4U)
#define MENU_AIR_SLOT_SIZE                  (2U)
#define MENU_AIR_MAGIC_NUMBER               (0x41495250UL)
#define MENU_AIR_VERSION                    (1U)
#define MENU_AIR_MAX_PARAMS                 (16U)

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

static float menu_air_clamp(float value, float min_val, float max_val)
{
    if(value < min_val)
    {
        return min_val;
    }
    if(value > max_val)
    {
        return max_val;
    }
    return value;
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
        menu_air_clamp(*(s_air_params[s_air_param_count].variable), min, max);
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
    uint8 result;

    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        return 1U;
    }

    value = menu_air_clamp(value, s_air_params[index].min_val, s_air_params[index].max_val);
    result = air_comm_car_set_param(s_air_params[index].name, value);
    if(result == 0U)
    {
        *(s_air_params[index].variable) = value;
    }

    return result;
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
    uint8 failed_count = 0U;

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ?
                        "Air Offline" : "Car Active");
        return 1U;
    }

    for(index = 0U; index < s_air_param_count; index++)
    {
        if(air_comm_car_set_param(s_air_params[index].name,
                                  *(s_air_params[index].variable)) != 0U)
        {
            failed_count++;
        }
    }

    if(failed_count > 0U)
    {
        menu_show_error("Sync Failed");
        return 1U;
    }

    return 0U;
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
        value = menu_air_clamp(value, s_air_params[index].min_val, s_air_params[index].max_val);
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
