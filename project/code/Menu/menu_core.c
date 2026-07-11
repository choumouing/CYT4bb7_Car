/*********************************************************************************************************************
* 菜单核心框架实现文件
* 功能：菜单引擎核心实现，包括菜单导航、按键处理、屏幕渲染、Flash存档管理、参数编辑系统
* 特点：框架稳定，通常不需要修改
*
* 适配：逐飞科技 IPS114 + CYT4BB7 Flash + 统一参数集合存档模式
* 版本：v2.1 (CYT4BB7移植版本)
* 日期：2025年
********************************************************************************************************************/
#include "menu_core.h"

//====================================================内部变量====================================================
// 菜单状态变量
static menu_item_t* current_menu = NULL;               // 当前菜单
static menu_item_t* menu_stack[MENU_MAX_DEPTH];        // 菜单堆栈
static uint8_t menu_index_stack[MENU_MAX_DEPTH];       // 返回时恢复的索引
static uint8_t menu_offset_stack[MENU_MAX_DEPTH];      // 返回时恢复的偏移
static uint8_t menu_depth = 0;                         // 当前菜单深度
static uint8_t current_index = 0;                      // 当前选中项索引
static uint8_t current_item_count = 0;                 // 当前菜单项数量
static menu_state_t menu_state = MENU_STATE_NORMAL;    // 菜单状态

// 参数管理变量
static param_config_t param_configs[MENU_MAX_PARAMS];  // 参数配置表
static uint8_t param_count = 0;                        // 已注册参数数量（会在注册时递增）
static uint8_t current_slot = 0;                       // 当前存档号
static uint8_t air_edit_active = 0U;
static uint8_t air_edit_index = 0U;
static float air_edit_value = 0.0f;

// 显示控制变量
static uint8_t need_refresh = 1;                       // 需要刷新标志
static uint8_t display_offset = 0;                     // 显示偏移量（滚动支持）
static uint8_t diag_refresh_divider = 0;               // 诊断页10Hz刷新分频

// 局部刷新优化变量
static refresh_type_t refresh_type = REFRESH_FULL;     // 刷新类型
static uint8_t last_selected_index = 0;                // 上次选中的项索引
static uint8_t last_menu_state = MENU_STATE_NORMAL;    // 上次菜单状态

// 长按加速控制
#define KEY_REPEAT_FIRST_DELAY_TICKS   15   // 第一次重复延迟（10ms单位）
#define KEY_REPEAT_CONT_DELAY_TICKS    8   // 连续重复间隔（10ms单位）
static uint8_t key_hold_ticks[KEY_NUMBER] = {0};        // 按键按下计时
static uint8_t key_repeat_counters[KEY_NUMBER] = {0};   // 长按重复计数器
static uint8_t key_press_consumed[KEY_NUMBER] = {0};    // 是否已经在按下过程中处理
static const gpio_pin_enum menu_key_pins[KEY_NUMBER] = KEY_LIST; // 按键引脚映射
static uint8_t menu_wait_key_release = 0U;
static menu_external_view_config_t external_view_config;

// Flash操作延迟执行
static uint8_t pending_flash_operation = 0;            // 待执行的Flash操作
static uint8_t pending_slot_number = 0;                // 待操作的存档号
#define FLASH_OP_NONE    0
#define FLASH_OP_LOAD    1
#define FLASH_OP_SAVE    2

/* 判断菜单项是否为参数类型（本地参数或Air参数） */
static uint8_t menu_is_param_item(const menu_item_t *item)
{
    if(item == NULL)
    {
        return 0U;
    }

    return ((item->type == MENU_TYPE_PARAMETER) ||
            (item->type == MENU_TYPE_AIR_PARAMETER) ||
            (item->type == MENU_TYPE_EXTERNAL_PARAMETER)) ? 1U : 0U;
}

static void menu_clear_key_tracking(void)
{
    key_clear_all_state();
    memset(key_hold_ticks, 0, sizeof(key_hold_ticks));
    memset(key_repeat_counters, 0, sizeof(key_repeat_counters));
    memset(key_press_consumed, 0, sizeof(key_press_consumed));
}

static void menu_air_edit_reset(void)
{
    air_edit_active = 0U;
    air_edit_index = 0U;
    air_edit_value = 0.0f;
}

static uint8_t menu_air_edit_begin(uint8_t index)
{
    if(index >= menu_get_air_param_count())
    {
        menu_air_edit_reset();
        return 1U;
    }

    air_edit_index = index;
    air_edit_value = menu_get_air_param_by_index(index);
    air_edit_active = 1U;
    return 0U;
}

static uint8_t menu_air_get_display_value(uint8_t index, float *value)
{
    if((value == NULL) || (index >= menu_get_air_param_count()))
    {
        return 0U;
    }

    if((air_edit_active != 0U) &&
       (air_edit_index == index) &&
       (menu_state == MENU_STATE_EDIT))
    {
        *value = air_edit_value;
        return 1U;
    }

    *value = menu_get_air_param_by_index(index);
    return 1U;
}

static uint8_t menu_air_edit_adjust(const menu_item_t *item, float delta)
{
    const menu_air_param_config_t *config;

    if((item == NULL) || (item->type != MENU_TYPE_AIR_PARAMETER))
    {
        return 1U;
    }

    config = menu_get_air_param_config(item->param_index);
    if(config == NULL)
    {
        return 1U;
    }

    if((air_edit_active == 0U) || (air_edit_index != item->param_index))
    {
        if(menu_air_edit_begin(item->param_index) != 0U)
        {
            return 1U;
        }
    }

    air_edit_value += delta;
    if(air_edit_value < config->min_val)
    {
        air_edit_value = config->min_val;
    }
    if(air_edit_value > config->max_val)
    {
        air_edit_value = config->max_val;
    }
    return 0U;
}

static uint8_t menu_air_edit_commit(const menu_item_t *item)
{
    uint8_t result;

    if((item == NULL) || (item->type != MENU_TYPE_AIR_PARAMETER))
    {
        menu_air_edit_reset();
        return 1U;
    }

    if((air_edit_active == 0U) || (air_edit_index != item->param_index))
    {
        if(menu_air_edit_begin(item->param_index) != 0U)
        {
            return 1U;
        }
    }

    result = menu_air_commit_param_value(item->param_index, air_edit_value);
    menu_air_edit_reset();
    return result;
}

/* 获取菜单项对应的参数值（统一处理本地/Air参数） */
static uint8_t menu_get_item_param_value(const menu_item_t *item, float *value)
{
    if((item == NULL) || (value == NULL))
    {
        return 0U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 0U;
        }
        *value = menu_get_param_by_index(item->param_index);
        return 1U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        return menu_air_get_display_value(item->param_index, value);
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        *value = *(item->external_param->variable);
        return 1U;
    }

    return 0U;
}

/* 获取菜单项对应的编辑步进值 */
static uint8_t menu_get_item_param_step(const menu_item_t *item, float *step)
{
    const menu_air_param_config_t *air_config;

    if((item == NULL) || (step == NULL))
    {
        return 0U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 0U;
        }
        *step = param_configs[item->param_index].step;
        return 1U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        air_config = menu_get_air_param_config(item->param_index);
        if(air_config == NULL)
        {
            return 0U;
        }
        *step = air_config->step;
        return 1U;
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        *step = item->external_param->step;
        return 1U;
    }

    return 0U;
}

/* 设置菜单项对应的参数值（自动处理Air参数的dirty标记） */
static uint8_t menu_set_item_param_value(const menu_item_t *item, float value)
{
    if(item == NULL)
    {
        return 1U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 1U;
        }
        menu_set_param_by_index(item->param_index, value);
        return 0U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        return menu_set_air_param_by_index(item->param_index, value);
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        if(value < item->external_param->min_val)
        {
            value = item->external_param->min_val;
        }
        if(value > item->external_param->max_val)
        {
            value = item->external_param->max_val;
        }
        *(item->external_param->variable) = value;
        return 0U;
    }

    return 1U;
}

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t slot_id;
    uint16_t param_count;
    uint32_t data_checksum;
    uint32_t header_checksum;
    char slot_name[16];
} menu_flash_slot_header_t;

/* 获取当前格式存档对应的 Flash 页号。 */
static uint32_t menu_get_slot_page(uint8_t slot);

static uint32_t menu_flash_calc_data_checksum(uint16_t count, uint32_t offset)
{
    uint16_t index;
    uint32_t checksum = 0x2468ACE0UL;

    for(index = 0U; index < count; index++)
    {
        checksum ^= flash_union_buffer[offset + index].uint32_type;
        checksum = (checksum << 7) | (checksum >> 25);
        checksum += (uint32_t)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint32_t menu_flash_calc_header_checksum(const menu_flash_slot_header_t *header)
{
    uint32_t checksum = 0x13579BDFUL;

    if(header == NULL)
    {
        return 0U;
    }

    checksum ^= header->magic;
    checksum = (checksum << 5) | (checksum >> 27);
    checksum ^= ((uint32_t)header->version << 24);
    checksum ^= ((uint32_t)header->slot_id << 16);
    checksum ^= (uint32_t)header->param_count;
    checksum = (checksum << 5) | (checksum >> 27);
    checksum ^= header->data_checksum;

    return checksum;
}

static uint8_t menu_flash_slot_valid(uint8_t slot, menu_flash_slot_header_t *out_header)
{
    uint32_t page_num;
    uint32_t offset;
    menu_flash_slot_header_t *header;

    if((slot >= MENU_SLOT_COUNT) || (param_count == 0U))
    {
        return 0U;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        return 0U;
    }

    if(flash_check(0U, page_num) == 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(0U, page_num, FLASH_PAGE_LENGTH);
    header = (menu_flash_slot_header_t *)flash_union_buffer;
    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);

    if((header->magic != MENU_MAGIC_NUMBER) ||
       (header->version != MENU_VERSION) ||
       (header->slot_id != slot) ||
       (header->param_count != param_count) ||
       ((offset + header->param_count) > FLASH_PAGE_LENGTH))
    {
        return 0U;
    }

    if(header->data_checksum != menu_flash_calc_data_checksum(header->param_count, offset))
    {
        return 0U;
    }

    if(header->header_checksum != menu_flash_calc_header_checksum(header))
    {
        return 0U;
    }

    if(out_header != NULL)
    {
        *out_header = *header;
    }

    return 1U;
}

static uint32_t menu_get_slot_page(uint8_t slot)
{
    return MENU_SLOT_BASE_PAGE + ((uint32_t)slot * MENU_SLOT_SIZE);
}

//====================================================菜单核心功能====================================================

/**
 * @brief 10ms定时器中断处理函数
 * 注意：此函数需要在用户的中断服务函数中调用
 * 例如：在 pit_ch0_handler() 中调用 menu_timer_handler()
 */
void menu_timer_handler(void)
{
    key_scanner();  // 按键扫描放在定时器中断中
}

void menu_discard_key_events(void)
{
    menu_wait_key_release = 1U;
    menu_clear_key_tracking();
}

void menu_runtime_suspend(void)
{
    menu_air_edit_reset();
    if((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
       (external_view_config.allow_runtime_locked == 0U))
    {
        if(external_view_config.on_exit != NULL)
        {
            external_view_config.on_exit();
        }
        memset(&external_view_config, 0, sizeof(external_view_config));
        menu_state = MENU_STATE_NORMAL;
    }
    else if(menu_state != MENU_STATE_EXTERNAL_VIEW)
    {
        menu_state = MENU_STATE_NORMAL;
    }
    diag_refresh_divider = 0U;
    pending_flash_operation = FLASH_OP_NONE;
    pending_slot_number = 0U;
    menu_discard_key_events();
}

void menu_runtime_resume(void)
{
    menu_request_refresh(REFRESH_FULL);
}

/**
 * @brief 菜单系统初始化
 */
void menu_init(void)
{
    // 初始化IPS114
    ips114_init();

    // 初始化显示配置 (纯黑背景，白色文字)
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);

    // 初始化Flash存储
    flash_init();

    // 初始化按键
    key_init(10);  // 10ms扫描周期

    // 初始化变量
    menu_depth = 0;
    current_index = 0;
    current_item_count = 0;
    menu_state = MENU_STATE_NORMAL;
    diag_refresh_divider = 0U;
    param_count = 0;
    current_slot = 0;
    display_offset = 0;
    menu_air_edit_reset();
    menu_wait_key_release = 0U;
    memset(&external_view_config, 0, sizeof(external_view_config));

    // 初始化局部刷新变量
    refresh_type = REFRESH_FULL;
    last_selected_index = 0;
    last_menu_state = MENU_STATE_NORMAL;
    need_refresh = 1;

    // 初始化Flash操作标志
    pending_flash_operation = FLASH_OP_NONE;
    pending_slot_number = 0;

    // 延时确保系统稳定
    for(volatile uint32_t i = 0; i < 100000; i++);

    // 清除屏幕
    menu_clear_screen();

    // 首次显示菜单
    menu_request_refresh(REFRESH_FULL);
}

/**
 * @brief 菜单任务处理（主循环调用）
 */
void menu_update_100HZ(void)
{
    if((car_menu_is_runtime_locked() != 0U) &&
       (menu_external_view_runtime_active() == 0U))
    {
        menu_discard_key_events();
        return;
    }

    menu_process_keys();

    if(menu_state == MENU_STATE_EXTERNAL_VIEW)
    {
        if(external_view_config.refresh_10hz != 0U)
        {
            diag_refresh_divider++;
            if(diag_refresh_divider >= 10U)
            {
                diag_refresh_divider = 0U;
                if(external_view_config.render != NULL)
                {
                    external_view_config.render();
                }
            }
        }
        return;
    }

    if(menu_state == MENU_STATE_DIAG_VIEW)
    {
        diag_refresh_divider++;
        if(diag_refresh_divider >= 10U)
        {
            diag_refresh_divider = 0U;
            if((current_index < current_item_count) &&
               (current_menu[current_index].type == MENU_TYPE_DIAG_VIEW) &&
               (current_menu[current_index].function != NULL))
            {
                current_menu[current_index].function();
            }
        }
        return;
    }

    // 处理待执行的Flash操作（在主循环中执行，避免在中断中操作Flash）
    if(pending_flash_operation != FLASH_OP_NONE)
    {
        switch(pending_flash_operation)
        {
        case FLASH_OP_LOAD:
            if(menu_flash_check_slot(pending_slot_number))
            {
                // 存档有效，加载数据
                menu_flash_load_params(pending_slot_number);
                menu_show_success("Load OK");
            }
            else
            {
                // 存档无效，显示错误
                menu_show_error("No Data");
            }
            break;

        case FLASH_OP_SAVE:
            menu_flash_save_params(pending_slot_number);
            if(menu_flash_check_slot(pending_slot_number) != 0U)
            {
                menu_show_success("Save OK");
            }
            else
            {
                menu_show_error("Save Fail");
            }
            break;
        }
        pending_flash_operation = FLASH_OP_NONE;
        need_refresh = 1;
    }

    // 仅在需要时刷新显示（使用局部刷新优化）
    if(need_refresh)
    {
        menu_render_current_optimized();
        need_refresh = 0;
    }
}

/**
 * @brief 手动刷新菜单显示（外部调用）
 */
void menu_show(void)
{
    menu_request_refresh(REFRESH_FULL);  // 手动刷新使用全屏刷新
}

//====================================================参数管理====================================================

/**
 * @brief 注册参数
 * @param var 参数变量指针
 * @param step 编辑步进值
 * @param min 最小值
 * @param max 最大值
 */
void menu_register_param(float* var, float step, float min, float max)
{
    if(param_count >= MENU_MAX_PARAMS) return;

    param_configs[param_count].variable = var;
    param_configs[param_count].step = step;
    param_configs[param_count].min_val = min;
    param_configs[param_count].max_val = max;

    param_count++;
}

/**
 * @brief 获取参数数量
 */
uint8_t menu_get_param_count(void)
{
    return param_count;
}

/**
 * @brief 显示系统调试信息
 */
void menu_show_debug_info(void)
{
    char debug_text[32];
    ips114_clear();

    sprintf(debug_text, "Params: %d", param_count);
    ips114_show_string(0, 0, debug_text);

    sprintf(debug_text, "Slot: %d", current_slot);
    ips114_show_string(0, 16, debug_text);

    sprintf(debug_text, "Flash: Ready");
    ips114_show_string(0, 32, debug_text);

    // 显示前3个参数的实际值
    if(param_count >= 3)
    {
        sprintf(debug_text, "P0: %.2f", *(param_configs[0].variable));
        ips114_show_string(0, 48, debug_text);

        sprintf(debug_text, "P1: %.2f", *(param_configs[1].variable));
        ips114_show_string(0, 64, debug_text);

        sprintf(debug_text, "P2: %.2f", *(param_configs[2].variable));
        ips114_show_string(0, 80, debug_text);
    }

    ips114_show_string(0, 96, "Press key to exit");
}

/**
 * @brief 按索引获取参数值
 */
float menu_get_param_by_index(uint8_t index)
{
    if(index >= param_count || param_configs[index].variable == NULL)
        return 0.0f;
    return *(param_configs[index].variable);
}

/**
 * @brief 按索引设置参数值
 */
void menu_set_param_by_index(uint8_t index, float value)
{
    if(index >= param_count || param_configs[index].variable == NULL)
        return;

    // 限制在范围内
    if(value < param_configs[index].min_val)
        value = param_configs[index].min_val;
    if(value > param_configs[index].max_val)
        value = param_configs[index].max_val;

    *(param_configs[index].variable) = value;
}

//====================================================存档管理====================================================

/**
 * @brief 加载指定存档
 */
void menu_load_slot(uint8_t slot)
{
    if(slot >= MENU_SLOT_COUNT) return;

    // 如果有参数注册，则执行Flash操作
    if(param_count > 0)
    {
        // 设置延迟执行标志
        pending_flash_operation = FLASH_OP_LOAD;
        pending_slot_number = slot;
        current_slot = slot;
    }
    else
    {
        // 参数未注册，直接设置当前存档号
        current_slot = slot;
    }
}

/**
 * @brief 保存到指定存档
 */
void menu_save_slot(uint8_t slot)
{
    if(slot >= MENU_SLOT_COUNT) return;

    // 设置延迟执行标志
    pending_flash_operation = FLASH_OP_SAVE;
    pending_slot_number = slot;
    current_slot = slot;
}

/**
 * @brief 获取当前存档号
 */
uint8_t menu_get_current_slot(void)
{
    return current_slot;
}

//====================================================菜单控制====================================================

/**
 * @brief 设置根菜单
 */
void menu_set_root(menu_item_t* root_menu)
{
    if(root_menu == NULL) return;  // 安全检查

    current_menu = root_menu;
    menu_depth = 0;
    current_index = 0;
    display_offset = 0;

    // 计算菜单项数量
    current_item_count = 0;
    while(current_menu[current_item_count].name[0] != '\0')
    {
        current_item_count++;
        if(current_item_count >= MENU_MAX_ITEMS) break;
    }

    menu_request_refresh(REFRESH_FULL);  // 设置根菜单需要全屏刷新
}

/**
 * @brief 进入子菜单
 */
void menu_enter_submenu(menu_item_t* submenu)
{
    if(menu_depth >= MENU_MAX_DEPTH - 1) return;

    // 保存当前菜单与位置到堆栈
    menu_stack[menu_depth] = current_menu;
    menu_index_stack[menu_depth] = current_index;
    menu_offset_stack[menu_depth] = display_offset;
    menu_depth++;

    // 设置新菜单
    current_menu = submenu;
    current_index = 0;
    display_offset = 0;

    // 计算菜单项数量
    current_item_count = 0;
    while(current_menu[current_item_count].name[0] != '\0')
    {
        current_item_count++;
        if(current_item_count >= MENU_MAX_ITEMS) break;
    }

    menu_request_refresh(REFRESH_FULL);  // 进入子菜单需要全屏刷新
}

/**
 * @brief 返回上级菜单
 */
void menu_return_to_parent(void)
{
    if(menu_depth > 0)
    {
        menu_depth--;
        current_menu = menu_stack[menu_depth];
        current_index = menu_index_stack[menu_depth];
        display_offset = menu_offset_stack[menu_depth];

        // 重新计算菜单项数量
        current_item_count = 0;
        while(current_menu[current_item_count].name[0] != '\0')
        {
            current_item_count++;
            if(current_item_count >= MENU_MAX_ITEMS) break;
        }

        if(current_item_count == 0)
        {
            current_index = 0;
            display_offset = 0;
        }
        else
        {
            if(current_index >= current_item_count)
            {
                current_index = current_item_count - 1;
            }
            if(current_index < display_offset)
            {
                display_offset = current_index;
            }
            if(current_index >= display_offset + 8)
            {
                display_offset = (current_index >= 7) ? (current_index - 7) : 0;
            }
        }
    }
    else
    {
        // 在主菜单时，重置到第一项
        current_index = 0;
        display_offset = 0;
    }

    menu_state = MENU_STATE_NORMAL;
    ips114_clear();  // 返回菜单前强制清屏
    menu_request_refresh(REFRESH_FULL);  // 返回上级菜单需要全屏刷新
}

/**
 * @brief 重置选择到第一项
 */
void menu_reset_to_first(void)
{
    current_index = 0;
    display_offset = 0;
    menu_state = MENU_STATE_NORMAL;
    menu_request_refresh(REFRESH_FULL);  // 重置到第一项需要全屏刷新
}

uint8_t menu_enter_external_view(const menu_external_view_config_t *config)
{
    if((config == NULL) || (config->render == NULL) ||
       (car_menu_is_runtime_locked() != 0U) ||
       (menu_state != MENU_STATE_NORMAL))
    {
        return 1U;
    }

    external_view_config = *config;
    menu_state = MENU_STATE_EXTERNAL_VIEW;
    diag_refresh_divider = 0U;
    external_view_config.render();
    return 0U;
}

uint8_t menu_external_view_runtime_active(void)
{
    return ((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
            (external_view_config.allow_runtime_locked != 0U)) ? 1U : 0U;
}

static void menu_exit_external_view(uint8_t wait_key_release)
{
    if(external_view_config.on_exit != NULL)
    {
        external_view_config.on_exit();
    }
    memset(&external_view_config, 0, sizeof(external_view_config));
    menu_state = MENU_STATE_NORMAL;
    diag_refresh_divider = 0U;
    if(wait_key_release != 0U)
    {
        menu_wait_key_release = 1U;
        menu_clear_key_tracking();
    }
    menu_request_refresh(REFRESH_FULL);
}

//====================================================按键处理====================================================

/**
 * @brief 处理按键事件（检查按键状态并处理）
 */
void menu_process_keys(void)
{
    uint8_t all_released = 1U;
    uint8_t i;

    if(menu_wait_key_release != 0U)
    {
        for(i = 0U; i < KEY_NUMBER; i++)
        {
            if(gpio_get_level(menu_key_pins[i]) != KEY_RELEASE_LEVEL)
            {
                all_released = 0U;
            }
        }

        menu_clear_key_tracking();
        if(all_released != 0U)
        {
            menu_wait_key_release = 0U;
        }
        return;
    }

    if((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
       (external_view_config.long_back_only != 0U) &&
       (key_get_state(KEY_4) == KEY_LONG_PRESS))
    {
        key_clear_state(KEY_4);
        menu_exit_external_view(1U);
        return;
    }

    // 检测按键按下事件（使用逐飞按键库的状态）
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        if(!key_press_consumed[KEY_1])
        {
            menu_key_handler(KEY_UP);
        }
        key_clear_state(KEY_1);
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        if(!key_press_consumed[KEY_2])
        {
            menu_key_handler(KEY_DOWN);
        }
        key_clear_state(KEY_2);
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        menu_key_handler(KEY_ENTER);
        key_clear_state(KEY_3);
    }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        menu_key_handler(KEY_BACK);
        key_clear_state(KEY_4);
    }

    // 自定义长按加速处理（基于按键实时电平）
    for(i = 0U; i < 2U; i++)
    {
        if(gpio_get_level(menu_key_pins[i]) != KEY_RELEASE_LEVEL)
        {
            if(key_hold_ticks[i] < 0xFF)
            {
                key_hold_ticks[i]++;
            }

            if(key_hold_ticks[i] == 1)
            {
                menu_key_handler((i == KEY_1) ? KEY_UP : KEY_DOWN);
                key_press_consumed[i] = 1;
                key_repeat_counters[i] = KEY_REPEAT_FIRST_DELAY_TICKS;
            }
            else if(key_repeat_counters[i] > 0)
            {
                key_repeat_counters[i]--;
            }
            else
            {
                menu_key_handler((i == KEY_1) ? KEY_UP : KEY_DOWN);
                key_repeat_counters[i] = KEY_REPEAT_CONT_DELAY_TICKS;
            }
        }
        else
        {
            key_hold_ticks[i] = 0;
            key_repeat_counters[i] = 0;
            key_press_consumed[i] = 0;
        }
    }
}

/**
 * @brief 按键处理
 */
void menu_key_handler(menu_key_t key)
{
    if(current_menu == NULL) return;
    if((car_menu_is_runtime_locked() != 0U) &&
       (menu_external_view_runtime_active() == 0U)) return;

    switch(menu_state)
    {
        case MENU_STATE_NORMAL:
            switch(key)
            {
                case KEY_UP:
                    if(current_index > 0)
                    {
                        current_index--;

                        // 分页逻辑: 如果光标移出当前页,翻到上一页
                        if(current_index < display_offset)
                        {
                            // 整页向上翻
                            if(display_offset >= MENU_MAX_VISIBLE_LINES)
                                display_offset -= MENU_MAX_VISIBLE_LINES;
                            else
                                display_offset = 0;

                            menu_request_refresh(REFRESH_FULL);  // 翻页时全屏刷新
                        }
                        else
                        {
                            menu_request_refresh(REFRESH_SELECTION);  // 页内移动，局部刷新
                        }
                    }
                    else
                    {
                        // 循环到最后一项
                        current_index = current_item_count - 1;

                        // 跳转到最后一页
                        if(current_item_count > MENU_MAX_VISIBLE_LINES)
                        {
                            // 计算最后一页的起始偏移
                            uint8_t last_page_start = ((current_item_count - 1) / MENU_MAX_VISIBLE_LINES) * MENU_MAX_VISIBLE_LINES;
                            display_offset = last_page_start;
                        }
                        else
                        {
                            display_offset = 0;
                        }

                        menu_request_refresh(REFRESH_FULL);  // 跨页跳转，全屏刷新
                    }
                    break;

                case KEY_DOWN:
                    if(current_index < current_item_count - 1)
                    {
                        current_index++;

                        // 分页逻辑: 如果光标移出当前页,翻到下一页
                        if(current_index >= display_offset + MENU_MAX_VISIBLE_LINES)
                        {
                            // 整页向下翻
                            display_offset += MENU_MAX_VISIBLE_LINES;

                            // 防止越界
                            if(display_offset + MENU_MAX_VISIBLE_LINES > current_item_count)
                            {
                                display_offset = ((current_item_count - 1) / MENU_MAX_VISIBLE_LINES) * MENU_MAX_VISIBLE_LINES;
                            }

                            menu_request_refresh(REFRESH_FULL);  // 翻页时全屏刷新
                        }
                        else
                        {
                            menu_request_refresh(REFRESH_SELECTION);  // 页内移动，局部刷新
                        }
                    }
                    else
                    {
                        // 循环到第一项
                        current_index = 0;
                        display_offset = 0;

                        menu_request_refresh(REFRESH_FULL);  // 跨页跳转，全屏刷新
                    }
                    break;

                case KEY_ENTER:
                    if(current_index >= current_item_count) break;  // 安全检查

                    switch(current_menu[current_index].type)
                    {
                        case MENU_TYPE_SUBMENU:
                            if(current_menu[current_index].submenu != NULL)
                                menu_enter_submenu(current_menu[current_index].submenu);
                            break;

                        case MENU_TYPE_FUNCTION:
                            if(current_menu[current_index].function != NULL)
                            {
                                current_menu[current_index].function();
                                menu_request_refresh(REFRESH_FULL);  // 函数执行后全屏刷新
                            }
                            break;

                        case MENU_TYPE_DIAG_VIEW:
                            if(current_menu[current_index].function != NULL)
                            {
                                menu_state = MENU_STATE_DIAG_VIEW;
                                diag_refresh_divider = 0U;
                                current_menu[current_index].function();
                            }
                            break;

                        case MENU_TYPE_AIR_COMMAND:
                            (void)menu_air_command_start(current_menu[current_index].param_index);
                            break;

                        case MENU_TYPE_PARAMETER:
                        case MENU_TYPE_AIR_PARAMETER:
                        case MENU_TYPE_EXTERNAL_PARAMETER:
                            if(menu_is_param_item(&current_menu[current_index]) != 0U)
                            {
                                if((current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER) &&
                                   (menu_can_edit_air_params() == 0U))
                                {
                                    menu_show_error((menu_is_air_connected() == 0U) ?
                                                    "Air Offline" :
                                                    ((car_menu_is_runtime_locked() != 0U) ?
                                                     "Runtime Lock" : "Air Not Ready"));
                                    break;
                                }
                                if((current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER) &&
                                   (menu_air_edit_begin(current_menu[current_index].param_index) != 0U))
                                {
                                    menu_show_error("Set Fail");
                                    break;
                                }
                                menu_state = MENU_STATE_EDIT;
                                menu_request_refresh(REFRESH_VALUE);  // 进入编辑模式，只是颜色变化
                            }
                            break;
                    }
                    break;

                case KEY_BACK:
                    if(menu_air_command_is_active() != 0U)
                    {
                        (void)menu_air_command_stop();
                    }
                    else if(menu_depth > 0)
                    {
                        menu_return_to_parent();
                    }
                    else
                    {
                        current_index = 0;
                        display_offset = 0;
                        menu_request_refresh(REFRESH_SELECTION);
                    }
                    break;
            }
            break;

        case MENU_STATE_EDIT:
            if((current_index < current_item_count) &&
               (menu_is_param_item(&current_menu[current_index]) != 0U))
            {
                float current_val = 0.0f;
                float step = 0.0f;

                switch(key)
                {
                    case KEY_UP:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            if((menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                               (menu_air_edit_adjust(&current_menu[current_index], step) == 0U))
                            {
                                menu_request_refresh(REFRESH_VALUE);
                            }
                            else
                            {
                                menu_air_edit_reset();
                                menu_state = MENU_STATE_NORMAL;
                                menu_show_error("Set Fail");
                            }
                            break;
                        }

                        if((menu_get_item_param_value(&current_menu[current_index], &current_val) != 0U) &&
                           (menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                           (menu_set_item_param_value(&current_menu[current_index], current_val + step) == 0U))
                        {
                            menu_request_refresh(REFRESH_VALUE);  // 参数值变化，局部刷新
                        }
                        else
                        {
                            menu_state = MENU_STATE_NORMAL;
                            menu_show_error("Set Fail");
                        }
                        break;

                    case KEY_DOWN:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            if((menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                               (menu_air_edit_adjust(&current_menu[current_index], -step) == 0U))
                            {
                                menu_request_refresh(REFRESH_VALUE);
                            }
                            else
                            {
                                menu_air_edit_reset();
                                menu_state = MENU_STATE_NORMAL;
                                menu_show_error("Set Fail");
                            }
                            break;
                        }

                        if((menu_get_item_param_value(&current_menu[current_index], &current_val) != 0U) &&
                           (menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                           (menu_set_item_param_value(&current_menu[current_index], current_val - step) == 0U))
                        {
                            menu_request_refresh(REFRESH_VALUE);  // 参数值变化，局部刷新
                        }
                        else
                        {
                            menu_state = MENU_STATE_NORMAL;
                            menu_show_error("Set Fail");
                        }
                        break;

                    case KEY_ENTER:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            menu_state = MENU_STATE_NORMAL;
                            (void)menu_air_edit_commit(&current_menu[current_index]);
                            break;
                        }
                        // 确认键：保存参数并退出编辑模式
                        menu_state = MENU_STATE_NORMAL;
                        menu_request_refresh(REFRESH_VALUE);  // 退出编辑模式，只是颜色变化
                        break;

                    case KEY_BACK:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            menu_state = MENU_STATE_NORMAL;
                            (void)menu_air_edit_commit(&current_menu[current_index]);
                            break;
                        }
                        // 返回键：保存参数并退出编辑模式（与确认键相同）
                        menu_state = MENU_STATE_NORMAL;
                        menu_request_refresh(REFRESH_VALUE);  // 退出编辑模式，只是颜色变化
                        break;
                }
            }
            break;

        case MENU_STATE_DIAG_VIEW:
            if((key == KEY_BACK) || (key == KEY_ENTER))
            {
                menu_state = MENU_STATE_NORMAL;
                diag_refresh_divider = 0U;
                menu_request_refresh(REFRESH_FULL);
            }
            break;

        case MENU_STATE_EXTERNAL_VIEW:
            if((key == KEY_BACK) &&
               (external_view_config.long_back_only == 0U))
            {
                menu_exit_external_view(0U);
            }
            break;
    }
}

//====================================================Flash存档实现====================================================

/**
 * @brief 检查存档是否存在（简化版本）
 */
uint8_t menu_flash_check_slot(uint8_t slot)
{
    return menu_flash_slot_valid(slot, NULL);
}

/**
 * @brief 从Flash加载参数，只接受带magic/version/checksum的新格式。
 */
void menu_flash_load_params(uint8_t slot)
{
    menu_flash_slot_header_t header;
    uint32_t offset;
    uint16_t index;
    float value;

    if(slot >= MENU_SLOT_COUNT || param_count == 0U)
    {
        menu_show_error("Error");
        return;
    }

    if(menu_flash_slot_valid(slot, &header) == 0U)
    {
        menu_show_error("No Data");
        return;
    }

    menu_show_progress("Loading...");
    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);

    for(index = 0U; index < header.param_count; index++)
    {
        if(param_configs[index].variable != NULL)
        {
            value = flash_union_buffer[offset + index].float_type;
            if(value < param_configs[index].min_val)
            {
                value = param_configs[index].min_val;
            }
            if(value > param_configs[index].max_val)
            {
                value = param_configs[index].max_val;
            }
            *(param_configs[index].variable) = value;
        }
    }

    current_slot = slot;
}

/**
 * @brief 保存参数到Flash，写入magic/version/checksum防止垃圾值进控制器。
 */
void menu_flash_save_params(uint8_t slot)
{
    menu_flash_slot_header_t *header;
    uint32_t page_num;
    uint32_t offset;
    uint16_t index;

    if(slot >= MENU_SLOT_COUNT || param_count == 0U)
    {
        menu_show_error("Error");
        return;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_error("Error");
        return;
    }

    menu_show_progress("Saving...");
    flash_buffer_clear();

    header = (menu_flash_slot_header_t *)flash_union_buffer;
    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);
    if((offset + param_count) > FLASH_PAGE_LENGTH)
    {
        menu_show_error("Error");
        return;
    }

    for(index = 0U; index < param_count; index++)
    {
        if(param_configs[index].variable != NULL)
        {
            flash_union_buffer[offset + index].float_type = *(param_configs[index].variable);
        }
    }

    header->magic = MENU_MAGIC_NUMBER;
    header->version = MENU_VERSION;
    header->slot_id = slot;
    header->param_count = param_count;
    header->data_checksum = menu_flash_calc_data_checksum(param_count, offset);
    header->header_checksum = menu_flash_calc_header_checksum(header);
    memset(header->slot_name, 0, sizeof(header->slot_name));
    sprintf(header->slot_name, "Slot%u", (unsigned int)slot);

    (void)flash_write_page_from_buffer(0U, page_num, offset + param_count);
    current_slot = slot;
}

/**
 * @brief 格式化指定存档槽
 */
void menu_flash_format_slot(uint8_t slot)
{
    uint32_t page_num;
    uint8_t index;

    if(slot >= MENU_SLOT_COUNT)
    {
        menu_show_message("Invalid slot");
        return;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_message("Page error");
        return;
    }

    for(index = 0U; index < MENU_SLOT_SIZE; index++)
    {
        flash_erase_page(0U, page_num + index);
    }

    menu_show_message("Formatted");
}

/**
 * @brief 渲染当前菜单 (分页模式，每页8项)
 */
void menu_render_current(void)
{
    if(current_menu == NULL || current_item_count == 0) return;  // 安全检查

    // 清除屏幕并设置默认显示颜色
    ips114_clear();
    ips114_set_font(UI_FONT_NORMAL);

    // 计算当前页信息（暂未使用，预留给未来的页码显示功能）
    // uint8_t total_pages = (current_item_count + MENU_MAX_VISIBLE_LINES - 1) / MENU_MAX_VISIBLE_LINES;
    // uint8_t current_page = display_offset / MENU_MAX_VISIBLE_LINES + 1;

    // 显示菜单项 (当前页的所有项)
    uint8_t display_count = (current_item_count > MENU_MAX_VISIBLE_LINES) ? MENU_MAX_VISIBLE_LINES : current_item_count;

    for(uint8_t i = 0; i < display_count; i++)
    {
        uint8_t item_index = display_offset + i;
        if(item_index >= current_item_count) break;

        uint8_t selected = (item_index == current_index);
        uint8_t editing = (selected && menu_state == MENU_STATE_EDIT);

        menu_render_item(i, &current_menu[item_index], selected, editing);
    }

    if(current_menu[0].type == MENU_TYPE_AIR_COMMAND)
    {
        menu_air_cmd_status_t command_status;
        const char *state_text = "IDLE";

        menu_get_air_command_status(&command_status);
        if(command_status.state == MENU_AIR_CMD_STATE_WAIT_START_ACK)
        {
            state_text = "WAIT ACK";
        }
        else if(command_status.state == MENU_AIR_CMD_STATE_INSTANT_RUNNING)
        {
            state_text = "RUNNING";
        }
        else if(command_status.state == MENU_AIR_CMD_STATE_WAIT_EXIT_ACK)
        {
            state_text = "WAIT EXIT";
        }

        ips114_set_color(UI_COLOR_VALUE, UI_COLOR_BG);
        ips114_show_string(0, 96, state_text);
        if(command_status.last_ack_text[0] != '\0')
        {
            ips114_show_string(0, 112, command_status.last_ack_text);
        }
    }
}

/**
 * @brief 渲染菜单项 (优化颜色和选中效果)
 */
void menu_render_item(uint8_t line, menu_item_t* item, uint8_t selected, uint8_t editing)
{
    float value = 0.0f;

    if(item == NULL) return;  // 安全检查

    // 边界检查: 不渲染超出可视区域的行 (0-7行)
    if(line >= MENU_MAX_VISIBLE_LINES) return;

    char display_text[32];
    uint8_t y = line * 16;

    // Y坐标二次检查 (135px高度，8行×16px=128px)
    if(y > 120) return;  // 留点余量

    // 设置颜色
    if((item->type == MENU_TYPE_AIR_COMMAND) &&
       (menu_air_command_is_running(item->param_index) != 0U))
    {
        ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);
    }
    else if(editing)
    {
        ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);  // 黄色编辑状态
    }
    else if(selected)
    {
        ips114_set_color(UI_COLOR_SELECTED, UI_COLOR_BG); // 绿色选中项
    }
    else
    {
        ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);   // 白色正常项
    }

    // 构建显示文本 (左右对齐布局)
    if(menu_is_param_item(item) != 0U)
    {
        // 检查参数索引是否有效
        if(menu_get_item_param_value(item, &value) == 0U) return;

        // 参数名称 (左对齐)
        char param_name[20];
        if(selected && !editing)
        {
            snprintf(param_name, sizeof(param_name), ">%s", item->name);  // 选中标记
        }
        else
        {
            snprintf(param_name, sizeof(param_name), " %s", item->name);  // 普通显示
        }
        param_name[18] = '\0';

        // 参数值 (右对齐，3位小数)
        char param_value[12];
        sprintf(param_value, "%7.3f", (double)value);  // 右对齐，宽度7，3位小数

        // 分别显示参数名和参数值
        ips114_show_string(0, y, param_name);              // 左侧显示名称

        // 计算右对齐位置
        uint8_t value_x = 240 - strlen(param_value) * 8;   // 右对齐位置
        ips114_show_string(value_x, y, param_value);       // 右侧显示数值
    }
    else
    {
        // 非参数项 (普通菜单项)
        sprintf(display_text, " %s", item->name);

        // 添加现代化选择标记 (仅普通选中状态)
        if(selected && !editing)
        {
            display_text[0] = 0x10;  // 使用三角箭头符号 (如果字库支持)
            // 如果不支持特殊符号，则使用 '>'
            if(display_text[0] == 0x10) display_text[0] = '>';
        }

        // 显示文本
        ips114_show_string(0, y, display_text);
    }
}

/**
 * @brief 显示消息 (已废弃，建议使用专用的显示函数)
 */
void menu_show_message(const char* msg)
{
    ips114_clear();
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_show_string(0, 0, msg);
    // 简单延时显示
    for(volatile uint32_t i = 0; i < 8000000; i++);
    menu_request_refresh(REFRESH_FULL);  // 消息显示后需要全屏刷新
}

/**
 * @brief 显示成功消息 (绿色大字体)
 */
void menu_show_success(const char* msg)
{
    ips114_clear();
    ips114_set_color(UI_COLOR_SUCCESS, UI_COLOR_BG);
    ips114_set_font(UI_FONT_LARGE);

    // 精确居中显示重要消息
    uint16_t msg_len = strlen(msg);
    uint16_t x = (135 - msg_len * 8) / 2;  // 水平居中
    uint16_t y = (240 - 16) / 2;           // 垂直居中（240是屏幕高度，16是字体高度）

    ips114_show_string(x, y, msg);

    // 成功消息显示1秒
    for(volatile uint32_t i = 0; i < 4000000; i++);
    menu_request_refresh(REFRESH_FULL);  // 成功消息后需要全屏刷新
}

/**
 * @brief 显示错误消息 (红色大字体)
 */
void menu_show_error(const char* msg)
{
    ips114_clear();
    ips114_set_color(UI_COLOR_ERROR, UI_COLOR_BG);
    ips114_set_font(UI_FONT_LARGE);

    // 精确居中显示重要消息
    uint16_t msg_len = strlen(msg);
    uint16_t x = (135 - msg_len * 8) / 2;  // 水平居中
    uint16_t y = (240 - 16) / 2;           // 垂直居中（240是屏幕高度，16是字体高度）

    ips114_show_string(x, y, msg);

    // 错误消息显示2秒
    for(volatile uint32_t i = 0; i < 8000000; i++);
    menu_request_refresh(REFRESH_FULL);  // 错误消息后需要全屏刷新
}

/**
 * @brief 显示进度消息 (黄色，不自动清除)
 */
void menu_show_progress(const char* msg)
{
    ips114_clear();
    ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);
    ips114_set_font(UI_FONT_LARGE);

    // 精确居中显示
    uint16_t msg_len = strlen(msg);
    uint16_t x = (135 - msg_len * 8) / 2;  // 水平居中
    uint16_t y = (240 - 16) / 2;           // 垂直居中

    ips114_show_string(x, y, msg);
    // 进度消息不自动清除，需要手动调用其他显示函数
}

/**
 * @brief 设置显示颜色
 */
void menu_set_display_color(uint16_t text_color)
{
    ips114_set_color(text_color, UI_COLOR_BG);
}

/**
 * @brief 清除屏幕
 */
void menu_clear_screen(void)
{
    ips114_clear();
}

//====================================================局部刷新优化====================================================

/**
 * @brief 请求指定类型的刷新
 */
void menu_request_refresh(refresh_type_t type)
{
    if(type > refresh_type) {
        refresh_type = type;  // 优先级：FULL > SELECTION > VALUE > NONE
    }
    need_refresh = 1;
}

/**
 * @brief 清除指定行
 */
void menu_clear_line(uint8_t line)
{
    uint8_t y = line * 16;

    // Y坐标检查
    if(y > 120) return;

    // 方法1: 使用背景色填充整行
    ips114_set_color(UI_COLOR_BG, UI_COLOR_BG);

    // 方法2: 用足够长的空格字符串覆盖整行 (确保覆盖所有可能的字符)
    char clear_line[20];  // 增加长度，确保覆盖完整
    for(uint8_t i = 0; i < 18; i++) {  // 增加空格数量
        clear_line[i] = ' ';
    }
    clear_line[18] = '\0';

    // 从行首开始覆盖
    ips114_show_string(0, y, clear_line);

    // 额外保险：从稍后位置再次覆盖
    ips114_show_string(8, y, clear_line);
}

/**
 * @brief 渲染单个菜单项（用于局部刷新）
 */
void menu_render_single_item(uint8_t item_index)
{
    if(current_menu == NULL || item_index >= current_item_count) return;

    // 计算显示行号
    if(item_index < display_offset || item_index >= display_offset + 15) return; // 不在显示范围内

    uint8_t line = item_index - display_offset;
    uint8_t selected = (item_index == current_index);
    uint8_t editing = (selected && menu_state == MENU_STATE_EDIT);

    // 先清除该行，再渲染
    menu_clear_line(line);
    menu_render_item(line, &current_menu[item_index], selected, editing);
}

/**
 * @brief 优化的渲染函数（局部刷新）
 */
void menu_render_current_optimized(void)
{
    if(current_menu == NULL || current_item_count == 0) return;

    switch(refresh_type)
    {
        case REFRESH_FULL:
            // 全屏刷新：直接调用完整渲染函数（包含页码指示器）
            menu_render_current();
            break;

        case REFRESH_SELECTION:
            // 选择项刷新：只重绘上次选中项和当前选中项
            ips114_set_font(UI_FONT_NORMAL);

            // 重绘上次选中的项（清除高亮）
            if(last_selected_index != current_index) {
                menu_render_single_item(last_selected_index);
            }

            // 重绘当前选中的项（添加高亮）
            menu_render_single_item(current_index);
            break;

        case REFRESH_VALUE:
            // 数值刷新：只重绘当前选中项的数值
            // 检查状态是否变化（编辑→普通 或 普通→编辑）
            ips114_set_font(UI_FONT_NORMAL);

            if(last_menu_state != menu_state) {
                // 状态变化时，必须先清除整行再重绘
                uint8_t line = current_index - display_offset;
                if(line < MENU_MAX_VISIBLE_LINES) {
                    menu_clear_line(line);
                }
            }

            menu_render_single_item(current_index);
            break;

        case REFRESH_NONE:
        default:
            // 无需刷新
            return;
    }

    // 更新状态
    last_selected_index = current_index;
    last_menu_state = menu_state;
    refresh_type = REFRESH_NONE;
}

/**
 * @brief 设置需要刷新标志
 */
void menu_set_need_refresh(void)
{
    ips114_clear();  // 外部请求刷新时强制清屏，确保显示完整
    menu_request_refresh(REFRESH_FULL);  // 外部请求刷新，使用全屏刷新
}

/**
 * @brief 显示参数值（内部函数）
 */
void menu_show_param_value(uint8_t x, uint8_t y, uint8_t index, uint8_t editing)
{
    if(index >= param_count) return;

    char value_str[16];
    float value = menu_get_param_by_index(index);
    sprintf(value_str, "%.2f", (double)value);

    if(editing)
    {
        // 编辑模式，添加特殊标记
        char edit_str[20];
        sprintf(edit_str, "*%s*", value_str);
        ips114_show_string(x, y, edit_str);
    }
    else
    {
        ips114_show_string(x, y, value_str);
    }
}
