#ifndef MENU_AIR_SUPPORT_H
#define MENU_AIR_SUPPORT_H

#include "zf_common_headfile.h"

typedef struct
{
    char name[16];
    float *variable;
    float step;
    float min_val;
    float max_val;
} menu_air_param_config_t;

extern float air_min_area;
extern float air_hold_ms;
extern float air_x_bias;
extern float air_y_bias;

void menu_air_support_init(void);
void menu_register_param_air(const char *name, float *var, float step, float min, float max);
uint8 menu_get_air_param_count(void);
float menu_get_air_param_by_index(uint8 index);
uint8 menu_set_air_param_by_index(uint8 index, float value);
const menu_air_param_config_t *menu_get_air_param_config(uint8 index);
uint8 menu_is_air_connected(void);
uint8 menu_can_edit_air_params(void);
uint8 menu_sync_all_air_params(void);
uint8 menu_load_air_slot(uint8 slot);
uint8 menu_save_air_slot(uint8 slot);

#endif
