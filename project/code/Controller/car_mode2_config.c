/* Mode2 默认目标点配置
 * 坐标系：全局坐标，原点为里程计起点
 * 路径：(2,2) → (-2,-0.5) → (0,-1.5) 三个点
 */
#include "car_mode2_config.h"


/* 加载默认目标点并重启巡航 */
void car_mode2_load_default_targets(void)
{
    car_mode2_clear_targets();

    (void)car_mode2_add_target(2.00f, 2.00f);
    (void)car_mode2_add_target(-2.00f, -0.50f);
    (void)car_mode2_add_target(0.00f, -1.50f);

    car_mode2_restart_targets();
}
