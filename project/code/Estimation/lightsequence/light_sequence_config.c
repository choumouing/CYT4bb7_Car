/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "light_sequence_config.h"

/*
 * 每轮直接填写亮灯编号，例如16表示1号和6号灯，456表示4号、5号和6号灯。
 * 每轮设置1至3盏灯，灯号范围为1至6，同一轮灯号不能重复。
 * 每套序列均保证同一轮灯号唯一，并且相邻两轮没有相同灯号。
 */
const light_sequence_preset_t g_light_sequence_presets[LIGHT_SEQUENCE_PRESET_COUNT] =
{
    /* 序列1 */
    {6U, {15U, 42U, 36U, 45U, 12U, 46U}},

    /* 序列2 */
    {6U, {12U, 34U, 56U, 14U, 23U, 45U}},

    /* 序列3 */
    {6U, {15U, 46U, 23U, 14U, 56U, 24U}},

    /* 序列4 */
    {6U, {14U, 23U, 46U, 15U, 43U, 21U}},

};
