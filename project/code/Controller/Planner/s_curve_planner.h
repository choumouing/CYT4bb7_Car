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
/*********************************************************************************************************************
* CYT4BB 麦克纳姆轮惯性导航系统 - 7段S曲线速度规划模块 (头文件)
*
* @file    s_curve_planner.h
* @brief   基于Jerk受限的7段S曲线轨迹规划,支持双向运动和可变边界条件
*
* @details
* 算法基于2023年全国大学生智能汽车竞赛一等奖代码改进
*
* 7段时间分配:
*   [0-----Tj1-----Ta-Tj1-----Ta-----Ta+Tv-----Ta+Tv+Tj2-----T-Tj2-----T]
*    |      |         |        |       |           |            |        |
*   v0   Jerk加速   匀加速   Jerk回零  匀速    Jerk减速      匀减速   Jerk回零
*
* 运动学约束:
*   - 速度:   |v(t)| ≤ v_max
*   - 加速度: |a(t)| ≤ a_max
*   - Jerk:   |j(t)| ≤ j_max
*
* 单位约定:
*   - 位移: mm
*   - 速度: mm/s
*   - 加速度: mm/s^2
*   - Jerk: mm/s^3
*   - 时间: s
*
* @author  Claude Code (基于范总国赛代码重构)
* @date    2025-12-10
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _S_CURVE_PLANNER_H_
#define _S_CURVE_PLANNER_H_



/**
 * @brief 7段S曲线速度规划器数据结构
 *
 * @note 字段顺序已优化兼容性:前3个字段必须保持位置不变(被外部直接访问)
 */
typedef struct {
    // ========== 对外接口字段 (必须保留,位置不变) ==========
    float v_target;       /**< 当前目标速度 (mm/s) - 被navigation_task读取 */
    uint8_t is_running;   /**< 运行状态标志 (1:运行中 0:停止) */
    uint8_t is_finished;  /**< 完成状态标志 (1:已完成 0:未完成) */

    // ========== 输入约束参数 ==========
    float j_max;          /**< 最大Jerk (mm/s^3) */
    float a_max;          /**< 最大加速度 (mm/s^2) */
    float v_max;          /**< 最大速度 (mm/s) */

    // ========== 边界条件 ==========
    float v0;             /**< 起始速度 (mm/s, 物理坐标系), 默认0 */
    float v1;             /**< 结束速度 (mm/s, 物理坐标系), 默认0 */

    // ========== 计算结果 ==========
    float sigma;          /**< 运动方向 (+1.0:正向 -1.0:反向) */
    float vlim;           /**< 实际达到的最大速度 (mm/s) */
    float alim_accel;     /**< 加速段实际加速度 (mm/s^2) */
    float alim_decel;     /**< 减速段实际加速度 (mm/s^2) */
    uint8_t flag;         /**< 位移过短标志 (0:正常 1:过短,降级处理) */

    // ========== 时间节点 ==========
    float Tj1;            /**< 加速段Jerk时间 (s) */
    float Ta;             /**< 总加速时间 (s) */
    float Tv;             /**< 匀速时间 (s) */
    float Tj2;            /**< 减速段Jerk时间 (s) */
    float Td;             /**< 总减速时间 (s) */
    float T;              /**< 总运动时间 (s) */

    // ========== 运行时状态 ==========
    float time_current;   /**< 当前时间 (s) */
    uint8_t phase;        /**< 当前阶段 (1-7) */
} s_curve_planner_t;

/**
 * @brief 初始化7段S曲线速度规划器
 *
 * @details
 * 根据给定的运动距离和约束条件,计算7段S曲线的时间节点
 *
 * 算法流程:
 *   1. 参数验证与限幅
 *   2. sigma归一化处理双向运动
 *   3. v0/v1投影到运动方向
 *   4. 最小位移检查(distance<5mm直接完成)
 *   5. 判断amax能否达到(加速段/减速段)
 *   6. 判断vmax能否达到
 *   7. vlim迭代求解(vmax不能达到时)
 *   8. 边界检查和降级处理
 *   9. 初始化运行状态
 *
 * @param[in,out] planner   规划器实例指针
 * @param[in]     distance  运动距离 (mm), 正数向前,负数向后
 * @param[in]     v_max     最大速度 (mm/s), 必须>0
 * @param[in]     a_max     最大加速度 (mm/s^2), 必须>0
 * @param[in]     j_max     最大Jerk (mm/s^3), 必须>0
 *
 * @return 0:成功 -1:参数错误
 *
 * @note 起始/结束速度默认为0。如需设置非零初速度，请在调用前修改planner->v0/v1
 *       (v0/v1应为物理坐标系速度，可正可负，函数内部会自动投影到运动方向)
 * @warning distance=0或<5mm时会立即完成(flag=1)
 *
 * @see s_curve_planner_step()
 */
int s_curve_planner_init(s_curve_planner_t *planner,
                          float distance,
                          float v_max,
                          float a_max,
                          float j_max);

/**
 * @brief 更新7段S曲线速度规划器(周期调用)
 *
 * @details
 * 根据时间推进,计算当前目标速度并更新状态
 * 应在固定周期(如5ms)中调用
 *
 * @param[in,out] planner 规划器实例指针
 * @param[in]     dt      时间步长 (s), 通常为0.005 (5ms)
 *
 * @return 无
 *
 * @note 更新后可通过planner->v_target获取目标速度
 * @note 完成后planner->is_finished会置1
 */
void s_curve_planner_step(s_curve_planner_t *planner, float dt);

#endif // _S_CURVE_PLANNER_H_
