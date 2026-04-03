#include "Game_logic.h"
#include <string.h>

/*
 * 工具函数：判断两个网格坐标是否相同。
 */
static int pos_equal(Position a, Position b)
{
    return (a.row == b.row) && (a.col == b.col);
}
/*
 * 工具函数：从数组中按下标删除一个元素（后续元素前移）。
 * 参数说明：
 * - arr:      坐标数组
 * - cnt:      当前有效元素数量（输入输出）
 * - index:    要删除的下标
 */
static void remove_position_at(Position *arr, int *cnt, int index)
{
    int i;

    if (arr == 0 || cnt == 0 || *cnt <= 0)
    {
        return;
    }
    if (index < 0 || index >= *cnt)
    {
        return;
    }

    for (i = index; i < (*cnt - 1); i++)
    {
        arr[i] = arr[i + 1];
    }

    /* 尾部补零，便于调试观察。 */
    arr[*cnt - 1].row = 0;
    arr[*cnt - 1].col = 0;
    (*cnt)--;
}
/*
 * 工具函数：在数组中查找坐标，找到返回下标，找不到返回 -1。
* 参数说明：
 * - arr:      坐标数组
 * - cnt:      数组元素数量
 * - target:   要查找的坐标
 */
static int find_position_index(const Position *arr, int cnt, Position target)
{
    int i;
    for (i = 0; i < cnt; i++)
    {
        if (pos_equal(arr[i], target))
        {
            return i;
        }
    }
    return -1;
}

/*
 * 将“单轮规划路径”拼接到总路径 merged_path。
 * 为避免段与段连接处出现重复点，会在“上一段终点 == 下一段起点”时自动跳过下一段首点。
 */
void append_segment_path(Position *merged_path, int *merged_len,
                               const Position *segment_path, int segment_len)
{
    int start_index = 0;
    int copy_len;

    //如果上一段终点和本段起点相同，则跳过本段第一个点
    if (*merged_len > 0 && pos_equal(merged_path[*merged_len - 1], segment_path[0]))
    {
        start_index = 1;
    }

    copy_len = segment_len - start_index;

    memcpy(&merged_path[*merged_len],
           &segment_path[start_index],
           (size_t)copy_len * sizeof(Position));
    *merged_len += copy_len;
}

/*
 * 应用一轮规划结果到“临时地图状态”：
 * 1) 若使用炸弹：删除被使用炸弹，并模拟爆炸清墙；
 * 2) 删除本轮被推进目标并消失的箱子；
 * 3) 删除对应目标点（箱子进目标后目标也消失）；
 * 4) 更新小车起点为该轮路径终点，供下一轮规划使用。
 */
void apply_round_result(const path_plan_result *plan,
                              int selected_box_index,
                              Position *local_obstacles, int *local_obstacles_cnt,
                              Position *local_bombs, int *local_bombs_cnt,
                              Position *local_boxes, int *local_boxes_cnt,
                              Position *local_targets, int *local_targets_cnt,
                              Position *io_car)
{
    int target_index;

    /* 1) 处理炸弹副作用：消耗炸弹并清除爆炸范围内障碍。 */
    if (plan->used_bomb)
    {
        if (plan->bomb_index >= 0 && plan->bomb_index < *local_bombs_cnt)
        {
            remove_position_at(local_bombs, local_bombs_cnt, plan->bomb_index);
        }

        if (plan->bomb_target.row >= 0 && plan->bomb_target.col >= 0)
        {
            simulate_bomb_explosion(local_obstacles, local_obstacles_cnt, plan->bomb_target);
        }
    }

    /* 2) 移除本轮已完成的箱子。 */
    remove_position_at(local_boxes, local_boxes_cnt, selected_box_index);

    /* 3) 移除被匹配到的目标点（规则：箱子进入目标后目标消失）。 */
    target_index = find_position_index(local_targets, *local_targets_cnt, plan->box_target);
    if (target_index >= 0)
    {
        remove_position_at(local_targets, local_targets_cnt, target_index);
    }
    else if (*local_targets_cnt > 0)
    {
        /*
         * 正常情况下应能按坐标匹配到目标。
         * 若受地图动态扰动导致未匹配到，保底移除一个目标，保持“完成一箱对应一目标”的计数一致。
         */
        remove_position_at(local_targets, local_targets_cnt, 0);
    }

    /* 4) 下一轮从当前轮路径末点继续规划。 */
    *io_car = plan->car_path[plan->total_steps - 1];

}

/*
 * 关卡一路径规划：无箱子目标点关联，寻找最短路径
 *
 * 内部核心：按“剩余箱子数量”连续规划直到全部推完。
 * 规划策略：
 * - 每一轮在“当前临时地图状态”下，尝试所有候选箱子；
 * - 选择本轮可行且路径最短的方案；
 * - 立刻应用该方案对地图状态的影响，再进入下一轮。
 */
void Plan_path_Mode1(void)
{
    // 为避免修改全局地图状态，先将全局地图元素复制到本地临时数组进行规划使用。
    Position local_obstacles[MAX_OBSTACLES];
    Position local_bombs[MAX_BOMBS];
    Position local_boxes[MAX_BOXES];
    Position local_targets[MAX_TARGETS];
    int local_obstacles_cnt = Obstacles_count;
    int local_bombs_cnt = Bombs_count;
    int local_boxes_cnt = Boxes_count;
    int local_targets_cnt = Targets_count;
    memcpy(local_obstacles, obstacles, (size_t)local_obstacles_cnt * sizeof(Position));
    memcpy(local_bombs, bombs, (size_t)local_bombs_cnt * sizeof(Position));
    memcpy(local_boxes, boxes, (size_t)local_boxes_cnt * sizeof(Position));
    memcpy(local_targets, targets, (size_t)local_targets_cnt * sizeof(Position));
    // curr_car 记录每轮规划的起点，初始为全局 car。
    Position curr_car = car;
    // merged_path 用于拼接每轮规划的路径段，最终写回全局 car_path。
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    memset(merged_path, 0, sizeof(merged_path));

    while (local_boxes_cnt > 0)
    {
        // 每轮规划在当前地图状态下尝试所有箱子，选择最优可行方案。
        int best_steps = -1;
        int best_box_index = -1;
        path_plan_result best_plan;
        for (uint8 i = 0; i < local_boxes_cnt; i++)
        {
            path_plan_result current_plan;
            int steps = integrated_path_output(MAP_ROWS, MAP_COLS,
                                               local_obstacles, local_obstacles_cnt,
                                               local_bombs, local_bombs_cnt,
                                               local_boxes, local_boxes_cnt,
                                               local_targets, local_targets_cnt,
                                               i,
                                               curr_car,
                                               &current_plan);
            if (steps > 0 && (best_steps < 0 || steps < best_steps))
            {
                best_steps = steps;
                best_box_index = i;
                best_plan = current_plan;
            }
        }

        // 将本轮最优方案路径段拼接到总路径，并更新总路径长度。
        append_segment_path(merged_path, &merged_len, best_plan.car_path, best_plan.total_steps);

        // 应用本轮规划结果到地图状态，准备下一轮使用。
        apply_round_result(&best_plan,
                            best_box_index,
                            local_obstacles, &local_obstacles_cnt,
                            local_bombs, &local_bombs_cnt,
                            local_boxes, &local_boxes_cnt,
                            local_targets, &local_targets_cnt,
                            &curr_car);
    }

    // 全局输出最终拼接路径。
    memset(car_path, 0, sizeof(car_path));
    if (merged_len > 0)
    {
        memcpy(car_path, merged_path, (size_t)merged_len * sizeof(Position));
    }
    Car_path_count = (size_t)merged_len;
}
