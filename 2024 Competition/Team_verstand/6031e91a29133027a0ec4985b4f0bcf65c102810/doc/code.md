```c++
std::vector<list<int>>& tasks; // 所有任务的集合
```

new_tasks是在这个时间步reveal的新任务。

ongoing_tasks是一个id向task指针的map. 

1     init_heuristic用于planner中每个时间步的规划，还有get_h. 

2     init_heuristics用于planner中的初始化，schedule的初始化，还有get_h. 

+ greedy_only_first. 默认分配算法，cost只计算agent和任务pickup点的位置。
+ greedy_sum. 默认分配算法，从默认代码中复制到TaskScheduler.cpp。
+ greedy_sum_at_once. 默认分配算法，一次性把任务长度计算完储存在map。
+ greedy_sum_suburb_first. 优先给靠近地图角落的agent分配任务, 因为地图边缘不容易堵车。
+ greedy_sum_urban_first. 优先给靠近地图中心的agent分配任务, 因为地图中心更容易堵车。
+ greedy_sum_sparse_first. 优先给车流稀疏的agent分配任务, 因为车流稀疏区域不容易堵车。
+ greedy_sum_dense_first. 优先给车流密集的agent分配任务, 因为车流密集区域更容易堵车。
+ hungarian_only_first. 匈牙利分配算法，cost只计算agent和任务pickup点的位置。
+ hungarian_pickup_snatch. 匈牙利分配算法，cost只计算agent和任务pickup点的位置；加入了抢单机制，也就是把被指派任务但还没有取货的agent也考虑在内。
+ hungarian_sum. 匈牙利分配算法, cost使用默认计算方法。
+ hungarian_sum_at_once. 匈牙利分配算法，一次性把任务长度计算完储存在unordered_map。
+ hungarian_sum_snatch. 匈牙利分配算法，一次性把任务长度计算完储存在unordered_map；加入了抢单机制，也就是把被指派任务但还没有取货的agent也考虑在内。
+ hungarian_sum_snatch_complex. 在hungarian_sum_snatch的基础上，考虑了单据不够、单据超出等多种情况。尚且不稳定，official platform测试会失败。
+ pickup_jam_based_current. 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数之和决定。
+ pickup_jam_based_current_square. 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent距离该任务的倒数平方之和决定。
+ adaptive_jam_pickup_current_circle. 以agent为圆心，agent-task为半径朝向task画出一个半圆，位于这个半圆内的other agent计入拥堵系数。jam = cos<ao, at> / |ao| = inner(ao, at) / (|ao||ao||at|) if inner(ao, at) > 0 and |ao| < |at|; =0, otherwise
+ adaptive_jam_task_circle_current. 检索以task为圆心，所有agent距离它的倒数之和
+ --
+ (1) adaptive_jam_task_circle_count_current. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数。// 1
+ adaptive_jam_task_circle_count_current_compare_dist. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数。compare dist以简化计算。
+ hungarian_sum_snatch_adaptive_jam_task_circle_count_current. hungarian_sum_snatch计算cost matrix加上启发式拥堵adaptive_jam_task_circle_count_current. 
+ adaptive_jam_task_region_count_current. 将地图分成若干32x32的区域, task所在区域中agent的数量作为sum jam weight. 
+ hungarian_sum_snatch_adaptive_jam_task_pickup_region_count_current. hungarian_sum_snatch计算cost matrix加上启发式拥堵adaptive_jam_task_pickup_region_count_current. 
+ adaptive_jam_task_region_current_density. 将地图分成12x12的区域, task所在区域agent密度作为sum jam weight. 
+ hungarian_sum_snatch_adaptive_jam_task_pickup_region_current_density. 将地图分成8x8的区域, task pickup区域agent密度作为sum jam weight, 叠加匈牙利抢单算法

+ adaptive_jam_task_region_current_density_times_agent_pickup. 将地图分成12x12的区域, task所在区域agent密度 x |agent-pickup|作为sum jam weight. 

+ hungarian_sum_snatch_adaptive_jam_task_pickup_region_current_density_times_agent_pickup. 将地图分成8x8的区域, task pickup区域agent密度x |agent pickup|作为sum jam weight, 叠加匈牙利抢单算法。
+ 
+ --
+ adaptive_jam_task_circle_count_current_sample. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量（只考察前2048个agent）为拥堵系数。 
+ adaptive_jam_task_circle_count_current_extrapolation. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的数量为拥堵系数，并把这个系数外推出去。
+ --
+ (2) adaptive_jam_task_circle_count_current_busy. 统计以task为圆心，|agent-task|为半径的圆中有任务的other agent的数量作为jam.
+ --
+ adaptive_jam_middle_circle_count_current. 统计以agent-task中点为圆心，|agent-task|/2为半径的圆中other agent的数量作为jam。
+ adaptive_jam_task_circle_vector_current. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
+ adaptive_jam_task_circle_vector_current_busy. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内有任务的other agent的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
+ adaptive_jam_task_circle_vector_current_extrapolation. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数，并把这个系数外推出去。jam = (cost<ao,at> * |at| / |ao|) * (total_dist / |at|) = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
+ adaptive_jam_task_circle_vector_current_complex. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent的计入拥堵系数, 有|agent-task|欧几里得距离作为分母。jam = cost<ao,at> / |ao| = inner(ao, at) / (inner(ao, ao) * |at|) if |to| < |at|; =0, otherwise
+ pickup_jam_based_goal. 每个任务的拥堵系数在一个时间步对所有agent都是相同的。由所有agent目标距离该任务的倒数之和决定。
+ --
+ (3) adaptive_jam_task_circle_count_goal. 统计以task为圆心，|agent-task|为半径的圆中other agent goal的数量作为jam. 
+ adaptive_jam_task_circle_count_goal_compare_dist. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的数量为拥堵系数。compare dist以简化计算。
+ hungarian_sum_snatch_adaptive_jam_task_circle_count_goal. hungarian_sum_snatch计算cost matrix加上启发式拥堵adaptive_jam_task_circle_count_goal. 
+ --
+ (4) adaptive_jam_task_circle_vector_goal. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
+ --
+ (5) adaptive_jam_task_circle_count_both_current_goal. 统计以task为圆心，|agent-task|为半径的圆中other agent current location 和 goal的数量作为jam
+ adaptive_jam_task_circle_count_both_current_goal. 统计以task为圆心，|agent-task|为半径的圆中other agent current location 和 goal的数量作为jam. compare dist以简化计算。
+ hungarian_sum_snatch_adaptive_jam_task_circle_count_both_current_goal. hungarian_sum_snatch计算cost matrix加上启发式拥堵adaptive_jam_task_circle_count_both_current_goal. 
+ --
+ (6) adaptive_jam_task_circle_count_middle_current_goal. 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的数量作为jam. 
+ adaptive_jam_task_circle_count_middle_current_goal_compare_dist. 统计以task为圆心，|agent-task|为半径的圆中, other agent 当前位置和目标点位置的中点的数量作为jam. compare dist以简化计算。
+ hungarian_sum_snatch_adaptive_jam_task_circle_count_middle_current_goal. hungarian_sum_snatch计算cost matrix加上启发式拥堵adaptive_jam_middle_current_goal_task_circle_count. 
+ --
+ (7) adaptive_jam_task_Manhattan_circle_count_middle_current_goal. 统计以task为圆心，|agent-task|为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam. 
+ --
+ (8) adaptive_jam_task_fix_Manhattan_circle_count_middle_current_goal. 统计以task为圆心，|num_rows + num_columns| / 8为半径的Manhattan圆中, other agent 当前位置和目标点位置的中点的数量作为jam. 如果fix能够调好，就意味着区域数数成为可能，大算例可以极大减少时间，和匈牙利的结合也不会导致系数过大。
+ --
+ (9) adaptive_jam_task_region_count_middle_current_goal. 将地图分成32x32的区域, task所在区域中agent当前位置和目标点位置的中点的数量作为作为sum jam weight.
+ 
+ (10) adaptive_jam_task_circle_vector_middle_current_goal. 以task pickup为圆心，agent-task为半径朝向task画出一个圆，位于这个圆内的other agent goal的计入拥堵系数。为了量纲相等还要乘以|at|。jam = cost<ao,at> * |at| / |ao| = inner(ao, at) / inner(ao, ao) if |to| < |at|; =0, otherwise
+ (11) adaptive_jam_curr_pickup_intersect_curr_goal. 统计other agent和它目标点的连线与agent-pickup连线发生交叉的数量。
+ adaptive_jam_curr_pickup_delivery_intersect_curr_goal. 统计other agent和它目标点的连线与agent-pickup连线及pickup-delivery连线发生交叉的数量。



