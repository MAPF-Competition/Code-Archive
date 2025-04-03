
#ifndef pibt_hpp
#define pibt_hpp

#include <vector>
#include <list>
#include <unordered_set>
#include <tuple>
#include "Types.h"
#include "utils.h"
#include "heuristics.h"
#include "TrajLNS.h"
#include "utils.h"

namespace DefaultPlanner{

    /**
* @brief: 提供的不同启发式策略，选择合适的方法来估计从target点到ai路径终点的距离。
* @param TrajLNS& lns: 被计算路径保存的地方
* @param int ai: agent ai的路径。
* @param int target: 被估计的点。
* @return void
*/
int get_gp_h(TrajLNS& lns, int ai, int target);

/**
* @brief: 在一个路径规划算法中为某个agent分配下一步的移动位置。
* @param int _curr_id: 被规划的agent id。
 * @param int _higher_id: 表示当前机器人是否受某个优先级更高的机器人阻塞。如果为 -1，表示没有受其他机器人约束。
 * @param std::vector<State>& _prev_states: 每个agent下达指令前的位置
 * @param std::vector<State>& _next_states: 每个agent下达指令后的位置。
 * @param std::vector<int>& _prev_decision: 每个地图点下达指令前被哪个agent占据
 * @param std::vector<int>& _decision: 每个地图点下达指令后被哪个agent占据
 * @param std::vector<bool>& _occupied: 每个地图点是否被占据
 * @param TrajLNS& _lns: 储存各种信息
 * @return bool: true, 规划成功; false, 规划失败。
 */
bool causalPIBT(int _curr_id, int _higher_id, std::vector<State>& _prev_states,
                std::vector<State>& _next_states,
                std::vector<int>& _prev_decision, std::vector<int>& _decision,
                std::vector<bool>& _occupied, TrajLNS& _lns
	  );

/**
* @brief: 根据当前状态 (prev) 和目标位置 (next_loc)，计算出适合的 行动（Action）
* @param State& prev: 当前状态，包括当前的位置 (location) 和当前的朝向 (orientation)。
 * @param State& next: 下个状态，包括下个的位置 (location) 和下个的朝向 (orientation)。
 * @return Action: 返回适合的行动(Action)
 */
Action getAction(State& prev, State& next);

/**
* @brief: 根据当前状态 (prev) 和目标位置 (next_loc)，计算出适合的 行动（Action）
* @param State& prev: 当前状态，包括当前的位置 (location) 和当前的朝向 (orientation)。
 * @param int next_loc: 智能体希望移动到的目标位置。
 * @param SharedEnvironment* env: 共享的环境对象，提供一些地图信息，比如列数 (cols)。
 * @return Action: 返回适合的行动(Action)
 */
Action getAction(State& prev, int next_loc, SharedEnvironment* env);

/**
* @brief: 检查agent是否移动。
* @param int id: 当前智能体的 ID。
 * @param std::vector<bool>& checked: 记录每个智能体是否已经被检查过，避免重复检查（递归中的循环调用）。
 * @param std::vector<DCR>& decided: 存储每个智能体的决策结果，其中包含目标位置（loc）。
 * @param std::vector<Action>& actions: 当前智能体的行动列表，例如：Action::FW（前进）、Action::W（等待）。
 * @param std::vector<int>& prev_decision: 每个目标位置的决策记录，存储目标位置被哪个智能体占据。如果目标位置没有被占据，值为 -1。
 * @return bool: 返回一个行动（例如移动）是否可以被执行
 */
bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);
}
#endif