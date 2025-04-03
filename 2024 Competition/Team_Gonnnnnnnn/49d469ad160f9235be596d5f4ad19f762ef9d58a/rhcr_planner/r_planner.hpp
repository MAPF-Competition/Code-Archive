#include <random>
#include <vector>
#include <unordered_set>
#include <utility>
#include <functional>

#include "SharedEnv.h"
#include "ActionModel.h"
#include "r_states2.hpp"
#include "r_planresult.hpp"  //新加的 -hzj
// #include "r_env.hpp"



namespace RHCR_Planner{
    // -hzj
    struct AgentPriority {
        size_t index;
        int distance;
        bool operator<(const AgentPriority& other) const {
            if (distance != other.distance)
                return distance > other.distance;
            return index > other.index;
        }
    };

    // 由于Cost是模板参数，我们需要明确指定类型  -hzj
    extern std::vector<PlanResult<State2, Action, float>> previous_solution;
    extern std::unordered_set<size_t> activeAgents;  // 添加声明

    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto hash1 = std::hash<T1>{}(p.first);
            auto hash2 = std::hash<T2>{}(p.second);
            return hash1 ^ hash2; // 简单的哈希组合
        }
    };

    void map_to_2d(std::vector<int> map, int rows, int cols, std::vector<std::vector<int>> &map_2d, 
    std::unordered_set<std::pair<int, int>,PairHash> &obs_2d); 

    std::pair<int,int> to_2d(int location,SharedEnvironment* env);

    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env);

    Action r_getAction(State2& prev_states2, State2& next_states2);

    // -hzj 新增：计算活跃智能体的函数
    std::unordered_set<size_t> calculateActiveAgents(
        const std::vector<State2>& initialStates,
        const std::vector<std::pair<int,int>>& curr_goals,
        const size_t maxActiveAgents = 1000);  //1500改1000试试 -hzj


}
