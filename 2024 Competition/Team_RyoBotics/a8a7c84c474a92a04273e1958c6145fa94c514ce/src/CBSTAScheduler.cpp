#include "CBSTAScheduler.h"
#include <iostream>
using namespace SchedulerUtils;
using namespace ECBSTA_NS;

// コスト行列を作成する補助関数
std::vector<std::vector<int>> createCostMatrix(
    const std::unordered_set<int> &free_agents,
    const std::unordered_set<int> &available_tasks,
    SharedEnvironment *env,
    const Graph &graph)
{
    std::vector<int> agent_list(free_agents.begin(), free_agents.end());
    std::vector<int> task_list(available_tasks.begin(), available_tasks.end());

    std::vector<std::vector<int>> cost_matrix(agent_list.size(),
                                              std::vector<int>(task_list.size()));

    // 各エージェントと各タスク間のコストを計算
    for (size_t i = 0; i < agent_list.size(); ++i)
    {
        int agent_id = agent_list[i];
        int agent_loc = env->curr_states[agent_id].location;

        for (size_t j = 0; j < task_list.size(); ++j)
        {
            int task_id = task_list[j];
            const auto &task = env->task_pool.find(task_id)->second;

            // タスクが実行中の場合は大きなコストを設定
            if (task.idx_next_loc > 0)
            {
                cost_matrix[i][j] = std::numeric_limits<int>::max();
                continue;
            }

            // エージェントの現在位置からタスクの目標位置までの推定距離をコストとする
            cost_matrix[i][j] = graph.getEstimatedDistance(agent_loc, task.locations[task.idx_next_loc]);
        }
    }

    return cost_matrix;
}

void schedule_plan_CBSTA(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    // 現在の状態を更新
    updateRunningTasks(env);
    updateTaskAssignments(env);
    updateAgents(env);

    // グラフの初期化
    Graph graph(env->map, env->rows, env->cols);

    // 利用可能なエージェントとタスクのリストを取得
    const auto &free_agents = global_available_agents;
    const auto &available_tasks = global_available_tasks;

    if (free_agents.empty() || available_tasks.empty())
    {
        return; // 割り当て可能なエージェントまたはタスクがない
    }

    // コスト行列の作成
    // auto cost_matrix = createCostMatrix(free_agents, available_tasks, env, graph);

    // CBSTAソルバーの初期化
    ECBSTA solver(graph);

    // 実行中のタスクに関する制約を設定
    // std::unordered_set<int> running_tasks;
    // for (const auto &[task_id, task] : env->task_pool)
    // {
    //     if (task.idx_next_loc > 0)
    //     {
    //         running_tasks.insert(task_id);
    //     }
    // }

    // エージェントとタスクの位置情報を収集
    std::vector<int> starts;
    std::vector<int> goals;
    std::unordered_map<int, int> agent_to_index; // agent_id -> index in starts
    std::unordered_map<int, int> task_to_index;  // task_id -> index in goals
    std::unordered_map<int, int> index_to_task;  // index in goals -> task_id

    // 1. まず、実行中のタスクとそのエージェントを追加
    for (const auto &[task_id, task] : env->task_pool)
    {
        if (task.idx_next_loc > 0) // タスクがopen状態
        {
            int agent_id = task.agent_assigned;
            agent_to_index[agent_id] = starts.size();
            task_to_index[task_id] = goals.size();

            starts.push_back(env->curr_states[agent_id].location);
            goals.push_back(task.locations[task.idx_next_loc]);
        }
    }

    // 2. 次に、フリーエージェントを追加
    for (int agent_id : free_agents)
    {
        agent_to_index[agent_id] = starts.size();
        starts.push_back(env->curr_states[agent_id].location);
    }

    // 3. 最後に、利用可能なタスクを追加
    for (int task_id : available_tasks)
    {
        const auto &task = env->task_pool.find(task_id)->second;
        if (task.idx_next_loc == 0) // タスクが未open状態
        {
            task_to_index[task_id] = goals.size();
            goals.push_back(task.locations[0]); // 最初の位置
        }
    }
    for (const auto &[task_id, task_idx] : task_to_index)
    {
        index_to_task[task_idx] = task_id;
        std::cout << "task_id " << task_id << " index " << task_idx << std::endl;
    }

    // 固定割り当ての制約を作成
    std::unordered_set<ECBSTA_NS::AgentTaskPair> I; // 強制割り当て
    std::unordered_set<ECBSTA_NS::AgentTaskPair> O; // 割り当て禁止

    // 実行中のタスクとエージェントの組み合わせを強制割り当てに追加
    for (const auto &[task_id, task] : env->task_pool)
    {
        if (task.idx_next_loc > 0)
        {
            ECBSTA_NS::AgentTaskPair pair;
            pair.agent = agent_to_index[task.agent_assigned];
            pair.task = task_to_index[task_id];
            I.insert(pair); // このタスクは必ず割り当てる
            std::cout << "強制割り当て: " << pair.agent << " " << pair.task << std::endl;

            // このタスクは他のエージェントには割り当てない
            for (const auto &[other_agent_id, other_agent_idx] : agent_to_index)
            {
                if (other_agent_id != task.agent_assigned)
                {
                    ECBSTA_NS::AgentTaskPair pair;
                    pair.agent = other_agent_idx;
                    pair.task = task_to_index[task_id];
                    O.insert(pair);
                }
            }
        }
    }

    // CBSTAで解を計算
    ECBSTA_NS::HighLevelNode solution_node = solver.solve(starts, goals, time_limit, I, O);
    std::cout << "solution_node.solution.size() = " << solution_node.solution.size() << std::endl;

    // 結果をproposed_scheduleに反映
    if (!solution_node.solution.empty())
    {
        // 新しい割り当てを反映
        for (const auto &[agent_id, agent_idx] : agent_to_index)
        {
            int task_idx = solution_node.assignment[agent_idx];
            int task_id = index_to_task[task_idx];
            proposed_schedule[agent_id] = task_id;
            current_task_assignments[task_id] = agent_id;
        }

        // std::cout << "エージェントごとのパス：" << std::endl;
        // for (const auto &[agent_id, agent_idx] : agent_to_index)
        // {
        //     std::cout << "Agent " << agent_id << " index " << agent_idx << " コスト: " << solution_node.solution[agent_idx].cost << " パス: ";
        //     for (const auto &node : solution_node.solution[agent_idx].nodes)
        //     {
        //         std::pair<int, int> xy = graph.locationToXY(node);
        //         std::cout << "(" << xy.first << ", " << xy.second << ") ";
        //     }
        //     std::cout << std::endl;
        // }

        std::cout << "proposed_schedule as end of CBSTA  : ";
        for (const auto &task : proposed_schedule)
        {
            std::cout << task << " ";
        }
        std::cout << std::endl;
    }
}
