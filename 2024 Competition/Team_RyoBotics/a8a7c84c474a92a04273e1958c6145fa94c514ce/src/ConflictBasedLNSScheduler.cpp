#include "ConflictBasedLNSScheduler.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_set>
using namespace SchedulerUtils;
namespace CBLNSScheduler
{
    std::mt19937 rng(0); // 乱数生成器
    std::vector<std::vector<SchedulerUtils::State>> path;

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment *env)
    {
        path.resize(env->num_of_agents, std::vector<SchedulerUtils::State>(0));
        // 初期化が必要な場合はここで実装
    }
    // タスクの完了時間を計算
    int calculateTaskCompletionTime(int agent_id, int task_id, SharedEnvironment *env)
    {
        if (task_id == -1)
        {
            std::cerr << "task_id is -1" << std::endl;
            return 0;
        }

        const Task &task = env->task_pool[task_id];
        int total_cost = 0;

        // エージェントの現在位置と向きから最初の位置までのコスト
        int current_loc = env->curr_states[agent_id].location;
        int current_dir = env->curr_states[agent_id].orientation;

        // 最初の位置までのコスト
        auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
            current_loc, current_dir, task.locations[0]);
        total_cost = cost_and_dir.first;
        current_dir = cost_and_dir.second;
        // 各経由地点間のコスト
        for (size_t i = 1; i < task.locations.size(); i++)
        {
            auto cost_and_dir = SchedulerUtils::getMinCostAndDirection(
                task.locations[i - 1], current_dir, task.locations[i]);
            total_cost += cost_and_dir.first;
            current_dir = cost_and_dir.second;
        }

        return total_cost;
    }

    // スケジュールの一部を破壊する関数
    std::vector<int> destroy(const std::vector<int> &current_schedule, double destroy_ratio)
    {
        // global_free_agentsをvectorに変換
        std::vector<int> free_agents(SchedulerUtils::global_available_agents.begin(), SchedulerUtils::global_available_agents.end());
        int num_destroy = static_cast<int>(free_agents.size() * destroy_ratio);

        // ランダムにシャッフル
        std::shuffle(free_agents.begin(), free_agents.end(), rng);

        // 必要な数だけ選択
        return std::vector<int>(free_agents.begin(), free_agents.begin() + num_destroy);
    }

    // 破壊された部分を修復する関数
    void repair(std::vector<int> &schedule, const std::unordered_map<int, int> &task_agent_map, const std::vector<int> &destroyed_agents, SharedEnvironment *env)
    {
        // 現在割り当て済みのタスクを記録
        std::unordered_set<int> assigned_tasks;
        std::unordered_set<int> destroyed_agents_set(destroyed_agents.begin(), destroyed_agents.end());

        // 破壊されたエージェントに対して、最も近い未割り当てタスクを割り当てる
        for (int agent_id : destroyed_agents)
        {
            int best_task = -1;
            int min_cost = std::numeric_limits<int>::max();
            for (int task_id : SchedulerUtils::global_available_tasks)
            {
                if (task_agent_map.find(task_id) == task_agent_map.end() && assigned_tasks.find(task_id) == assigned_tasks.end())
                {
                    int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        best_task = task_id;
                    }
                }
            }
            assert(best_task != -1);
            schedule[agent_id] = best_task;
            assigned_tasks.insert(best_task);
        }
    }

    // スケジュールの評価関数（makespanを返す）
    int evaluateSchedule(const std::vector<int> &schedule, SharedEnvironment *env)
    {
        int num_of_conflicts = SchedulerUtils::countAllAgentConflicts(path);
        return num_of_conflicts;
    }
    void generateInitialSchedule(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        const std::unordered_set<unsigned short> &global_available_agents = SchedulerUtils::global_available_agents;
        const std::unordered_set<int> &global_available_tasks = SchedulerUtils::global_available_tasks;
        std::unordered_set<int> assigned_tasks;
        for (int agent_id : global_available_agents)
        {
            if (std::chrono::steady_clock::now() >= end_time)
                break;
            if (env->curr_task_schedule[agent_id] != -1)
                continue;
            // global_available_tasksの中で、env->task_poolから未割り当てのタスクを取得
            // 同時に距離を計算
            std::vector<std::pair<int, int>> candidate_tasks;
            for (int task_id : global_available_tasks)
            {
                if (env->task_pool[task_id].agent_assigned == -1 && assigned_tasks.find(task_id) == assigned_tasks.end())
                {
                    // int cost = SchedulerUtils::evaluateCost(agent_id, task_id, env);
                    int cost = calculateTaskCompletionTime(agent_id, task_id, env);
                    candidate_tasks.push_back({task_id, cost});
                }
            }
            // 最小コストのタスクを見つける
            std::sort(candidate_tasks.begin(), candidate_tasks.end(),
                      [](const auto &a, const auto &b)
                      {
                          return a.second < b.second;
                      });
            int best_task = candidate_tasks[0].first;

            // タスクを割り当て
            if (best_task != -1)
            {
                proposed_schedule[agent_id] = best_task;
                // global_available_tasks.erase(best_task);
                assigned_tasks.insert(best_task);
            }
        }
    }
    void optimizeAssignment(std::vector<int> &proposed_schedule, SharedEnvironment *env, std::chrono::steady_clock::time_point end_time)
    {
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            path[agent_id] = SchedulerUtils::constructPath(env->curr_states[agent_id].location, env->curr_states[agent_id].orientation, env->task_pool[proposed_schedule[agent_id]].locations[0]);
        }
        // std::cout << "done construct path" << std::endl;
        std::vector<int> &best_schedule = proposed_schedule;
        int best_num_of_conflicts = evaluateSchedule(best_schedule, env);
        std::cout << "initial_num_of_conflicts: " << best_num_of_conflicts << std::endl;
        std::unordered_map<int, int> task_agent_map;
        for (int agent_id : SchedulerUtils::global_available_agents)
        {
            task_agent_map[best_schedule[agent_id]] = agent_id;
        }

        // LNSのパラメータ
        const double initial_destroy_ratio = 0.4;
        const double final_destroy_ratio = 0.1;
        int iteration = 0;
        auto start_time = std::chrono::steady_clock::now();

        int max_iterations = 100000000;
        int failed_attempts = 0;
        const int MAX_FAILED_ATTEMPTS = 500;
        // destroyするスケジュールの一時保存用
        // global_free_agentsのサイズでreserve
        std::vector<std::pair<int, int>> destroyed_schedule(SchedulerUtils::global_available_agents.size());
        std::unordered_map<int, std::vector<SchedulerUtils::State>> destroyed_paths;
        while (std::chrono::steady_clock::now() < end_time && iteration < max_iterations && failed_attempts < MAX_FAILED_ATTEMPTS)
        {
            // 破壊する比率を徐々に減少させる
            double progress = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() /
                              std::chrono::duration<double>(end_time - start_time).count();
            double current_destroy_ratio = initial_destroy_ratio +
                                           (final_destroy_ratio - initial_destroy_ratio) * progress;

            // 現在のスケジュールをコピー
            std::vector<int> current_schedule = best_schedule;

            // 破壊
            std::vector<int> destroyed_agents = destroy(current_schedule, current_destroy_ratio);
            for (int agent_id : destroyed_agents)
            {
                destroyed_paths[agent_id] = path[agent_id];
            }

            // 破壊された部分をクリア
            for (int agent_id : destroyed_agents)
            {
                destroyed_schedule.push_back({agent_id, current_schedule[agent_id]});
                task_agent_map.erase(current_schedule[agent_id]);
                current_schedule[agent_id] = -1;
            }

            // 修復
            repair(current_schedule, task_agent_map, destroyed_agents, env);
            for (int agent_id : destroyed_agents)
            {
                path[agent_id] = SchedulerUtils::constructPath(env->curr_states[agent_id].location, env->curr_states[agent_id].orientation, env->task_pool[current_schedule[agent_id]].locations[0]);
            }
            // 評価
            int current_num_of_conflicts = evaluateSchedule(current_schedule, env);

            // より良い解が見つかった場合は更新
            if (current_num_of_conflicts < best_num_of_conflicts)
            {
                best_num_of_conflicts = current_num_of_conflicts;
                best_schedule = current_schedule;
                for (int agent_id : destroyed_agents)
                {
                    task_agent_map[best_schedule[agent_id]] = agent_id;
                }
                std::cout << "Iteration " << iteration << ": New best num_of_conflicts = " << best_num_of_conflicts << std::endl;
                failed_attempts = 0;
            }
            else
            {
                // 破壊したスケジュールを元に戻す
                for (const auto &pair : destroyed_schedule)
                {
                    current_schedule[pair.first] = pair.second;
                    task_agent_map[pair.second] = pair.first;
                    path[pair.first] = destroyed_paths[pair.first];
                }
                failed_attempts++;
            }
            destroyed_schedule.clear();
            destroyed_paths.clear();

            iteration++;
        }
        std::cout << "best_num_of_conflicts: " << best_num_of_conflicts << std::endl;
        std::cout << "iteration: " << iteration << std::endl;
    }

    void schedule_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
    {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time_limit);
        generateInitialSchedule(proposed_schedule, env, end_time);
        optimizeAssignment(proposed_schedule, env, end_time);
        int changed_count = 0;
        for (int agent_id = 0; agent_id < env->num_of_agents; agent_id++)
        {
            if (env->curr_task_schedule[agent_id] != -1 && proposed_schedule[agent_id] != env->curr_task_schedule[agent_id])
                changed_count++;
        }
        std::cout << "changed_count: " << changed_count << std::endl;
    }
}
