
#include <random>
#include <vector>
#include <unordered_set>

#include "r_env.hpp"
#include "r_timer.hpp"



namespace RHCR_Planner{

    //default planner data
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<int> dummy_goals;
    std::mt19937 mt1;
    
    // 定义变量 -hzj
    std::vector<PlanResult<State2, Action, float>> previous_solution;
    std::unordered_set<size_t> activeAgents;
    
    // our data
    // 命名空间成员变量
    int preprocess_time_limit;
    int time_limit;

    // 障碍物
    std::unordered_set<std::pair<int,int>,PairHash> obs_2d;
    // 当前目标
    std::vector<std::pair<int, int>> curr_goals;
    // 2d地图
    std::vector<std::vector<int>> map_2d;

    std::vector<State2> prev_states2;
    std::vector<State2> next_states2;

    //  -hzj
    std::unordered_set<size_t> calculateActiveAgents(
        const std::vector<State2>& initialStates,
        const std::vector<std::pair<int,int>>& curr_goals,
        const size_t maxActiveAgents) {
        
        std::priority_queue<AgentPriority> pq;
        activeAgents.clear();  // 清空旧数据
        
        for (size_t i = 0; i < initialStates.size(); ++i) {
            // int distance = std::abs(initialStates[i].loc.first - env.get_goal(i).first) +
            //              std::abs(initialStates[i].loc.second - env.get_goal(i).second);
            int distance = std::abs(initialStates[i].loc.first - curr_goals[i].first) +
                         std::abs(initialStates[i].loc.second - curr_goals[i].second);
            
            AgentPriority ap{i, distance};
            
            if (pq.size() < maxActiveAgents) {
                pq.push(ap);
                activeAgents.insert(i);
            } else if (ap < pq.top()) {
                activeAgents.erase(pq.top().index);
                pq.pop();
                pq.push(ap);
                activeAgents.insert(i);
            }
        }
        
        return activeAgents;
    }

    void initialize(int preprocess_time_limit, SharedEnvironment* env)
    {
        Timer t0;
        ids.resize(env->num_of_agents);

        // 初始化previous_solution  -hzj
        previous_solution.clear();
        previous_solution.resize(env->num_of_agents);

        for (int i = 0; i < ids.size();i++){
            ids[i] = i;
            // std::cout << "--- RHCR agent id : " << ids[i] << std::endl;
        }
        std::cout << "--- RHCR map_name : " << env->map_name << std::endl;
        std::cout << "--- RHCR num_of_agents : " << env->num_of_agents << std::endl;
        std::cout << "--- RHCR goal_locations size : " << env->goal_locations.size() << std::endl;
        std::cout << "--- RHCR current states size : " << env->curr_states.size() << std::endl;

        std::cout << "env->rows = " << env->rows << ", env->cols = " << env->cols << std::endl; 

        map_to_2d(env->map, env->rows, env->cols, map_2d, obs_2d);
        t0.stop();
        std::cout << "--- RHCR Initial Function time : " << t0.elapsedSeconds() << std::endl;

        return;
    };
    void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env)
    {
        Timer t4;

        // 信息更新
        TimePoint start_time = std::chrono::steady_clock::now();
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - 100); 

        std::unordered_set<Location> obstacles;

        std::vector<Location> goals;
        goals.resize(env->num_of_agents);
        std::vector<State2> startStates;
        startStates.resize(env->num_of_agents);

        prev_states2.resize(env->num_of_agents);
        next_states2.resize(env->num_of_agents);
        curr_goals.resize(env->num_of_agents);

        // 将obs_2d中的障碍物位置转换为Location类型，并添加到obstacles集合中。
        for (const auto& o : obs_2d) {
            obstacles.insert(Location(o.first, o.second));
        }

        // 如果当前时间步为0（可能是初始化阶段），则为每个智能体分配一个虚拟目标位置，将其初始化为当前状态的位置。
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
                goals[i] = Location(to_2d(dummy_goals.at(i),env).first, to_2d(dummy_goals.at(i),env).second);
            }
        }

        //创建活跃智能体容器lhy
        // std::vector<size_t> active_agents;
        // active_agents.reserve(env->num_of_agents);
        std::vector<int> only_task_ids;
        int agent_with_task = 0;

        for (int i=0; i<env->num_of_agents; i++)
        {
            next_states2[i] = State2();
            prev_states2[i] = State2(to_2d(env->curr_states[i].location,env),0,env->curr_states[i].orientation);

            startStates[i] = prev_states2[i];
            // 任务获取
            // 如果智能体的目标位置列表为空，则将其任务设置为虚拟目标；
            // 否则，将其任务设置为目标位置列表的第一个元素。
            if (env->goal_locations[i].empty()){
                curr_goals[i] = to_2d(dummy_goals.at(i),env); //!!!!
                goals[i] = Location(curr_goals[i].first, curr_goals[i].second);
            }
            else{
                //将需要移动的智能体加入活跃列表lhy
                // active_agents.push_back(i); 
                curr_goals[i] = to_2d(env->goal_locations[i].front().first,env);
                goals[i].x  = curr_goals[i].first;
                goals[i].y = curr_goals[i].second;
                only_task_ids.push_back(i);
                agent_with_task++;
            }
        }
        bool disappearAtGoal = false;
        float w = 1.5;

        size_t dimy = map_2d.size();    // map_2d的行数
        size_t dimx = map_2d[0].size(); // 列数

        // std::cout << "dimx = " << dimx << ", dimy = " << dimy << std::endl; 
        

        Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal);
        
        if(env->num_of_agents >= 3500){
            // 计算活跃智能体 -hzj
        activeAgents = calculateActiveAgents(startStates, curr_goals);

        // -hzj
        mapf.setTemporaryObstacles(startStates, activeAgents);
        
        }

        ECBS<State2, Action, float, r_Conflict, Constraints, Environment> ecbs(mapf, w);
        
        
        std::vector<PlanResult<State2, Action, float> > solution = previous_solution;

        // std::cout << "RHCR CHECK before ecbs search " << std::endl;
        t4.stop();
        // std::cout << "****** RHCR Time Before ECBS Search : " << t4.elapsedSeconds() << std::endl;
        Timer timer;
        bool success;
        if(env->num_of_agents >= 3500){
            success = ecbs.search1(startStates, solution);}
        else {
            success = ecbs.search2(startStates, solution, only_task_ids);
        }
        timer.stop();
        std::cout << "****** RHCR Time For ECBS Search : " << timer.elapsedSeconds() <<std::endl;

        //处理非活跃智能体的等待动作lhy
        // for (int i = 0; i < env->num_of_agents; ++i) {
        //     // 只更新活跃智能体的状态
        //     if (std::find(active_agents.begin(), active_agents.end(), i) == active_agents.end()) {
        //         // 非活跃智能体保持原位
        //         next_states2[i] = prev_states2[i];
        //         actions[i] = Action::W;
        //         continue;
        //     }

        //     // 原有活跃智能体处理逻辑
        //     // if (solution[i].states.size() == 1) {
        //     //     next_states2[i] = prev_states2[i];
        //     // } else {
        //     //     next_states2[i] = solution[i].states[1].first;
        //     // }
        //     // actions[i] = r_getAction(prev_states2[i], next_states2[i]);
        // }

        if (success) {
            // -hzj
            for (size_t i = 0; i < solution.size(); ++i) {
                if (solution[i].states.size() >= 2) {
                    
                    // 更新states（从第二个状态开始） --hzj
                previous_solution[i].states.clear();
                for (size_t j = 1; j < solution[i].states.size(); ++j) {
                    State2 newState = solution[i].states[j].first;
                    newState.timestep = j - 1;  // 重置时间步，从0开始
                    previous_solution[i].states.push_back(
                        std::make_pair(newState, solution[i].states[j].second - 1)
                    );
                }
                                    
                // 更新actions（从第二个动作开始）-hzj
                previous_solution[i].actions.assign(
                    solution[i].actions.begin() + 1,
                    solution[i].actions.end()
                );
    
                // 更新总代价 -hzj
                previous_solution[i].cost = solution[i].cost-solution[i].states[0].second;
                previous_solution[i].fmin = solution[i].fmin-1;

                } else {
                    // 如果路径太短，清空存储
                    previous_solution[i].states.clear();
                    previous_solution[i].actions.clear();
                    previous_solution[i].cost = 0;
                    previous_solution[i].fmin = 0;
                }
            }

            size_t SystemTime = env->curr_timestep;
            std::cout << "****** "<< SystemTime <<" Step Planning successful! Use Time: " << timer.elapsedSeconds()<<" ******" << std::endl;
            int cost = 0;
            int makespan = 0;
            for (const auto& s : solution) {
            cost += s.cost;
            makespan = std::max<int>(makespan, s.cost);
            }

        } else {
            std::cout << "Planning NOT successful!" << std::endl;
        }        

        actions.resize(env->num_of_agents);
        // 更新next_states2
        for (int i = 0; i < ids.size(); i++)
        {
            if (env->goal_locations[i].empty() ||  solution[i].states.size() == 1)
            {
                next_states2[i] = prev_states2[i];
                //std::cout << "env->goal_locations "<< i <<" EMPTY" << std::endl;
            } else {
                next_states2[i] = solution[i].states[1].first;
            }

            
            actions.at(i) = r_getAction(prev_states2[i],next_states2[i]);
        }
        return;
    };

    void map_to_2d(std::vector<int> map, int rows, int cols, std::vector<std::vector<int>> &map_2d, 
    std::unordered_set<std::pair<int, int>,PairHash> &obs_2d) 
    {
        // std::cout << "OBS IN MAP : " << std::endl;
        map_2d.resize(rows, std::vector<int>(cols, 0));
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int index = i * cols + j; 
                map_2d[i][j] = map[index]; 

                if (map[index] == 1) {
                    obs_2d.insert({j, i});
                }
            }
        }
        std::cout << std::endl;
    }

    std::pair<int,int> to_2d(int location,SharedEnvironment* env)
    {
        std::pair<int,int> loc;
        loc.second = location / env->cols; // 和DefaultPlanner比，好像坐标是反的，调换一下试试
        loc.first = location % env->cols;
        return loc;
    }

    Action r_getAction(State2& prev_states2, State2& next_states2)
    {
        if (prev_states2.loc == next_states2.loc && prev_states2.orientation == next_states2.orientation)
        {
            return Action::W;
        }
        if (prev_states2.loc != next_states2.loc && prev_states2.orientation == next_states2.orientation)
        {
            return Action::FW;
        }
        if (next_states2.orientation == (prev_states2.orientation + 1) % 4)
        {
            return Action::CR;
        }
        if (next_states2.orientation == (prev_states2.orientation + 3) % 4)
        {
            return Action::CCR;
        }
        return Action::W;
    }

}
