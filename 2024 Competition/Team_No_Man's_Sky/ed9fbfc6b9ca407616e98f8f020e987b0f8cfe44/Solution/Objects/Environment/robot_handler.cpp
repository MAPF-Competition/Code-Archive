#include <Objects/Environment/robot_handler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/info.hpp>
#include <Objects/Environment/raw_heuristic_matrix.hpp>

bool Robot::is_disable() const {
    ASSERT((priority == -1) == !target, "invalid disable");
    return priority == -1;
}

RobotsHandler::RobotsHandler(uint32_t agents_num) : robots(agents_num) {
}

void RobotsHandler::update(const SharedEnvironment &env) {
    ASSERT(env.curr_states.size() == robots.size(), "invalid sizes");
    for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(r < env.curr_states.size(), "invalid r");

        Position pos(env.curr_states[r].location + 1, env.curr_states[r].orientation);
        ASSERT(pos.is_valid(), "invalid position");
        uint32_t node = get_graph().get_node(pos);

        robots[r].node = node;
        robots[r].target = 0;
        robots[r].priority = -1;

        int task_id = env.curr_task_schedule[r];
        if (!env.task_pool.count(task_id)) {
            continue;
        }

        ASSERT(task_id != -1, "invalid task");

        auto &task = env.task_pool.at(task_id);
        uint32_t target = task.locations[task.idx_next_loc] + 1;

        ASSERT(0 <= target && target < get_map().get_size(), "invalid target");
        ASSERT(Position(target, 0).is_valid(), "invalid");
        ASSERT(Position(target, 1).is_valid(), "invalid");
        ASSERT(Position(target, 2).is_valid(), "invalid");
        ASSERT(Position(target, 3).is_valid(), "invalid");

        uint32_t task_dist = 0;
        {
            uint32_t node = robots[r].node;
            for (int i = task.idx_next_loc; i < task.locations.size(); i++) {
                int to_pos = task.locations[i] + 1;
                task_dist += get_hm().get(node, to_pos);
                node = get_graph().get_node(Position(to_pos, 0));
            }
        }
        // TODO:
        // if task_dist + env.curr_timestep > get_test_info().steps_num
        // then this robot is free
        robots[r].target = target;
        robots[r].priority = task_dist;
    }

    // влияет только на PIBTS
#ifdef DISABLE_LATE_AGENTS
    {
        for (uint32_t r = 0; r < robots.size(); r++) {
            if (!robots[r].target) {
                continue;
            }
            // есть задача

            const auto &task = env.task_pool.at(env.curr_task_schedule[r]);
            uint32_t task_dist = 0;
            {
                uint32_t node = robots[r].node;
                for (int i = task.idx_next_loc; i < task.locations.size(); i++) {
                    int to_pos = task.locations[i] + 1;
                    task_dist += get_rhm().get(node, to_pos);
                    node = get_graph().get_node(Position(to_pos, 0));
                }
            }

            // этот агент не успевает выполнить задачу, ну пусть тогда другим не мешает
            if (env.curr_timestep + task_dist > get_test_info().steps_num + 5) {
                // disable
                robots[r].priority = -1;
                robots[r].target = 0;
            }
        }
    }
#endif

    // влияет только на PIBTS
#ifdef DISABLE_AGENTS
    uint32_t max_task_assigned = get_test_info().max_task_assigned;
    if (max_task_assigned < robots.size()) {
        std::vector<std::pair<double, uint32_t>> ids;
        for (uint32_t r = 0; r < robots.size(); r++) {
            ids.emplace_back(robots[r].priority, r);
        }
        std::sort(ids.begin(), ids.end());

        for (auto [metric, r]: ids) {
            // не инициализирован
            if (!robots[r].target) {
                continue;
            }
            // есть задача
            if (max_task_assigned > 0) {
                // enable
                max_task_assigned--;
            } else {
                // disable
                robots[r].priority = -1;
                robots[r].target = 0;
            }
        }
    }
#endif
}

const Robot &RobotsHandler::get_robot(uint32_t r) const {
    ASSERT(r < robots.size(), "invalid r");
    return robots[r];
}

const std::vector<Robot> &RobotsHandler::get_robots() const {
    return robots;
}

uint32_t RobotsHandler::size() const {
    return robots.size();
}

RobotsHandler &get_robots_handler() {
    static RobotsHandler rh;
    return rh;
}
