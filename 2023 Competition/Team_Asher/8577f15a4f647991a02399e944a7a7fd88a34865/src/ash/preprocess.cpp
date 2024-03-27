#include "ash/preprocess.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <queue>
#include <thread>
#include <tuple>

void ash::Preprocessor::init(const SharedEnvironment& env,
        bool multithreading)
{
    this->env = &env;
    auto start = std::chrono::high_resolution_clock::now();
    location_index.clear();
    location_index.resize(env.map.size(), -1);
    free_locations.clear();
    distances.clear();
    for (int location = 0; location < env.map.size(); ++location)
    {
        if (!env.map[location]) // 0 means there's no obstacle
        {
            location_index[location] = free_locations.size();
            free_locations.push_back(location); 
        }
    }
    fill_cell_properties();

    distances.resize(4*free_locations.size()*free_locations.size(),
                     MAX_DISTANCE);

    if (multithreading)
    {
        int num_of_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> init_threads(num_of_threads);
        int min_task_size = free_locations.size() / num_of_threads;
        int remainder = free_locations.size() % num_of_threads;
        int begin = 0;
        for (int i = 0; i < num_of_threads; ++i)
        {
            int task_size = min_task_size + (i<remainder);
            init_threads[i] = std::thread(
                    [this](int begin, int task_size)
                    {
                        for (int map_idx = begin; map_idx < begin+task_size;
                             ++map_idx)
                        {
                            fill_map(free_locations[map_idx]);
                        }
                    },
                    begin, task_size);
            begin += task_size;
        }
        for (auto& thread : init_threads)
        {
            thread.join();
        }
    }
    else
    {
        for (int map_idx = 0; map_idx < free_locations.size(); ++map_idx)
        {
            fill_map(free_locations[map_idx]);
        }
    }
    std::chrono::duration<double> elapsed_duration =
        std::chrono::high_resolution_clock::now() - start;
    elapsed = elapsed_duration.count();
}

ash::Preprocessor::Distance ash::Preprocessor::get_shortest_distance(
        const AgentPosition& src, int dst) const
{
    return at(get_distance_map(dst), src);
}

double ash::Preprocessor::get_memory_usage() const
{
    size_t size_b = sizeof(Preprocessor);
    size_b += cell_properties.size()*sizeof(CellProperties);
    size_b += free_locations.size()*sizeof(int);
    size_b += location_index.size()*sizeof(int);
    size_b += distances.size()*sizeof(Distance);
    double size_mb = size_b/(1024.0*1024.0);
    return size_mb;
}

void ash::Preprocessor::fill_map(int origin)
{
    Distance* dm = get_distance_map(origin);
    std::queue<AgentPosition> open_set;
    for (auto orientation : ALL_ORIENTATIONS)
    {
        AgentPosition position{origin, orientation};
        at(dm, position) = 0;
        open_set.push(position);
    }
    while (!open_set.empty())
    {
        AgentPosition front = open_set.front();
        open_set.pop();
        for (auto[_,neighbor] : get_reverse_neighbors(*env, front))
        {
            Distance tentative_g = at(dm, front) + 1;
            if (tentative_g < at(dm, neighbor))
            {
                at(dm, neighbor) = tentative_g;
                open_set.push(neighbor);
            }
        } 
    }
}

void ash::Preprocessor::fill_cell_properties()
{
    cell_properties.assign(env->map.size(), {0, INF, false});

    std::queue<Vec2i> depth_openset;
    std::vector<int> deadend_stack;

    for (int loc : free_locations)
    {
        int deg = get_neighbors(*env, loc).size();
        cell_properties[loc].degree = deg;
        if (deg > 2)
        {
            depth_openset.emplace(0, loc);
        }
        else if (deg <= 1)
        {
            deadend_stack.push_back(loc);
        }
    }

    while (!depth_openset.empty())
    {
        auto[d,loc] = depth_openset.front();
        depth_openset.pop();
        if (cell_properties[loc].depth != INF)
        {
            continue;
        }
        cell_properties[loc].depth = d;
        for (auto&&[_, neighbor] : get_neighbors(*env, loc))
        {
            if (cell_properties[neighbor].depth == INF)
            {
                depth_openset.emplace(d+1, neighbor);
            }
        }
    }

    while (!deadend_stack.empty())
    {
        int loc = deadend_stack.back();
        deadend_stack.pop_back();
        if (cell_properties[loc].degree > 2)
        {
            continue;
        }
        cell_properties[loc].is_deadend = true;
        for (auto&&[_, neighbor] : get_neighbors(*env, loc))
        {
            if (!cell_properties[neighbor].is_deadend)
            {
                deadend_stack.push_back(neighbor);
            }
        }
    }
}

