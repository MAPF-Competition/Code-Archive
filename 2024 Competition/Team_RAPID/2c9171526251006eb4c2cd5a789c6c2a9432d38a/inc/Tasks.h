#pragma once
#include "common.h"

struct Task
{
    int task_id;
    int t_completed = -1;
    int t_revealed = -1;
    int agent_assigned = -1;

    vector<int> locations;  //一个整数向量，存储任务所需访问的各个地点。任务可能需要依次访问多个位置，这个列表记录了它们的顺序。
    int idx_next_loc = 0;   //当前任务中下一个未完成的目标位置的索引。




    int get_next_loc()
    {
        if (idx_next_loc < locations.size())
        {
            return locations.at(idx_next_loc);
        } 
        else 
        {
            assert(false);
            return -1;
        }
    }

    bool is_finished()
    {
        return idx_next_loc == locations.size();
    }

    //Task(int task_id, int location): task_id(task_id), locations({location}) {};
    Task(int task_id, list<int> location, int t_revealed): task_id(task_id), t_revealed(t_revealed)
    {
        for (auto loc: location)
            locations.push_back(loc);
    };

    Task(){};

    Task(Task* other)
    {
        task_id = other->task_id;
        t_completed = other->t_completed;
        locations = other->locations;
        t_revealed = other->t_revealed;
        idx_next_loc = other->idx_next_loc;
        agent_assigned = other->agent_assigned;
    };

    Task(const Task& other)
    {
        task_id = other.task_id;
        t_completed = other.t_completed;
        locations = other.locations;
        t_revealed = other.t_revealed;
        idx_next_loc = other.idx_next_loc;
        agent_assigned = other.agent_assigned;
    };
};
