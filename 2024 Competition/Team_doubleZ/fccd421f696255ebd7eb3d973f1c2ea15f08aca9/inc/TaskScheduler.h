#pragma once
#include "Tasks.h"
#include "SharedEnv.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <random>
using namespace std;

class TaskScheduler
{
public:
    SharedEnvironment *env;

    TaskScheduler(SharedEnvironment *env) : env(env) {};
    TaskScheduler() { env = new SharedEnvironment(); };
    virtual ~TaskScheduler() { delete env; };
    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);

    void my_plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env);
    int cul_score(int robot_id, int task_id, SharedEnvironment *env, int cur_task_dis);
    int get_est_dist(int loc, int c_loc, SharedEnvironment *env);
    void get_anchor(SharedEnvironment *env);
    void get_sector(SharedEnvironment *env);
    void get_sector_tasks(int sector_idx, SharedEnvironment *env);
    std::vector<int> loc_sector, sector_anchor_list;
    std::vector<int> all_task_ids, robot_ids, task_ids;
    int anchor_loc_1, anchor_loc_2, anchor_loc_3, anchor_loc_4, time_step;
    int robot_size_limit{200};
    int task_size_limit{400};
};

class KuhnMunkres
{
private:
    vector<vector<int>> graph; // 权重矩阵
    vector<int> matchX;        // X部集合对应的匹配点
    vector<int> matchY;        // Y部集合对应的匹配点
    vector<int> lx;            // X部点的顶标
    vector<int> ly;            // Y部点的顶标
    vector<bool> visitX;       // X部点是否访问
    vector<bool> visitY;       // Y部点是否访问
    int n, m;                  // n为X部点数,m为Y部点数
    const int INF = 0x3f3f3f3f;

    bool dfs(int x)
    {
        visitX[x] = true;
        for (int y = 0; y < m; y++)
        {
            if (visitY[y])
                continue;
            int gap = lx[x] + ly[y] - graph[x][y];
            if (gap == 0)
            {
                visitY[y] = true;
                if (matchY[y] == -1 || dfs(matchY[y]))
                {
                    matchY[y] = x;
                    matchX[x] = y;
                    return true;
                }
            }
        }
        return false;
    }

public:
    KuhnMunkres(int n, int m) : n(n), m(m)
    {
        graph.resize(n, vector<int>(m));
        matchX.resize(n, -1);
        matchY.resize(m, -1);
        lx.resize(n, -INF);
        ly.resize(m, 0);
        visitX.resize(n);
        visitY.resize(m);
    }

    void setEdge(int x, int y, int w)
    {
        graph[x][y] = w;
        lx[x] = max(lx[x], w); // 初始化X部点的顶标为与其相连边中的最大权值
    }

    long long solve()
    {
        for (int x = 0; x < n; x++)
        {
            while (true)
            {
                fill(visitX.begin(), visitX.end(), false);
                fill(visitY.begin(), visitY.end(), false);
                if (dfs(x))
                    break;

                // 如果没找到增广路，则需要修改顶标
                int delta = INF;
                for (int i = 0; i < n; i++)
                {
                    if (!visitX[i])
                        continue;
                    for (int j = 0; j < m; j++)
                    {
                        if (visitY[j])
                            continue;
                        delta = min(delta, lx[i] + ly[j] - graph[i][j]);
                    }
                }

                for (int i = 0; i < n; i++)
                {
                    if (visitX[i])
                        lx[i] -= delta;
                }
                for (int i = 0; i < m; i++)
                {
                    if (visitY[i])
                        ly[i] += delta;
                }
            }
        }

        // 计算最大权值和
        long long res = 0;
        for (int i = 0; i < n; i++)
        {
            if (matchX[i] != -1)
            {
                res += graph[i][matchX[i]];
            }
        }
        return res;
    }
    vector<int> get_result()
    {
        return matchX;
    }
};