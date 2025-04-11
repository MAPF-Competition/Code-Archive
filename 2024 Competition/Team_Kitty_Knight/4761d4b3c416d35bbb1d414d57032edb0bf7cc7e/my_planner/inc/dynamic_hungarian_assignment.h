#ifndef DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP
#define DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP

#include <queue>
#include <set>
#include <cmath>
#include <vector>
#include "common.h"

const int INF7f = 0x7fffff;

class DynamicHungarianAssignment {

public:
    int org_n, org_m, n;
    vector<vector<long> > W;
    vector<int> mateL, mateR, p;
    vector<long> lx, ly, slack;
    vector<bool> m;
    long inf = std::numeric_limits<long>::max();
    long DHinf = 1e9;

    DynamicHungarianAssignment(){}

    void initHungarian(int agent_num, int goal_num)
    {
        org_n = agent_num;
        org_m = goal_num;
        n = std::max(org_m, org_n);
        W = vector<vector<long> >(org_n, vector<long>(org_m, 0));
        mateL = vector<int>(n, -1);
        mateR = vector<int>(n, -1);
        p = vector<int>(n);
        lx = vector<long>(n, -inf);
        ly = vector<long>(n);
        slack = vector<long>(n);
        m = vector<bool>(n);
    }

    void addEdge(int u, int v, int w) {
        if (u >= org_n) return ;
        W[u][v] = DHinf - w;
    }

    void augment(int j) {
        int i, next;
        do {
            i = p[j];
            mateR[j] = i;
            next = mateL[i];
            mateL[i] = j;
            if (next != -1) j = next;
        } while (next != -1);
    }

    int64_t solve_hungarian() {
        int vrex = 0;
        for (int i = 0; i < n; i++) if (mateL[i] == -1) vrex++;
        while (vrex > 0) {
            for (int i = 0; i < n; i++) {
                m[i] = false;
                p[i] = -1;
                slack[i] = inf;
            }
            const int MAXV = n;
            int numQ = 0;
            bool aug = false;
            std::vector<bool> Q(MAXV, false);
            for (int i = 0; i < n; i++)
                if (mateL[i] == -1) {
                    Q[i] = true;
                    numQ++;
                }

            do {
                int i, j;
                for (int k = 0; k < n; k++)
                    if (Q[k]) {
                        i = k;
                        numQ--;
                        Q[i] = false;
                        m[i] = true;
                        j = 0;
                        break;
                    }

                while (aug == false && j < n) {
                    if (mateL[i] != j) {
                        int tmp_w;
                        if (i >= org_n) tmp_w = 0;
                            else tmp_w = W[i][j];
                        if (lx[i] + ly[j] - tmp_w < slack[j]) {
                            slack[j] = lx[i] + ly[j] - tmp_w;
                            p[j] = i;
                            if (slack[j] == 0) {
                                if (mateR[j] == -1) {
                                    augment(j);
                                    aug = true;
                                    vrex--;
                                } else {
                                    if (Q[mateR[j]] == false) {
                                        Q[mateR[j]] = true;
                                        numQ++;
                                    }
                                }
                            }
                        }
                    }
                    j++;
                }

                if (aug == false && numQ == 0) {
                    long tao = inf;
                    for (int k = 0; k < n; k++)
                        if (slack[k] > 0)
                            tao = std::min(tao, slack[k]);
                    for (int k = 0; k < n; k++)
                        if (m[k])
                            lx[k] -= tao;

                    int x = -1;
                    bool X[MAXV];
                    for (int k = 0; k < n; k++)
                        if (slack[k] == 0)
                            ly[k] += tao;
                        else {
                            slack[k] -= tao;
                            if (slack[k] == 0 && mateR[k] == -1) x = k;
                            if (slack[k] == 0) X[k] = true;
                            else X[k] = false;
                        }

                    if (x == -1) {
                        for (int k = 0; k < n; k++)
                            if (X[k]) {
                                Q[mateR[k]] = true;
                                numQ++;
                            }
                    } else {
                        augment(x);
                        aug = true;
                        vrex--;
                    }
                }
            } while (aug == false);
        }

        int64_t ans = 0;
        for (int i = 0; i < org_n; i++) {
            ans += DHinf - W[i][mateL[i]];
        }
        return ans;
    }

    void create_cost_matrix(const vector<vector<int > >& fmin_matrix, int u = -1)
    {
        int n = fmin_matrix.size();
        int m = fmin_matrix[0].size();
        if (u==-1) {
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++) {
                    if (fmin_matrix[i][j] == INF7f) {
                        addEdge(i, j, DHinf);
                    } else {
                        addEdge(i, j, fmin_matrix[i][j]);
                    }
                }
        }
        else {
            for (int j=0;j<m; j++)
            {
                if (fmin_matrix[u][j] == INF7f) addEdge(u, j, DHinf);
                else addEdge(u, j, fmin_matrix[u][j]);
            }
        }
    }


    int64_t firstSolution(const vector<vector<int> >& cost_matrix,
                          unordered_map<int, int>& out_TA_solution)
    {
        create_cost_matrix(cost_matrix);
        for(int i=0;i<n;i++)
        {
            for(int j=0;j<n;j++)
            {
                long tmp_w;
                if (i >= org_n) tmp_w = 0;
                    else tmp_w = W[i][j];
                lx[i] = std::max(lx[i], tmp_w);
            }
            ly[i] = 0;
        }
        int64_t ans = solve_hungarian();
        for (int i = 0; i < org_n; i++) {
            out_TA_solution[i] = mateL[i];
        }
        return ans;
    }

    int64_t incrementalSolutionX(const vector<vector<int > >& fmin_matrix,
                                 unordered_map<int, int>& out_TA_solution,
                                 const int & u)
    {
        create_cost_matrix(fmin_matrix, u);
        mateR[ mateL[u] ] = -1; mateL[u] = -1;
        lx[u] = -inf;
        for(int i=0;i<n;i++)
        {
            long tmp_w;
            if (u >= org_n) tmp_w = 0;
                else tmp_w = W[u][i];
            lx[u] = std::max(lx[u], tmp_w-ly[i]);
        }
        int64_t ans = solve_hungarian();
        for (int i = 0; i < org_n; i++) {
            int task_idx = mateL[i];
            out_TA_solution[i] = task_idx;
        }
        return ans;
    }

};



//
#endif //DYNAMIC_HUNGARIAN_ASSIGNMENT_HPP