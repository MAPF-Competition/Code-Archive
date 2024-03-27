#pragma once
#include <bits/stdc++.h>
using namespace std;
namespace scc_city
{
  const int N = 222600;
  int low[N], depth[N], timer, cyc;
  vector<int> a[N], scc[N];
  stack<int> stck;
  bool vis[N], in[N];
  int sccs[N];

  void dfs(int x, const int *nebors)
  {
    // cout << x << endl;
    low[x] = depth[x] = timer++;
    stck.push(x);
    vis[x] = 1;
    in[x] = 1;
    int nxt;
    // for (int i = 0; i < a[x].size(); ++i)
    // {
    for (int i = 0; i < 4; ++i)
    {
      nxt = nebors[x * 4 + i];
      if (nxt == -1)
        continue;
      if (!vis[nxt])
      {
        dfs(nxt, nebors);
        low[x] = min(low[x], low[nxt]);
      }
      else
      {
        if (in[nxt])
          low[x] = min(low[x], depth[nxt]);
      }
    }
    if (low[x] >= depth[x])
    {
      while (1)
      {
        nxt = stck.top();
        scc[cyc].push_back(nxt);
        sccs[nxt]=cyc;
        stck.pop();
        in[nxt] = 0;
        if (nxt == x)
          break;
      }
      ++cyc;
    }
  }

  int comp_depth[N];
  set<int> comps[N];
  int main_component;
  void cal_depth(int comp, int dp){
    if(vis[comp])return;
    comp_depth[comp] = dp;
    vis[comp] = 1;
    for(auto it:comps[comp]){
      cal_depth(it, dp+1);
    }
  }

  void cal_sccs(int num_of_cells, const int *nebors)
  {
    for (int i = 0; i < num_of_cells; ++i)
    {
      scc[i].clear();
    }
    memset(in, 0, sizeof in);
    memset(vis, 0, sizeof vis);
    memset(sccs, 0, sizeof sccs);
    timer = 0;
    cyc = 0;
    for (int i = 0; i < num_of_cells; ++i){
      if(!vis[i])
        dfs(i, nebors);
    }
    cout << "number of cycles= " << cyc << endl;
    int mx = 0;
    for (int i = 0; i < cyc; ++i)
    {
      if(scc[i].size() > mx){
        mx = scc[i].size();
        main_component = i;
      }
      for (int j = 0; j < scc[i].size(); ++j)
      {
        sccs[scc[i][j]]=i;
        // cout << scc[i][j] << " ";
      }
      // cout << endl;
    }
    for (int i = 0; i < cyc; ++i)
    {
      for (int j = 0; j < scc[i].size(); ++j)
      {
        for (int k = 0; k < 4; ++k)
        {
          int nxt = nebors[scc[i][j] * 4 + k];
          if(nxt==-1)continue;
          if(sccs[nxt] != i){
            comps[i].insert(sccs[nxt]);
            comps[sccs[nxt]].insert(i);
          }
        }
      }
    }
    memset(vis, 0, sizeof vis);
    cal_depth(main_component, 0);
    cout<<"##"<<endl;
    cout<<main_component<<endl;
    cout<<mx<<" "<<scc[main_component].size()<<endl;
    // assert(0);
  }
}

// int main(){
//   int n, m, x, y, nxt, ans;
//   while(cin>>n>>m){
//     for(int i=1; i<=n; ++i){
//       a[i].clear();
//       scc[i-1].clear();
//     }
//     for(int i=0; i<m; ++i){
//       scanf("%d %d", &x, &y);
//       a[x].push_back(y);
//     }
//     memset(in, 0, sizeof in);
//     memset(vis, 0, sizeof vis);
//     memset(sccs, 0, sizeof sccs);
//     timer=0;
//     cyc=0;
//     dfs(1);
//     for(int i=0; i<cyc; ++i){
//       cout<<i<<":::"<<endl;
//        for(int j=0; j<scc[i].size(); ++j){
//         cout<<scc[i][j]<<" ";
//       }
//       cout<<endl;
//     }
//   }
//   return 0;
// }
