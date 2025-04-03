//
// Created by take_ on 25-1-10.
//



#include "usage.h"



/**
 * @brief: 根据两个地图点之间的 索引差值 (diff)，计算并返回两点之间的 方向编号
 * @param int diff: 两个地图点之间的 索引差值 (diff)
 * @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
 * @return int: 0: 右。1: 下。2: 左。3: 上
 */
int get_d(int diff, const SharedEnvironment* env)  {

    /*
    0: 向西（相邻格子在同一行，且索引差值为 +1）。
    1: 向南（相邻格子在下一行，且索引差值为 +cols，cols 是地图的列数）。
    2: 向东（相邻格子在同一行，且索引差值为 -1）。
    3: 向北（相邻格子在上一行，且索引差值为 -cols）
     */
    return (diff == 1)? 0: (diff == -1)? 2: (diff == env->cols)? 1: 3;

}

/**
* @brief: 验证从位置 loc 到位置 loc2 的移动是否有效。
* @param int loc: 移动起点
* @param int loc2: 移动终点
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @return bool: true valid; false invalid.
*/
bool validateMove(int loc, int loc2, const SharedEnvironment* env)
{
    // 如果超出地图边界, 无效
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;
    if (loc_x < 0 || loc_y < 0 || loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    // 如果超出地图边界, 无效
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (loc2_x < 0 || loc2_y < 0 ||loc2_x >= env->rows || loc2_y >= env->cols || env->map[loc2] == 1)
        return false;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1) // 如果移动大于1格, 无效
        return false;
    return true;

}

/**
* @brief: 计算从位置 loc 到位置 loc2 的曼哈顿距离。
* @param int loc: 起点
* @param int loc2: 终点
* @param SharedEnvironment* env: 提供环境的共享信息，例如地图尺寸、地图位置等。
* @return int: 从位置 loc 到位置 loc2 的曼哈顿距离。
*/
int manhattanDistance(int loc, int loc2,const SharedEnvironment* env){
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc_x-loc2_x) + abs(loc_y-loc2_y);
}

void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
    neighbors.clear();
    //forward
    assert(location >= 0 && location < env->map.size());
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    assert(forward!=location);

#ifndef NDEBUG
    std::cout<<"forward: "<<forward<<std::endl;
#endif
    if (validateMove(location, forward, env)	){
#ifndef NDEBUG
        std::cout<<"forward yes"<<std::endl;
#endif
        neighbors.emplace_back(std::make_pair(forward,new_direction));
    }
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    neighbors.emplace_back(std::make_pair(location,direction)); //wait
}

void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
    neighbors.clear();
    //forward
    assert(location >= 0 && location < env->map.size());
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (validateMove(location, forward, env)){
        assert(forward >= 0 && forward < env->map.size());
        neighbors.emplace_back(std::make_pair(forward,new_direction));
    }
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
}

/**
* @brief: 计算指定位置的邻居节点。
* @param const Neighbors* ns: global_neighbors
* @param std::vector<int>& neighbors: 邻居节点保存处
* @param int location: 对象位置。
* @return void。
*/
void getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) {
    neighbors.clear();
    // forward
    assert(location >= 0 && location < ns->size()); // 不能超出地图边界
    neighbors = ns->at(location); // 从 ns 中获取位置 location 的邻居节点列表，并将其赋值到 neighbors 中。
}

/**
* @brief: 计算指定位置的邻居节点。
* @param const Neighbors* ns: global_neighbors
* @param int neighbors[]: 邻居节点保存处
* @param int location: 对象位置。
* @return void。
*/
void getNeighborLocs(const Neighbors* ns, int neighbors[], int location) {
    //forward
    int size = 4;
    assert(location >= 0 && location < ns->size());

    // 已有的邻居填进去, 没有的邻居以-1的形式挂在末尾
    for (int i = 0; i < size; i++) {
        if (i < ns->at(location).size()){
            neighbors[i] = ns->at(location)[i];
        }
        else
            neighbors[i] = -1;
    }

}


