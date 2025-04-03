#ifndef memory_h
#define memory_h

#include <vector>
#include <iostream>

#include "search_node.h"

namespace DefaultPlanner{

    // 每个元素对应地图中的每个位置。每个元素都是一个search node.
class MemoryPool
{
    // every location in the map has a node with id equal to the location
public:
    // constructor
    MemoryPool(){
        size = 0;
        index = 0;
        nodes = nullptr;
        label = 0;
    };

    MemoryPool(int size){
        init(size);
    };

    /**
* @brief: 返回当前生成的节点数量
 * @return int: 返回当前生成的节点数量
 */
    int generated(){
        return index;
    }

    void init(int size){
        this->size = size;
        index = 0;
        label = 0;
        nodes = new s_node[size]; // 动态分配内存
        ready = true;
    }

    /**
* @brief: 检查内存池是否已初始化
* @return bool: 内存池是否已初始化
*/
    bool is_ready(){
        return ready;
    }

    /**
* @brief: 检查节点是否已生成。
* @param int id: 指定地图点。
* @return bool: 指定节点是否已生成。
*/
    bool has_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }

        // 节点的 label 和 id 是否匹配当前内存池。
        return nodes[id].label == label && nodes[id].id == id;
    }

    /**
* @brief: 检查节点是否已标记为关闭。
* @param int id: 指定地图点。
* @return bool: 指定节点是否已标记为关闭。
*/
    bool is_closed(int id){
        // 节点id不能超出地图
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }

        if (nodes[id].label != label){
            return false;
        }

        return nodes[id].is_closed();
    }

    /**
* @brief: 获得某个已生成的地图点。
* @param int id: 指定地图点。
* @return s_node*: 返回某个已生成的地图点。
*/
    s_node* get_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1){
            std::cout << "error node not generated yet" << std::endl;
            exit(1);
        }
        return &(nodes[id]);
    }

    /**
* @brief: 关闭某个地图节点
* @param int id: 指定地图点。
 * @return void
*/
    void close_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1){
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        nodes[id].close();
    }

    /**
     * @brief: 生成指定位置的地图节点。
     * @param int id: 指定地图点。
     * @param int g: 指定地图点和起点的距离。
     * @param int h: 指定地图点和终点的启发式距离。
     * @param int op_flow: 对向流量
     * @param int depth: 搜索深度??
     * @param int all_vertex_flow: 所有节点的流量之和??
     * @return s_node*: 返回生成指定位置的地图点。
     * comment: g, h, op_flow, all_vertex_flow是在外面计算的, 这里只赋值
     */
    s_node* generate_node(int id, int g, int h, int op_flow, int depth, int all_vertex_flow = 0){
        if (id >= size) // 不能超越地图边界
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }

        // 不能生成已经生成的节点
        if (nodes[id].label == label && nodes[id].id != -1){
            std::cout << "node already generated " << id << ","<< is_ready()<< std::endl;

            std::cout << "node already generated " << nodes[id].id<< std::endl;
            exit(1);
        }

        nodes[id].reset(); // 重置节点
        nodes[id].label = label; // 更新节点标签
        nodes[id].id = id; // 设置节点 ID
        nodes[id].g = g; // 设置路径代价 g
        nodes[id].h = h; // 设置启发式代价 h
        nodes[id].op_flow = op_flow; // 设置反向流量
        nodes[id].depth = depth; // 设置深度
        nodes[id].all_vertex_flow = all_vertex_flow;
        index++;
        return &(nodes[id]); // 返回生成的节点
    }

    /**
* @brief: 释放某个地图节点
* @param int id: 指定地图点。
* @return void
*/
    void free_node(int id){
        // 不能超出地图范围之外
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }

        // 不能释放本来就空的节点
        if (nodes[id].id == -1){
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }

        nodes[id].reset();
        index--;
    }

    /**
    * @brief: 重置内存池，并通过增加 label 区分新旧节点。
    * @param int id: 指定地图点。
    * @return void
    */
    void reset(){
        index = 0;
        label++;

    }

    // destructor
    ~MemoryPool(){
        if (nodes != nullptr){
            delete [] nodes; // 释放动态分配的内存
            nodes = nullptr;
        }
    }

private:
    s_node* nodes; // 节点数组，存储所有地图节点
    int size; // 内存池的总大小, 也就是地图大小
    int index;
    int label;
    bool ready = false;
};


}
#endif