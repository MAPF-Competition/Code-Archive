#ifndef SEARCH_NODE_H
#define SEARCH_NODE_H

#include <vector>
#include <iostream>

namespace DefaultPlanner{
struct s_node
{
    int label = 0; // 节点的标识，用于分类或分组
    int id = -1; // also location, -1 indicated not generated yet.
    int g = 0; // 从起点到当前节点的路径代价
    int h = 0; // 启发式代价（当前节点到目标的估计代价）
    int op_flow = 0; // 当前节点的对向流量乘积之和
    int all_vertex_flow = 0; // 所有顶点的流量之和
    bool closed = false; // 节点是否已关闭
    int depth = 0; // 当前节点的深度
    double tie_breaker = 0;
    s_node* parent = nullptr;

    unsigned int priority; // 节点的优先级，用于排序或队列操作

    // constructor
    s_node(int id, int g, int h, int op_flow, int depth) : id(id), g(g), h(h), op_flow(op_flow),depth(depth) {};
    s_node() = default;

    /**
* @brief: 返回 f=g+h的值，用于路径规划中的优先级比较。
* @return int: 返回 f=g+h的值
*/
    int get_f() const { return g + h; }

    /**
* @brief: 检查节点是否已关闭。
* @return bool: 返回 节点是否已关闭。
*/
    bool is_closed() const { return closed; }

    /**
* @brief: 关闭节点
* @return void
*/
    void close() { closed = true; }

    /**
* @brief: 取得对向流量
* @return int: 返回对向流量
     * comment: 给外部函数访问接口。计算流量不在这里。
*/
    int get_op_flow() const { return op_flow; }

    /**
* @brief: 取得所有顶点的流量之和
* @return int: 返回所有顶点的流量之和
 * comment: 给外部函数访问接口。计算流量不在这里。
*/
    int get_all_vertex_flow() const { return all_vertex_flow; }

    /**
* @brief: 设置顶点流量和对向流量
* @return void
* comment: 给外部函数访问接口。计算流量不在这里。
*/
    void set_all_flow(int op_flow,  int all_vertex_flow){
        this->op_flow = op_flow;
        this->all_vertex_flow = all_vertex_flow;
    };

    /**
* @brief: 获取g值
* @return int: 返回g值
* comment: 给外部函数访问接口。计算g不在这里。
*/
    int get_g() const { return g; }

    /**
* @brief: 获取h值
* @return int: 返回h值
* comment: 给外部函数访问接口。计算h不在这里。
*/
    int get_h() const { return h; }

    /**
* @brief: 获取priority值
* @return int: 返回priority值
* comment: 给外部函数访问接口。计算priority不在这里。
*/
    unsigned int get_priority() const { return priority; }

    /**
* @brief: 更新节点的优先级。
     * @param unsigned int p: 新的优先级值。
* @return void
*/
    void set_priority(unsigned int p) { priority = p; }

    /**
* @brief: 将节点恢复为初始状态，便于重复利用。
* @return void
*/
    void reset(){
        label = 0;
        id = -1;
        g = 0;
        h = 0;
        op_flow = 0;
        all_vertex_flow = 0;
        closed = false;
        depth = 0;
        parent = nullptr;
        tie_breaker = 0;

    }
    /* data */
};

// 判断两个节点是否相等。
struct equal_search_node
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        return lhs.id == rhs.id && lhs.get_op_flow() == rhs.get_op_flow() && lhs.get_all_vertex_flow() == rhs.get_all_vertex_flow() && lhs.get_g() == rhs.get_g();
    }
};

// 比较两个节点的 f 值，用于优先队列排序。
struct re_f{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        return lhs.get_f() < rhs.get_f();
    }
};

// 先比较节点的平局优先级（tie_breaker），若相等则比较 f(n) 值。
struct re_jam{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        if (lhs.tie_breaker  == rhs.tie_breaker)
        {
                return lhs.get_f() < rhs.get_f();
        }
        else
            return lhs.tie_breaker < rhs.tie_breaker;
    }
};

// 处理节点的比较，考虑 f、g 值，并在完全相等时随机选择。
struct cmp_less_f // focal search open
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

        if (lhs.get_f() == rhs.get_f()){
                if (lhs.get_g() == rhs.get_g())
                    return rand() % 2;
                else
                    return lhs.get_g() > rhs.get_g();
        }
        else
            return lhs.get_f() < rhs.get_f();

    }
};

// 扩展 f(n) 值的比较规则，将流量指标纳入优先级计算。
struct re_of{ //re-expansion rule astar
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

        return lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() < rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f();

    }
};

// 首先比较f+顶点流量+对象流量, 平局则比较f, 再平局则比较g, 再平局则随机打破平局
struct cmp_less_of //astar open 
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

            if (lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() == rhs.get_op_flow() + rhs.get_all_vertex_flow() + rhs.get_f())
            {
                if (lhs.get_f()  == rhs.get_f())
                {
                        if (lhs.get_g() == rhs.get_g())
                            if (lhs.tie_breaker  == rhs.tie_breaker)
                                return rand() % 2;
                            else
                                return lhs.tie_breaker < rhs.tie_breaker;
                        else
                            return lhs.get_g() > rhs.get_g();
                }
                else
                    return lhs.get_f() < rhs.get_f();
            }
            else
                return lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() < rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f();

    }
};


}
#endif