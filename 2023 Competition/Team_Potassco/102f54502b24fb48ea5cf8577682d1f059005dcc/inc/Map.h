#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>


//{0: "r", 1: "d", 2: "l", 3: "u"}
enum Orientation {right=0, down=1, left=2, up=3};

// int is the serialed position in the graph, Orientation is the direction of the edge
typedef std::pair<int,Orientation> directional_pos;

struct Node {
    //! Construct a node with the given name.
    Node(directional_pos name){
        this->name = name;
    }

    inline bool operator == (const Node& rhs) const {
        return this->name == rhs.name;
    }

    //! The name of the node.
    directional_pos name;
    //! The outgoing edges of the node.
    std::vector<Node *> out;
    //! The incoming edges of the node.
    std::vector<Node *> in;

    std::unordered_map<Node*, int> distances;

};

class MAP{

public:
    int dir_pos_to_int(directional_pos);

    int serialize_2d(int, int);

    std::pair<int, int> deserialize_2d(int);

    //! Add a node with the given name to the MAPF problem.
    //!
    //! Returns the same node for the same name.
    Node* add_node(directional_pos);

    //! Add an edge between two nodes.
    void add_edge(directional_pos, directional_pos);

    void init(std::vector<int>, int, int);

    auto get_grid_neighbors(int) -> std::vector<directional_pos>;

    auto get_map_neighbors(int, std::vector<int>) -> std::vector<directional_pos>;

    void bfs(directional_pos);

    int distance_between(directional_pos, directional_pos);
    int distance_between(Node*, Node*);

    void bfs_all_nodes();

    auto reachable_nodes_within_distance(int, int, directional_pos, directional_pos) -> std::vector< std::unordered_set<Node*> >;
    
private:
    //! Mapping from node names to actual nodes.
    std::unordered_map<int, Node> node_map_;
    //! The list of nodes in insertion order.
    std::vector<Node *> nodes_;

    int m_rows;
    int m_cols;


};