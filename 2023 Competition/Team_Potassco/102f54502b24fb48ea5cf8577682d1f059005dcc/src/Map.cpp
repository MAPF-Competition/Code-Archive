#include "Map.h"
#include <stdexcept>
#include <iostream>
#include <queue>
#include <cstdlib>

int MAP::dir_pos_to_int(directional_pos pos){
    // use 29 bits to store the first number and 2 bits to store the second, 
    // leave the 32th for the sign, although we do not need a sign
    if (pos.first > 536870912 || pos.second > 3){
        throw std::invalid_argument("directional pos is too big to be combined");
    }
    return (pos.first << 2) + pos.second;
}

int MAP::serialize_2d(int x, int y){
    // serialize a 2d position into a 1d position
    return (y * m_cols) + x;
}

std::pair<int, int> MAP::deserialize_2d(int u){
    // deserialize a 1d position into a 2d position
    int x = u % m_cols;
    int y = u / m_cols;
    return std::make_pair(x, y);
}

//! MAPF class for capturing MAPF problems.
Node * MAP::add_node(directional_pos u){
    int combined = dir_pos_to_int(u);
    auto ins = node_map_.try_emplace(combined, u);
    if (ins.second) {
        nodes_.emplace_back(&ins.first->second);
    }
    return &ins.first->second;
}

void MAP::add_edge(directional_pos u, directional_pos v) {
    auto *n_u = add_node(u);
    auto *n_v = add_node(v);
    n_u->out.emplace_back(n_v);
    n_v->in.emplace_back(n_u);
}

//! Initialize the problem from the given control object.
void MAP::init(std::vector<int> serialized_map, int rows, int cols) {
    int mapsize = serialized_map.size();
    m_rows = rows;
    m_cols = cols;
    std::cout << "starting init with map size " << mapsize << std::endl;
    for (int i=0 ; i < mapsize; i++){
        if (serialized_map[i] == 0){
            // directional nodes for the vertex
            directional_pos node_up = std::make_pair(i,Orientation::up);
            directional_pos node_down = std::make_pair(i,Orientation::down);
            directional_pos node_left = std::make_pair(i,Orientation::left);
            directional_pos node_right = std::make_pair(i,Orientation::right);

            // add edges between directional nodes
            add_edge(node_up, node_left);
            add_edge(node_left, node_down);
            add_edge(node_down, node_right);
            add_edge(node_right, node_up);
            add_edge(node_up, node_right);
            add_edge(node_right, node_down);
            add_edge(node_down, node_left);
            add_edge(node_left, node_up);

            // check neighbors and add edges
            std::vector<directional_pos> neighbors = get_map_neighbors(i, serialized_map);
            auto neighbors_len = neighbors.size();
            for (int j = 0; j < neighbors_len; j++){
                int pos = neighbors[j].first;
                Orientation ori = neighbors[j].second;
                switch (ori)
                {
                case Orientation::up:
                    add_edge(node_up, std::make_pair(pos,Orientation::up));
                    break;
                case Orientation::down:
                    add_edge(node_down, std::make_pair(pos,Orientation::down));
                    break;
                case Orientation::left:
                    add_edge(node_left, std::make_pair(pos,Orientation::left));
                    break;
                case Orientation::right:
                    add_edge(node_right, std::make_pair(pos,Orientation::right));
                    break;
                }                
            }
        }
    }
    std::cout << "init done" << std::endl;
    std::cout << "Final number of nodes is: " << nodes_.size() << std::endl;
}

auto MAP::get_grid_neighbors(int u) -> std::vector<directional_pos> {
    // it might be a neighbor in the given direction
    std::vector<directional_pos> neighbors;

    auto [x,y] = deserialize_2d(u);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy){
            if (!(dx == 0 && dy == 0) && x + dx >= 0 && x + dx < m_cols && y + dy >= 0 && y + dy < m_rows && (abs(dx)+abs(dy)==1)){
                // if the tested position is inside the grid
                Orientation orientation;
                if (dx == 1 && dy == 0){
                    orientation = Orientation::right;
                }
                else if (dx == 0 && dy == 1){
                    orientation = Orientation::down;
                }
                else if (dx == -1 && dy == 0){
                    orientation = Orientation::left;
                }
                else if (dx == 0 && dy == -1){
                    orientation = Orientation::up;
                }
                neighbors.push_back(std::make_pair(serialize_2d(x+dx, y+dy), orientation));
            }
        }
    }

    return neighbors;
}

auto MAP::get_map_neighbors(int u, std::vector<int> serialized_map) -> std::vector<directional_pos> {
    // maybe I should do this in the get grid neighbors function
    //  to avoid looping twice over the neighbors
    std::vector<directional_pos> map_neighbors; // neighbors actually in the map( they have an edge between them)
    std::vector<directional_pos> neighbors = get_grid_neighbors(u); // possible neighbors

    int neighbors_len = neighbors.size();

    for (int i= 0; i < neighbors_len; i++){
        // it is a real neighbor if the tested position is not an obstacle
        if (serialized_map[neighbors[i].first] == 0){
            map_neighbors.push_back(neighbors[i]);
        }
    }
    return map_neighbors;
}

void MAP::bfs(directional_pos node){
    // bfs from the given node if not done yet
    int combined = dir_pos_to_int(node);
    Node &start_node = node_map_.at(dir_pos_to_int(node));  

    // end if bfs is already done for this done
    if (start_node.distances.size() != 0){
        return;
    }

    std::queue<Node*> q_;
   
    q_.push(&start_node);
    start_node.distances[&start_node] = 0;
    while (!q_.empty()){

        Node* curr_node = q_.front();
        q_.pop();
        std::vector<Node*> neighbors = curr_node->out;

        int dist = start_node.distances.at(curr_node);
        for (int i = 0; i < neighbors.size(); i++){
            Node* neighbor = neighbors[i];

            if (!start_node.distances.count(neighbor)){
                start_node.distances[neighbor] = dist + 1;
                q_.push(neighbor);
            }
        }
    }
}

int MAP::distance_between(directional_pos node1, directional_pos node2){
    // return the distance between two nodes
    // always uses the distance map from node1
    int combined1 = dir_pos_to_int(node1);
    int combined2 = dir_pos_to_int(node2);
    Node &n1 = node_map_.at(combined1);
    Node &n2 = node_map_.at(combined2);

    int dist = n1.distances.at(&n2);
    return dist;
}

int MAP::distance_between(Node* node1, Node* node2){
    // same as the other function
    int dist = node1->distances.at(node2);
    return dist;
}

void MAP::bfs_all_nodes(){
    // dfs from all nodes
    // apparently it is too memory intensive!
    for (int i = 0; i < nodes_.size(); i++){
        // print node count every 1000 nodes

        bfs(nodes_[i]->name);

        if (i % 500 == 0){
            std::cout << "bfs from node " << i << std::endl;
        }
    }
}

std::vector< std::unordered_set<Node*> > MAP::reachable_nodes_within_distance(int within_dist, int maxtime, directional_pos start, directional_pos goal){
    // return all nodes within a given distance from the given node
    std::vector< std::unordered_set<Node*> > reach;
    int combined = dir_pos_to_int(start);
    Node &start_node = node_map_.at(combined);

    combined = dir_pos_to_int(goal);
    Node &goal_node = node_map_.at(combined);
    
    reach.push_back(std::unordered_set<Node*>());
    reach[0].insert(&start_node);

    int dist_to_goal = distance_between(&goal_node, &start_node);

    // if the distance to the goal is smaller than the within_dist
    // limit the reachability check to this value!
    int max_dist = within_dist;
    if (dist_to_goal < maxtime){
        max_dist = dist_to_goal;
    }

    // Loop through nodes in previous step
    // for each neihgbor, check that it can still reach the goal
    // if it can, add it to the next step
    // else, do nothing
    // leq on check to within dist since we do not count step 0
    for (int time = 1; time <= within_dist; time++){
        reach.push_back(std::unordered_set<Node*>());

        for (Node* node : reach[time-1]){
            std::vector<Node*> neighbors = node->out;
            neighbors.push_back(node);

            for(Node* neighbor : neighbors){
                if (distance_between(&goal_node, neighbor) <= maxtime - time ){
                    reach[time].insert(neighbor);
                }
            }
        }
    }
    return reach;
}
