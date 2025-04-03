//
// Created by take_ on 24-12-27.
//

#ifndef BASICGRAPH_H
#define BASICGRAPH_H

#include "common.h"
#include "States.h"

#define WEIGHT_MAX INT_MAX/2


class BasicGraph
{
public:
    vector<std::string> types;
    // 每个地图点到所有地图点的启发式距离（可以带旋转）
    boost::unordered_map<int, vector<double>> heuristics;
    virtual ~BasicGraph()= default;
    string map_name;
    virtual bool load_map(string fname) = 0;
    [[nodiscard]] list<State> get_neighbors(const State& v) const;
    [[nodiscard]] list<int> get_neighbors(int v) const;
    [[nodiscard]] list<State> get_reverse_neighbors(const State& v) const; // ignore time
    [[nodiscard]] double get_weight(int from, int to) const; // fiducials from and to are neighbors
    [[nodiscard]] vector<vector<double> > get_weights() const {return weights; }
    [[nodiscard]] int get_rotate_degree(int dir1, int dir2) const; // return 0 if it is 0; return 1 if it is +-90; return 2 if it is 180

    void print_map() const;
    [[nodiscard]] int get_rows() const { return rows; }
    [[nodiscard]] int get_cols() const { return cols; }
    [[nodiscard]] int size() const { return rows * cols; }

    [[nodiscard]] bool valid_move(int loc, int dir) const {return (weights[loc][dir] < WEIGHT_MAX - 1); }
    [[nodiscard]] int get_Manhattan_distance(int loc1, int loc2) const;
    int move[4];
    void copy(const BasicGraph& copy);
    [[nodiscard]] int get_direction(int from, int to) const;

    [[nodiscard]] vector<double> compute_heuristics(int root_location) const; // compute distances from all lacations to the root location
    bool load_heuristics_table(std::ifstream& myfile);
    void save_heuristics_table(string fname);

    int rows;
    int cols;
    vector<vector<double> > weights; // (directed) weighted 4-neighbor grid
    bool consider_rotation;
};

#endif // BASICGRAPH_H
