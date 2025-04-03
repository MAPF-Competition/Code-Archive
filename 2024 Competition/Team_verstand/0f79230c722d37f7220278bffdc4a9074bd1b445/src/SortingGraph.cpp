//
// Created by take_ on 24-12-27.
//
#include "SortingGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool SortingGrid::load_map(std::string fname)
{
    std::string line;
    std::ifstream myfile ((fname).c_str());
    if (!myfile.is_open())
    {
        std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }

    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
    std::size_t pos = fname.rfind('.');      // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline (myfile, line); // skip the words "grid size"
    getline(myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
    this->rows = atoi((*beg).c_str()); // read number of cols
    beg++;
    this->cols = atoi((*beg).c_str()); // read number of rows
    move[0] = 1;
    move[1] = -cols;
    move[2] = -1;
    move[3] = cols;

    getline(myfile, line); // skip the headers

    //read tyeps, station ids and edge weights
    this->types.resize(rows * cols);
    this->weights.resize(rows * cols);
    for (int i = 0; i < rows * cols; i++)
    {
        getline(myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++; // skip id
        this->types[i] = std::string(beg->c_str()); // read type
        beg++;
        if (types[i] == "Induct")
            this->inducts[beg->c_str()] = i; // read induct station id
        else if (types[i] == "Eject")
        {
            boost::unordered_map<std::string, std::list<int> >::iterator it = ejects.find(beg->c_str());
            if (it == ejects.end())
            {
                this->ejects[beg->c_str()] = std::list<int>();
            }
            this->ejects[beg->c_str()].push_back(i); // read eject station id
        }
        beg++;
        beg++; // skip x
        beg++; // skip y
        weights[i].resize(5);
        for (int j = 0; j < 5; j++) // read edge weights
        {
            if (std::string(beg->c_str()) == "inf")
                weights[i][j] = WEIGHT_MAX;
            else
                weights[i][j] = std::stod(beg->c_str());
            beg++;
        }
    }

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with " << inducts.size() << " induct stations and " <<
              ejects.size() << " eject stations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}


bool SortingGrid::load_LoRR_map(SharedEnvironment* env)
{
    std::cout << "*** Loading LoRR map ***" << std::endl;
    clock_t t = std::clock();
    map_name = "LoRR";     // get the name without extension
    this->rows = env->rows; // read number of cols
    this->cols = env->cols; // read number of rows
    move[0] = 1;
    move[1] = -cols;
    move[2] = -1;
    move[3] = cols;

    // read tyeps, station ids and edge weights
    this->types.resize(rows * cols);
    this->weights.resize(rows * cols);
    for (int i = 0; i < rows * cols; i++)
    {
        // 把type标记为Travel和Obstacle
        // map[loc] == 0: 当前位置可通行。map[loc] == 1: 当前位置为障碍物，不可通行。
        if(env->map[i] == 0)
        {
            types[i] = "Travel";
        }
        else if(env->map[i] == 1)
        {
            types[i] = "Obstacle";
        }
    }

    /*// 检测type是否正确读入
    for(int i = 0; i < rows * cols; i++)
    {
        if(i % cols == 0)
        {
            cout << endl;
        }

        cout << types[i] << " ";
    }
     */

    // 根据上下左右的邻居是否存在以及可通行, 计算weight
    for (int row=0; row<env->rows; row++)
    {
        for (int col=0; col<env->cols; col++)
        {
            int loc = row*env->cols+col;
            weights[loc].resize(5, WEIGHT_MAX);
            weights[loc][4] = 1;
            // map[loc] == 0: 当前位置可通行。map[loc] == 1: 当前位置为障碍物，不可通行。
            if (env->map[loc]==0)
            {
                // 为当前节点 loc 添加其上方邻居节点。
                if (row>0 && env->map[loc-env->cols]==0){
                    weights[loc][1] = 1;
                }

                // 下方邻居
                if (row<env->rows-1 && env->map[loc+env->cols]==0){
                    weights[loc][3] = 1;
                }

                // 左侧邻居
                if (col>0 && env->map[loc-1]==0){
                    weights[loc][2] = 1;
                }

                // 右侧邻居
                if (col<env->cols-1 && env->map[loc+1]==0){
                    weights[loc][0] = 1;
                }
            }
        }
    }

    /*// 检测neighbors是否正确读入
    for(int i = 0; i < rows * cols; i++)
    {
        if(i % cols == 0)
        {
            cout << endl;
        }

        cout << weights[i][0] << " " << weights[i][1] << " " << weights[i][2] << " "
                << weights[i][3] << " " << weights[i][4] << " ";
    }
     //*/

    //*
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with " << inducts.size() << " induct stations and " <<
              ejects.size() << " eject stations." << std::endl;
    std::cout << "Load Map Done! (" << runtime << " s)" << std::endl;
     //*/
    return true;
}


void SortingGrid::preprocessing(bool consider_rotation)
{
    std::cout << "*** PreProcessing map ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    /*// LoRR无处load
    std::string fname;
    if (consider_rotation)
        fname = map_name + "_rotation_heuristics_table.txt";
    else
        fname = map_name + "_heuristics_table.txt";
    std::ifstream myfile(fname.c_str());
    bool succ = false;
    if (myfile.is_open())
    {
        succ = load_heuristics_table(myfile);
        myfile.close();
        // ensure that the heuristic table is correct
        for (auto h : heuristics)
        {
            if (types[h.first] != "Induct" && types[h.first] != "Eject")
            {
                cout << "The heuristic table does not match the map!" << endl;
                exit(-1);
            }
        }
    }
     */
    // if (!succ)
    {
        for (const auto& induct : inducts)
        {
            heuristics[induct.second] = compute_heuristics(induct.second);
        }
        for (const auto& eject_station : ejects)
        {
            for (int eject : eject_station.second)
            {
                heuristics[eject] = compute_heuristics(eject);
            }
        }
        // save_heuristics_table(fname);
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

void SortingGrid::preprocessing_LoRR(bool consider_rotation, SharedEnvironment* env)
{
    std::cout << "*** PreProcessing LoRR map ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;

    /*
    for(int i=0;i<env->map.size();i++)
    {
        if(env->map[i] == 0)
        {
            heuristics[i] = compute_heuristics(i);
        }
    }
     */

    // save_heuristics_table("LoRR_heuristics_table.txt");

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "PreProcessing Done! (" << runtime << " s)" << std::endl;
}
