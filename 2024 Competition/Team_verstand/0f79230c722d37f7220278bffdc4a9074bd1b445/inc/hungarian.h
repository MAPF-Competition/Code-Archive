///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class Hungarian.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#ifndef HUNGARIAN_HPP
#define HUNGARIAN_HPP

#include <iostream>
#include <vector>

using namespace std;


class Hungarian
{
private:
    vector< vector<double> > cost_matrix; // 原版cost_matrix还是需要保留的，因为最后要计算总cost.
    vector<int> assignment;
    vector< vector<double> > dist_matrix; // num_row * num_column
    vector< vector<bool> > star_matrix; // num_row * num_column
    vector< vector<bool> > new_star_matrix; // num_row * num_column
    vector< vector<bool> > prime_matrix; // num_row * num_column

    vector<bool> covered_columns; // dim: num_columns
    vector<bool> covered_rows; // dim: num_rows

    unsigned num_rows;
    unsigned num_columns;
    int row_index, column_index;

    int min_dim;

public:
    Hungarian(vector <vector<double> >& input_cost_matrix);

    double find_optimal_assignment();

    void build_assignment_vector();

    double compute_assignment_cost();

    void step2a();

    void step2b();

    void step3();

    void step4(int input_row, int input_column);

    void step5();

    double Solve(vector<int>& input_assignments);

    void print_cost_matrix();
};


#endif