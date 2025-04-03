///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class Hungarian.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#include <cstdlib>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include "hungarian.h"


Hungarian::Hungarian(vector <vector<double> >& input_cost_matrix)
{
    cost_matrix = input_cost_matrix;
    num_rows = cost_matrix.size();
    num_columns = cost_matrix[0].size();

    assignment.resize(num_rows);
}


//********************************************************//
// Solve optimal solution for p_assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
double Hungarian::find_optimal_assignment()
{
    double min_value;

    /* initialization */
    // 改动后将这里的行操作换成列操作。
    for (row_index = 0; row_index < num_rows; row_index++)
    {
        assignment[row_index] = -1;
    }

    /* generate working copy of distance Matrix */
    /* check if all matrix elements are positive */
    dist_matrix = cost_matrix;

    /* memory allocation */
    covered_columns.resize(num_columns, false);
    covered_rows.resize(num_rows);

    prime_matrix.resize(num_rows);
    for(row_index=0; row_index < num_rows; row_index++)
    {
        prime_matrix[row_index].resize(num_columns);
    }

    star_matrix.resize(num_rows);
    for(row_index=0; row_index < num_rows; row_index++)
    {
        star_matrix[row_index].resize(num_columns);
    }

    new_star_matrix.resize(num_rows);
    for(row_index=0; row_index < num_rows; row_index++)
    {
        new_star_matrix[row_index].resize(num_columns);
    }

    /* preliminary steps */
    if (num_rows <= num_columns)
    {
        min_dim = num_rows;

        // wiki step 1: Solve optimal solution for p_assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
        // 改动后将这里的列操作换成行操作。
        for (row_index = 0; row_index < num_rows; row_index++)
        {
            /* find the smallest element in the row_index */
            min_value = dist_matrix[row_index][0];
            for(column_index=0; column_index < num_columns; column_index++)
            {
                if(dist_matrix[row_index][column_index] < min_value)
                {
                    min_value = dist_matrix[row_index][column_index];
                }
            }

            /* subtract the smallest element from each element of the row_index */
            for(column_index=0; column_index < num_columns; column_index++)
            {
                if(dist_matrix[row_index][column_index] < min_value)
                {
                    dist_matrix[row_index][column_index] -= min_value;
                }
            }
        }

        /* Steps 1 and 2a */
        // 找出覆盖的列
        // 改动后将这里的列操作换成行操作，行操作换成列操作。
        for (row_index = 0; row_index < num_rows; row_index++)
        {
            for (column_index = 0; column_index < num_columns; column_index++)
            {
                if (fabs(dist_matrix[row_index][column_index]) < DBL_EPSILON)  // 改动后将这里的列检查换成行检查
                {
                    // cerr << "covered_columns[column_index]" << covered_columns[column_index] << endl;
                    if (!covered_columns[column_index])
                    {
                        star_matrix[row_index][column_index] = true;

                        covered_columns[column_index] = true;

                        break;
                    }
                }
            }
        }
    }
    else /* if(input_num_rows > input_num_columns) */
    {
        min_dim = num_columns;

        for (column_index = 0; column_index < num_columns; column_index++)
        {
            /* find the smallest element in the column */
            // 改动后将这里的列操作换成行操作
            min_value = dist_matrix[0][column_index];
            for(row_index=0; row_index < num_rows; row_index++)
            {
                if(dist_matrix[row_index][column_index] < min_value)
                {
                    min_value = dist_matrix[row_index][column_index];
                }
            }

            /* subtract the smallest element from each element of the column */
            // 改动后将这里的列操作换成行操作
            for(row_index=0; row_index < num_rows; row_index++)
            {
                if(dist_matrix[row_index][column_index] < min_value)
                {
                    dist_matrix[row_index][column_index] -= min_value;
                }
            }
        }

        /* Steps 1 and 2a */
        for (column_index = 0; column_index < num_columns; column_index++)
        {
            for (row_index = 0; row_index < num_rows; row_index++)
            {
                if (fabs(dist_matrix[row_index][column_index]) < DBL_EPSILON) // 改动后将这里的列检查换成行检查
                {
                    if (!covered_rows[row_index])
                    {
                        star_matrix[row_index][column_index] = true;

                        covered_columns[column_index] = true;
                        covered_rows[row_index] = true;

                        break;
                    }
                }
            }
        }

        for (row_index = 0; row_index < num_rows; row_index++)
        {
            covered_rows[row_index] = false;
        }

    }

    /* move to step 2b */
    step2b();

    /* compute cost and remove invalid assignments */
    double cost = compute_assignment_cost();

    return cost;
}


/********************************************************/
void Hungarian::step2a()
{
    /* cover every column containing a starred zero */
    for (column_index = 0; column_index < num_columns; column_index++)
    {
        for(row_index=0; row_index < num_rows; row_index++)
        {
            bool currentVal = star_matrix[row_index][column_index];
            if (currentVal)
            {
                covered_columns[column_index] = true;
                break;
            }
        }
    }

    /* move to step 3 */
    step2b();
}


/********************************************************/
void Hungarian::step2b()
{
    /* count covered columns */
    int num_covered_columns = 0;
    for (column_index = 0; column_index < num_columns; column_index++)
    {
        if (covered_columns[column_index])
        {
            num_covered_columns++;
        }
    }

    if (num_covered_columns == min_dim) // 已经全部覆盖
    {
        /* algorithm finished */
        build_assignment_vector();
    }
    else
    {
        /* move to step 3 */
        step3();
    }

}


// wiki step 3
// 必须用尽可能少的列或行标记来覆盖矩阵中的所有零。
void Hungarian::step3()
{
    int star_col;

    bool zeros_found = true;
    while (zeros_found)
    {
        zeros_found = false;
        for (column_index = 0; column_index < num_columns; column_index++)
        {
            if (!covered_columns[column_index])
            {
                for (row_index = 0; row_index < num_rows; row_index++)
                {
                    if ((!covered_rows[row_index]) && (fabs(dist_matrix[row_index][column_index]) < DBL_EPSILON))
                    {
                        /* prime zero */
                        prime_matrix[row_index][column_index] = true;

                        /* find starred zero in current row_index */
                        for (star_col = 0; star_col < num_columns; star_col++)
                        {
                            if (star_matrix[row_index][star_col])
                            {
                                break;
                            }
                        }

                        if (star_col == num_columns) /* no starred zero found */
                        {
                            /* move to step 4 */
                            step4(row_index, column_index);

                            return;
                        }
                        else
                        {
                            covered_rows[row_index] = true;
                            covered_columns[star_col] = false;
                            zeros_found = true;

                            break;
                        }
                    }
                }
            }
        }
    }

    /* move to step 5 */
    step5();
}


/********************************************************/
void Hungarian::step4(int input_row, int input_column)
{
    int star_row, star_column, prime_row, prime_column;
    int num_elements = num_rows * num_columns;

    /* generate temporary copy of star_matrix */
    new_star_matrix = star_matrix;

    /* star current zero */
    new_star_matrix[input_row][input_column] = true;

    /* find starred zero in current column */
    star_column = input_column;
    for (star_row = 0; star_row < num_rows; star_row++)
    {
        if (star_matrix[star_row][star_column])
        {
            break;
        }
    }

    while (star_row < num_rows)
    {
        /* unstar the starred zero */
        new_star_matrix[star_row][star_column] = false;

        /* find primed zero in current input_row */
        prime_row = star_row;
        for (prime_column = 0; prime_column < num_columns; prime_column++)
        {
            if (prime_matrix[prime_row][prime_column])
            {
                break;
            }
        }

        /* star the primed zero */
        new_star_matrix[prime_row][prime_column] = true;

        /* find starred zero in current column */
        star_column = prime_column;
        for (star_row = 0; star_row < num_rows; star_row++)
        {
            if (star_matrix[star_row][star_column])
            {
                break;
            }
        }
    }

    /* use temporary copy as new star_matrix */
    /* delete all primes, uncover all rows */
    for (row_index=0; row_index < num_rows; row_index++)
    {
        for(column_index=0; column_index < num_columns; column_index++)
        {
            prime_matrix[row_index][column_index] = false;
        }
    }

    star_matrix = new_star_matrix;

    for (int n = 0; n < num_rows; n++)
    {
        covered_rows[n] = false;
    }

    /* move to step 2a */
    step2a();
}


// Find the lowest uncovered value. Subtract this from every unmarked element and add it to every element covered by two lines.
void Hungarian::step5()
{
    double smallest_uncoverd, value;

    /* find smallest uncovered element smallest_uncoverd */
    smallest_uncoverd = DBL_MAX;
    for (row_index = 0; row_index < num_rows; row_index++)
    {
        if (!covered_rows[row_index])
        {
            for (column_index = 0; column_index < num_columns; column_index++)
            {
                if (!covered_columns[column_index])
                {
                    value = dist_matrix[row_index][column_index];
                    if (value < smallest_uncoverd)
                    {
                        smallest_uncoverd = value;
                    }
                }
            }
        }
    }

    /* add smallest_uncovered to each covered row_index */
    for (row_index = 0; row_index < num_rows; row_index++)
    {
        if (covered_rows[row_index])
        {
            for (column_index = 0; column_index < num_columns; column_index++)
            {
                dist_matrix[row_index][column_index] += smallest_uncoverd;
            }
        }
    }

    /* subtract smallest_uncoverd from each uncovered column */
    for (column_index = 0; column_index < num_columns; column_index++)
    {
        if (!covered_columns[column_index])
        {
            for (row_index = 0; row_index < num_rows; row_index++)
            {
                dist_matrix[row_index][column_index] -= smallest_uncoverd;
            }
        }
    }

    /* move to step 3 */
    step3();
}

/********************************************************/
void Hungarian::build_assignment_vector()
{
    for (row_index = 0; row_index < num_rows; row_index++)
    {
        for (column_index = 0; column_index < num_columns; column_index++)
        {
            if (star_matrix[row_index][column_index])
            {
                assignment[row_index] = column_index;

                break;
            }
        }
    }
}

/********************************************************/
double Hungarian::compute_assignment_cost()
{
    double cost = 0;
    int row, col;

    for (row = 0; row < num_rows; row++)
    {
        col = assignment[row];
        if (col >= 0)
        {
            cost += cost_matrix[row][col];
        }
    }

    return cost;
}


//********************************************************//
// A single function wrapper for solving assignment problem.
//********************************************************//
double Hungarian::Solve(vector<int>& input_assignments)
{
    // call solving function
    double cost = find_optimal_assignment();

    input_assignments.clear();
    for (row_index = 0; row_index < num_rows; row_index++)
    {
        input_assignments.push_back(assignment[row_index]);
    }

    return cost;
}

void Hungarian::print_cost_matrix()
{
    cout << "cost matrix: " << endl;
    for(auto const & row : cost_matrix)
    {
        for(auto element : row)
        {
            cout << element << " ";
        }
        cout << endl;
    }
}