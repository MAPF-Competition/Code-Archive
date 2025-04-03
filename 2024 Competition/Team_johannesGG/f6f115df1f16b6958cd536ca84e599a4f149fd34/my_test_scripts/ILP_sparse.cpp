#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "ortools/linear_solver/linear_solver.h"

using namespace operations_research;

int main() {
    // Initialize random seed
    std::srand(std::time(nullptr));

    // Problem size
    const int num_agents = 600;
    const int num_tasks = 300;

    const int cost_threshold = 30;

    // Generate a random cost matrix
    std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks));
    for (int i = 0; i < num_agents; ++i) {
        for (int j = 0; j < num_tasks; ++j) {
            cost_matrix[i][j] = 2 + (std::rand() % 99);  // Random cost in [2, 100]
        }
    }

    auto start = std::chrono::high_resolution_clock::now();


    // Create the solver
    // std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
    if (!solver) {
        std::cerr << "SCIP solver not available." << std::endl;
        return 1;
    }




    // Decision variables: Only for agent-task pairs with cost < cost_threshold
    std::unordered_map<int, std::unordered_map<int, const MPVariable*>> x;  // Sparse matrix
    for (int i = 0; i < num_agents; ++i) {
        for (int j = 0; j < num_tasks; ++j) {
            if (cost_matrix[i][j] < cost_threshold) {
            // x[i][j] = solver->MakeIntVar(0, 1, "x_" + std::to_string(i) + "_" + std::to_string(j));
                x[i][j] = solver->MakeNumVar(0.0, 1.0, "x_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }
    }

    // Objective function: Minimize the total cost
    MPObjective* const objective = solver->MutableObjective();
    for (int i = 0; i < num_agents; ++i) {
        for (int j = 0; j < num_tasks; ++j) {
            if (cost_matrix[i][j] < cost_threshold) {
                objective->SetCoefficient(x[i][j], cost_matrix[i][j]);
            }
        }
    }
    objective->SetMinimization();

    // Constraints: Each agent is assigned to at most one task
    for (int i = 0; i < num_agents; ++i) {
        LinearExpr agent_constraint;
        for (int j = 0; j < num_tasks; ++j) {
            if (cost_matrix[i][j] < cost_threshold) {
                agent_constraint += x[i][j];
            }
        }

        // Optional assignment for most agents
        if (i != 12 && i != 50 && i != 73) {
            solver->MakeRowConstraint(agent_constraint <= 1);
        }
        // Mandatory assignment for agents 12, 50, and 73
        else {
            solver->MakeRowConstraint(agent_constraint == 1);
        }
    }

    // Constraints: Each task is assigned to exactly one agent
    for (int j = 0; j < num_tasks; ++j) {
        LinearExpr task_constraint;
        for (int i = 0; i < num_agents; ++i) {
            if (cost_matrix[i][j] < cost_threshold) {
                task_constraint += x[i][j];
            }
        }
        solver->MakeRowConstraint(task_constraint == 1);
    }

    // Measure the time to solve
    const MPSolver::ResultStatus result_status = solver->Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    if (result_status != MPSolver::OPTIMAL) {
        std::cerr << "The problem does not have an optimal solution!" << std::endl;
    }

    // Output the results
    std::cout << "Optimal Total Cost: " << objective->Value() << std::endl;
    std::cout << "Optimal Assignments:" << std::endl;
    for (int i = 0; i < num_agents; ++i) {
        bool solutionFound = false;
        for (int j = 0; j < num_tasks; ++j) {
            if (cost_matrix[i][j] < cost_threshold) {
                if (x[i][j]->solution_value() > 0.5) {
                    // std::cout << "Agent " << i << " assigned to Task " << j
                    //           << " with Cost " << cost_matrix[i][j] << std::endl;
                    solutionFound = true;
                }
            }
        }
        if (!solutionFound){
            std::cout << "Keine Loesung gefunden XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
        }
    }

    // Print elapsed time
    std::cout << "Time to solve: " << elapsed.count() << " seconds" << std::endl;

    return 0;
}
