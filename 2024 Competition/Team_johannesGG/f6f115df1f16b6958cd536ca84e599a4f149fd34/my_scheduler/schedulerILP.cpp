#include "schedulerILP.h"

using namespace operations_research;

namespace schedulerILP{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::vector<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(free_tasks.end(), env->new_tasks.begin(), env->new_tasks.end());






    auto startElapsed = std::chrono::high_resolution_clock::now();
    clock_t start = clock();

    if (free_agents.size() > 0 && free_tasks.size() > 0){
        // Initialize random seed
        std::srand(std::time(nullptr));

        // Problem size
        const int num_agents = env->num_of_agents;
        const int num_tasks = free_tasks.size();

        int a_loc, t_loc, curr_task_id;

        // Generate a random cost matrix
        std::vector<std::vector<int>> cost_matrix(num_agents, std::vector<int>(num_tasks));
        for (int i = 0; i < num_agents; ++i) {

            if (env->curr_task_schedule[i] == -1){
                a_loc = env->curr_states.at(i).location;
            }else{
                curr_task_id = env->curr_task_schedule[i];
                a_loc = env->task_pool[curr_task_id].locations.back();
            }

            for (int j = 0; j < num_tasks; ++j) {
                t_loc = env->task_pool[free_tasks[j]].locations[0];
                cost_matrix[i][j] = DefaultPlanner::get_h(env, a_loc, t_loc);
            }
        }

        
        // Create the solver
        std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
        if (!solver) {
            std::cerr << "SCIP solver not available." << std::endl;
        }

        // Decision variables: x[i][j] = 1 if agent i is assigned to task j, else 0
        std::vector<std::vector<const MPVariable*>> x(num_agents, std::vector<const MPVariable*>(num_tasks, nullptr));
        for (int i = 0; i < num_agents; ++i) {
            for (int j = 0; j < num_tasks; ++j) {
                x[i][j] = solver->MakeNumVar(0.0, 1.0, "x_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }

        // Objective function: Minimize the total cost
        MPObjective* const objective = solver->MutableObjective();
        for (int i = 0; i < num_agents; ++i) {
            for (int j = 0; j < num_tasks; ++j) {
                objective->SetCoefficient(x[i][j], cost_matrix[i][j]);
            }
        }
        objective->SetMinimization();


        // tasks >= agents: all agents get one task
        if (num_tasks >= num_agents){
            for (int i = 0; i < num_agents; ++i) {
                LinearExpr agent_constraint;
                for (int j = 0; j < num_tasks; ++j) {
                    agent_constraint += x[i][j];
                }
                solver->MakeRowConstraint(agent_constraint == 1);
            }
            for (int j = 0; j < num_tasks; ++j) {
                LinearExpr task_constraint;
                for (int i = 0; i < num_agents; ++i) {
                    task_constraint += x[i][j];
                }
                solver->MakeRowConstraint(task_constraint <= 1);
            }
        }
        //tasks < agents: all tasks get one agent
        else {
            for (int j = 0; j < num_tasks; ++j) {
                LinearExpr task_constraint;
                for (int i = 0; i < num_agents; ++i) {
                    task_constraint += x[i][j];
                }
                solver->MakeRowConstraint(task_constraint == 1);
            }

            // tasks >= emtpy agents (aka have no task assigned at all): all empty agents get one task
            int empty_agents = std::count(env->curr_task_schedule.begin(), env->curr_task_schedule.end(), -1);
            if (num_tasks >= empty_agents){
                for (int i = 0; i < num_agents; ++i) {
                    LinearExpr agent_constraint;
                    for (int j = 0; j < num_tasks; ++j) {
                        agent_constraint += x[i][j];
                    }
                    if (env->curr_task_schedule[i] == -1){
                        solver->MakeRowConstraint(agent_constraint == 1);
                    }
                    else {
                        solver->MakeRowConstraint(agent_constraint <= 1);
                    }
                }
            }
            // tasks < empty agents: all non-empty agents get no task
            else{
                for (int i = 0; i < num_agents; ++i) {
                    LinearExpr agent_constraint;
                    for (int j = 0; j < num_tasks; ++j) {
                        agent_constraint += x[i][j];
                    }
                    if (env->curr_task_schedule[i] == -1){
                        solver->MakeRowConstraint(agent_constraint <= 1);
                    }
                    else {
                        solver->MakeRowConstraint(agent_constraint == static_cast<double>(0));
                    }
                }
            }
        }


        const MPSolver::ResultStatus result_status = solver->Solve();

       

        if (result_status != MPSolver::OPTIMAL) {
            std::cerr << "The problem does not have an optimal solution!" << std::endl;
        }



        std::vector<int> tasks_to_remove;

        for (int i = 0; i < num_agents; ++i){
            if (env->curr_task_schedule[i] == -1){
                bool solutionFound = false;
                for (int j = 0; j < num_tasks; ++j){
                    if (x[i][j]->solution_value() > 0.5){
                        proposed_schedule[i] = free_tasks[j];
                        tasks_to_remove.push_back(j);
                        free_agents.erase(i);
                        solutionFound = true;
                        break;
                    }
                }
                if (!solutionFound){
                    std::cout << "Keine Loesung gefunden du Spacko XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
                }
            }
        }


        //remove tasks in decending order since each time vector changes
        std::sort(tasks_to_remove.rbegin(), tasks_to_remove.rend());
        for (int index : tasks_to_remove) {
            free_tasks.erase(free_tasks.begin() + index);
        }
    }


    // Print elapsed time
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - startElapsed;
    std::cout << "Time Total: " << elapsed.count() << " seconds" << std::endl;


    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}
