#include "schedulerILPsparse.h"

using namespace operations_research;

namespace schedulerILPsparse{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::vector<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{   
    std::cout << "preprocessing started" << std::endl;
    auto now = std::chrono::high_resolution_clock::now();

    DefaultPlanner::init_heuristics(env);
    mt.seed(0);

    for (int i = 0; i < env->map.size(); i++){
        // std::cout << "Field " << i << " of " <<  env->map.size() << std::endl; 
        for (int j = 0; j < env->map.size(); j++){
            if (env->map[i] == 0 && env->map[j] == 0){
                DefaultPlanner::get_h(env, i, j);
            }
        }
    }

    #ifndef NDEBUG
    std::chrono::duration<double> passed = std::chrono::high_resolution_clock::now() - now;
    std::cout << "A* heuristic initialisation: " << passed.count() << " seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    #endif
    
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{   
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> passed;
    clock_t start = clock();
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(free_tasks.end(), env->new_tasks.begin(), env->new_tasks.end());


    



    
    

    if (free_agents.size() > 0 && free_tasks.size() > 0){
        // Initialize random seed
        std::srand(std::time(nullptr));

        // Problem size
        const int num_agents = env->num_of_agents;
        const int num_tasks = free_tasks.size();

        int a_loc, t_loc, curr_task_id;

        #ifndef NDEBUG
        passed = std::chrono::high_resolution_clock::now() - now;
        std::cout << "Time beginning: " << passed.count() << " seconds" << std::endl;
        now = std::chrono::high_resolution_clock::now();
        #endif


        // Generate cost matrix - potetnial improvement: only do optimization, cost, etc. for agents who have no task or are at their last task
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
                cost_matrix[i][j] = get_h_limited(env, a_loc, t_loc);
                // cost_matrix[i][j] = DefaultPlanner::get_h(env, a_loc, t_loc);
            }
        }

        #ifndef NDEBUG
        passed = std::chrono::high_resolution_clock::now() - now;
        std::cout << "Time cost matrix: " << passed.count() << " seconds" << std::endl;
        now = std::chrono::high_resolution_clock::now();
        #endif

        // Generate activation matrix aka 1 where we consider the cost and 0 where we assume the cost is so bad that we dont consider it during optimization
        int number_of_pairs_to_consider = 3;

        std::vector<std::vector<int>> activation_matrix;
        if (number_of_pairs_to_consider >= std::min(num_agents, num_tasks)) {
            activation_matrix = std::vector<std::vector<int>>(num_agents, std::vector<int>(num_tasks, 1));
        } else {
            activation_matrix = std::vector<std::vector<int>>(num_agents, std::vector<int>(num_tasks, 0));
            // Diagonal = 1 for solvability
            for (int i = 0; i < std::min(num_agents, num_tasks); ++i) {
                activation_matrix[i][i] = 1;
            }
            // make sure all empty agents have at least one possible task for solvability
            if (num_tasks < num_agents){
                int i = 0;
                int j = 0;
                while (i < num_agents && j < num_tasks){
                    if (env->curr_task_schedule[i] == -1){
                        activation_matrix[i][j] = 1;
                        i++;
                        j++;
                    }else{
                        i++;
                    }
                }
            }
            // iterate agents and find their number_of_pairs_to_consider best tasks
            std::vector<int> indices(num_tasks);
            for (int i = 0; i < num_agents; ++i) {
                std::iota(indices.begin(), indices.end(), 0); // Fill indices with 0, 1, ..., cols-1
                
                std::partial_sort(indices.begin(), indices.begin() + number_of_pairs_to_consider, indices.end(), 
                    [&cost_matrix, i](int a, int b) {
                        return cost_matrix[i][a] < cost_matrix[i][b];
                    });
                
                // Set the lowest k indices to 1 in the binary matrix
                for (int j = 0; j < number_of_pairs_to_consider; ++j) {
                    activation_matrix[i][indices[j]] = 1;
                }
            }
            // iterate tasks and find their number_of_pairs_to_consider best agents
            indices.resize(num_agents);
            for (int j = 0; j < num_tasks; ++j) {
                std::iota(indices.begin(), indices.end(), 0);

                std::partial_sort(indices.begin(), indices.begin() + number_of_pairs_to_consider, indices.end(),
                    [&cost_matrix, j](int a, int b) {
                        return cost_matrix[a][j] < cost_matrix[b][j];
                    });

                // Set the lowest k indices to 1 in the binary matrix
                for (int i = 0; i < number_of_pairs_to_consider; ++i) {
                    activation_matrix[indices[i]][j] = 1;
                }
            }
        }

        #ifndef NDEBUG
        passed = std::chrono::high_resolution_clock::now() - now;
        std::cout << "Time activation matrix: " << passed.count() << " seconds" << std::endl;
        now = std::chrono::high_resolution_clock::now();
        #endif

        // Create the solver
        std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
        if (!solver) {
            std::cerr << "SCIP solver not available." << std::endl;
        }

        // Decision variables: x[i][j] = 1 if agent i is assigned to task j, else 0
        std::unordered_map<int, std::unordered_map<int, const MPVariable*>> x; 
        for (int i = 0; i < num_agents; ++i) {
            for (int j = 0; j < num_tasks; ++j) {
                if (activation_matrix[i][j] == 1){
                    x[i][j] = solver->MakeNumVar(0.0, 1.0, "x_" + std::to_string(i) + "_" + std::to_string(j));
                }
            }
        }

        // Objective function: Minimize the total cost
        MPObjective* const objective = solver->MutableObjective();
        for (int i = 0; i < num_agents; ++i) {
            for (int j = 0; j < num_tasks; ++j) {
                if (activation_matrix[i][j] == 1){
                    objective->SetCoefficient(x[i][j], cost_matrix[i][j]);
                }
            }
        }
        objective->SetMinimization();


        // tasks >= agents: all agents get one task
        if (num_tasks >= num_agents){
            for (int i = 0; i < num_agents; ++i) {
                LinearExpr agent_constraint;
                for (int j = 0; j < num_tasks; ++j) {
                    if (activation_matrix[i][j] == 1){
                        agent_constraint += x[i][j];
                    }
                }
                solver->MakeRowConstraint(agent_constraint == 1);
            }
            for (int j = 0; j < num_tasks; ++j) {
                LinearExpr task_constraint;
                for (int i = 0; i < num_agents; ++i) {
                    if (activation_matrix[i][j] == 1){
                        task_constraint += x[i][j];
                    }
                }
                solver->MakeRowConstraint(task_constraint <= 1);
            }
        }
        //tasks < agents: all tasks get one agent
        else {
            for (int j = 0; j < num_tasks; ++j) {
                LinearExpr task_constraint;
                for (int i = 0; i < num_agents; ++i) {
                    if (activation_matrix[i][j] == 1){
                        task_constraint += x[i][j];
                    }
                }
                solver->MakeRowConstraint(task_constraint == 1);
            }

            // tasks >= emtpy agents (aka have no task assigned at all): all empty agents get one task
            int empty_agents = std::count(env->curr_task_schedule.begin(), env->curr_task_schedule.end(), -1);
            if (num_tasks >= empty_agents){
                for (int i = 0; i < num_agents; ++i) {
                    LinearExpr agent_constraint;
                    for (int j = 0; j < num_tasks; ++j) {
                        if (activation_matrix[i][j] == 1){
                            agent_constraint += x[i][j];
                        }
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
                        if (activation_matrix[i][j] == 1){
                            agent_constraint += x[i][j];
                        }
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

        #ifndef NDEBUG
        passed = std::chrono::high_resolution_clock::now() - now;
        std::cout << "Time create solver, objective, constrains, etc. : " << passed.count() << " seconds" << std::endl;
        now = std::chrono::high_resolution_clock::now();
        #endif
        

        auto startElapsed = std::chrono::high_resolution_clock::now();

        const MPSolver::ResultStatus result_status = solver->Solve();

        //solver time
        #ifndef NDEBUG
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - startElapsed;
        std::cout << "Time Solver: " << elapsed.count() << " seconds" << std::endl;

       
        if (result_status != MPSolver::OPTIMAL) {
            std::cerr << "The problem does not have an optimal solution!" << std::endl;
        }
        #endif

        std::vector<int> tasks_to_remove;

        for (int i = 0; i < num_agents; ++i){
            if (env->curr_task_schedule[i] == -1){
                bool solutionFound = false;
                for (int j = 0; j < num_tasks; ++j){
                    if (activation_matrix[i][j] == 1){
                        if (x[i][j]->solution_value() > 0.5){
                            proposed_schedule[i] = free_tasks[j];
                            tasks_to_remove.push_back(j);
                            free_agents.erase(i);
                            solutionFound = true;
                            break;
                        }
                    }
                }
                #ifndef NDEBUG
                if (!solutionFound){
                    std::cout << "Keine Loesung gefunden du Nuss XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
                }
                #endif
            }
        }


        //remove tasks in decending order since each time vector changes
        std::sort(tasks_to_remove.rbegin(), tasks_to_remove.rend());
        for (int index : tasks_to_remove) {
            free_tasks.erase(free_tasks.begin() + index);
        }

        #ifndef NDEBUG
        passed = std::chrono::high_resolution_clock::now() - now;
        std::cout << "Time rest : " << passed.count() << " seconds" << std::endl;
        now = std::chrono::high_resolution_clock::now();;
        #endif
    }



    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}

int get_h_limited(SharedEnvironment* env, int source, int target){
	// int sourceR = source / env->cols;
	// int sourceC = source % env->cols;
	// int targetR = target / env->cols;
	// int targetC = target % env->cols;

	// int dist = static_cast<int>(1.3 * std::abs(sourceR - targetR) + std::abs(sourceC - targetC));

	int dist = static_cast<int>(1.3 * std::abs((source / env->cols) - (target / env->cols)) + std::abs((source % env->cols) - (target % env->cols)));

	if (dist > 20) {
		return dist;
	}




	if (DefaultPlanner::global_heuristictable.empty()){
		DefaultPlanner::init_heuristics(env);
	}

	if (DefaultPlanner::global_heuristictable.at(target).empty()){
		DefaultPlanner::init_heuristic(DefaultPlanner::global_heuristictable.at(target),env,target);
	}

	return DefaultPlanner::get_heuristic(DefaultPlanner::global_heuristictable.at(target), env, source, &DefaultPlanner::global_neighbors);
}


}//end namespace
