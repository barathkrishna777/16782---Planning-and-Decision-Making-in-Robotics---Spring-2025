#include "utils.hpp"
#include "planner.hpp"

list<GroundedAction> planner(Env* env)
{
    ///TODO: INSERT YOUR PLANNER HERE
    auto start = std::chrono::high_resolution_clock::now();
    
    unordered_set<string> symbols = env->get_symbols();
    unordered_set<Action, ActionHasher> actions = env->get_actions();
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions = env->get_initial_conditions();
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions = env->get_goal_conditions();

    symbolic_planner sp(symbols, initial_conditions, goal_conditions, actions);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << "Time taken to create the planner: " << duration1.count() << " milliseconds" << endl;
    
    start = std::chrono::high_resolution_clock::now();
    sp.build_action_map();
    stop = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << "Time taken to build the action map: " << duration2.count() << " milliseconds" << endl;

    start = std::chrono::high_resolution_clock::now();
    list<GroundedAction> plan = sp.search();
    stop = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << "Time taken to search for the plan: " << duration3.count() << " milliseconds" << endl;

    std::cout << "Total time taken: " << duration1.count() + duration2.count() + duration3.count() << " milliseconds" << std::endl;
    std::cout << "Size of graph: " << sp.get_graph_size() << std::endl;
    return plan;
}

int main(int argc, char* argv[])
{ 
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}