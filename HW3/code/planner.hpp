#include "utils.hpp"


class symbolic_planner {
private:
    std::unordered_set<string> symbols;
    std::unordered_set<Condition, ConditionHasher, ConditionComparator> initial_conditions, goal_conditions, state_conditions;
    std::unordered_set<Action, ActionHasher> actions;

    struct stateHasher;

    struct state {
        std::unordered_set<Condition, ConditionHasher, ConditionComparator> condition;
        int g = std::numeric_limits<int>::max();
        int h = std::numeric_limits<int>::max();
        bool closed = false;
    
        state(std::unordered_set<Condition, ConditionHasher, ConditionComparator> condition_) : condition(condition_) {}
    
        bool operator==(const state& rhs) const {
            return this->condition == rhs.condition;
        }
    };

    struct stateHasher {
        std::size_t operator()(const std::shared_ptr<state>& s) const {
            std::size_t hash_value = 0;
            std::hash<std::size_t> hasher;
            for (const auto& cond : s->condition) {
                hash_value = hash_value * 31 + hasher(ConditionHasher{}(cond));
            }
            return hash_value;
        }        
    };

    struct stateComparator {
        bool operator()(const state& s1, const state& s2) const {
            return s1 == s2;
        }
        bool operator()(const std::shared_ptr<state>& s1, const std::shared_ptr<state>& s2) const {
            return *s1 == *s2;
        }
    };

    std::unordered_set<std::shared_ptr<state>, stateHasher, stateComparator> graph;
    std::unordered_map<std::shared_ptr<state>, std::pair<std::shared_ptr<Action>, std::shared_ptr<state>>> parent_map;
    std::unordered_map<Action, GroundedAction, ActionHasher, ActionComparator> action_map;

public:
    symbolic_planner(const std::unordered_set<std::string>& symbols, 
                     const std::unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& initial_conditions,
                     const std::unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& goal_conditions,
                     const std::unordered_set<Action, ActionHasher>& actions) {

        this->symbols = symbols;

        for (const auto& action : actions) {
            generateActions(action, symbols);
        }

        for (const auto& condition : initial_conditions) {
            Condition new_condition(condition.get_predicate(), condition.get_arg_values(), condition.get_truth());
            this->initial_conditions.insert(new_condition);
            this->state_conditions.insert(new_condition);
        }

        for (const auto& condition : goal_conditions) {
            Condition new_condition(condition.get_predicate(), condition.get_arg_values(), condition.get_truth());
            this->goal_conditions.insert(new_condition);
        }
    }

    int get_graph_size() const {
        return graph.size();
    }

    void generateActions(const Action& template_action, const std::unordered_set<std::string>& symbols) {
        std::vector<std::string> symbol_vec(symbols.begin(), symbols.end());
        int arg_count = template_action.get_args().size();
    
        if (arg_count > symbol_vec.size()) {
            std::cerr << "Error: Not enough symbols to generate distinct arguments." << std::endl;
            return;
        }
    
        std::vector<bool> select(symbol_vec.size(), false);
        std::fill(select.begin(), select.begin() + arg_count, true);
    
        do {
            std::vector<std::string> selected_args;
            for (size_t i = 0; i < symbol_vec.size(); ++i) {
                if (select[i]) {
                    selected_args.push_back(symbol_vec[i]);
                }
            }
    
            std::sort(selected_args.begin(), selected_args.end());
            do {
                std::list<std::string> args(selected_args.begin(), selected_args.end());
    
                std::map<std::string, std::string> argument_mapping;
                auto template_args = template_action.get_args();
    
                if (template_args.size() != args.size()) {
                    std::cerr << "Error: Argument size mismatch for action " << template_action.get_name() << std::endl;
                    continue;
                }
    
                auto it_template = template_args.begin();
                auto it_selected = args.begin();
                while (it_template != template_args.end() && it_selected != args.end()) {
                    argument_mapping[*it_template] = *it_selected;
                    ++it_template;
                    ++it_selected;
                }
    
                for (const auto& effect : template_action.get_effects()) {
                    for (const auto& arg : effect.get_args()) {
                        if (argument_mapping.find(arg) == argument_mapping.end()) {
                            if (symbols.find(arg) != symbols.end()) {
                                argument_mapping[arg] = arg;
                            } else {
                                std::cerr << "Error: Symbol '" << arg << "' not found in declared symbols." << std::endl;
                                return;
                            }
                        }
                    }
                }
    
                for (const auto& precond : template_action.get_preconditions()) {
                    for (const auto& arg : precond.get_args()) {
                        if (argument_mapping.find(arg) == argument_mapping.end()) {
                            if (symbols.find(arg) != symbols.end()) {
                                argument_mapping[arg] = arg;
                            } else {
                                std::cerr << "Error: Symbol '" << arg << "' not found in declared symbols." << std::endl;
                                return;
                            }
                        }
                    }
                }
    
                std::unordered_set<Condition, ConditionHasher, ConditionComparator> new_preconditions;
                for (const auto& cond : template_action.get_preconditions()) {
                    std::list<std::string> new_args;
                    for (const auto& arg : cond.get_args()) {
                        new_args.push_back(argument_mapping.at(arg));
                    }
                    new_preconditions.insert(Condition(cond.get_predicate(), new_args, cond.get_truth()));
                }
    
                std::unordered_set<Condition, ConditionHasher, ConditionComparator> new_effects;
                for (const auto& cond : template_action.get_effects()) {
                    std::list<std::string> new_args;
                    for (const auto& arg : cond.get_args()) {
                        new_args.push_back(argument_mapping.at(arg));
                    }
                    new_effects.insert(Condition(cond.get_predicate(), new_args, cond.get_truth()));
                }
    
                GroundedAction grounded_action(template_action.get_name(), args);
                Action new_action = Action(template_action.get_name(), args, new_preconditions, new_effects);
                action_map.emplace(new_action, grounded_action);
    
                if (this->actions.find(new_action) != this->actions.end()) {
                    std::cerr << "Warning: Duplicate action detected: " << new_action.get_name() << std::endl;
                }
    
                this->actions.insert(new_action);
    
            } while (std::next_permutation(selected_args.begin(), selected_args.end()));
        } while (std::prev_permutation(select.begin(), select.end()));
    }

    bool isGoal(const std::unordered_set<Condition, ConditionHasher, ConditionComparator>& conditions) {
        
        for (const auto& goal_condition : this->goal_conditions) {
            if (conditions.find(goal_condition) == conditions.end()) {
                return false;
            }
        }
        return true;
    }

    int heuristic(const std::unordered_set<Condition, ConditionHasher, ConditionComparator>& conditions) {
        int missing_conditions = 0;
        for (const auto& goal_condition : this->goal_conditions) {
            if (conditions.find(goal_condition) == conditions.end()) {
                missing_conditions++;
            }
        }
        int scaling_factor = std::max(1, static_cast<int>(std::ceil(goal_conditions.size() / 3.0)));  
        return missing_conditions * scaling_factor;
    }
    

    std::vector<Action> get_applicable_actions(const std::unordered_set<Condition, ConditionHasher, ConditionComparator>& conditions) {
        std::vector<Action> applicable_actions;
    
        for (const auto& action : this->actions) {
            bool valid_action = true;
    
            for (const auto& precondition : action.get_preconditions()) {
                if (conditions.find(precondition) == conditions.end()) {
                    valid_action = false;
                    break;
                }
            }
    
            if (valid_action) {
                applicable_actions.push_back(action);
            }
        }
        return applicable_actions;
    }

    void build_action_map() {
        using StatePtr = std::shared_ptr<state>;
    
        auto cmp = [](const StatePtr& lhs, const StatePtr& rhs) {
            return (lhs->g + lhs->h) > (rhs->g + rhs->h);
        };
        
        std::priority_queue<StatePtr, std::vector<StatePtr>, decltype(cmp)> open(cmp);
        graph.clear();
        std::unordered_set<StatePtr, stateHasher, stateComparator> closed;
        
        auto initial_state = std::make_shared<state>(this->initial_conditions);
        initial_state->g = 0;
        initial_state->h = heuristic(initial_state->condition);
        
        open.push(initial_state);
        graph.insert(initial_state);
        
        while (!open.empty()) {
            StatePtr current = open.top();
            open.pop();
    
            if (isGoal(current->condition)) {
                std::cout << "Goal found! Plan length: " << current->g << std::endl;
                return;
            }
    
            if (closed.find(current) != closed.end()) continue;
            closed.insert(current);

            for (const auto& action : get_applicable_actions(current->condition)) {

                auto new_conditions = current->condition;
                auto relaxed_conditions = current->condition;
    
                for (const auto& effect : action.get_effects()) {
                    if (!effect.get_truth()) {
                        new_conditions.erase(Condition(effect.get_predicate(), effect.get_args(), true));
                    } else {
                        new_conditions.insert(effect);
                        relaxed_conditions.insert(effect);
                    }
                }
    
                if (new_conditions == current->condition) continue;
    
                auto new_state = std::make_shared<state>(new_conditions);
                new_state->g = current->g + 1;
                new_state->h = heuristic(relaxed_conditions);
    
                auto it = graph.find(new_state);
    
                if (it != graph.end()) {
                    if ((*it)->g > new_state->g) {
                        (*it)->g = new_state->g;
                        (*it)->h = heuristic(relaxed_conditions);
                        parent_map[*it] = {std::make_shared<Action>(action), current};
                        open.push(*it);
                    }
                } else {
                    graph.insert(new_state);
                    open.push(new_state);
                    parent_map[new_state] = {std::make_shared<Action>(action), current};
                }
            }
        }

        std::cout << "No plan found!" << std::endl;
    }
    

    std::list<GroundedAction> search() {
        std::list<GroundedAction> plan;
        
        if (graph.empty()) {
            std::cout << "No plan found: empty graph!" << std::endl;
            return plan;
        }
        // assign current as the state in the graph with the same conditions as the goal state
        auto it = std::find_if(graph.begin(), graph.end(), [&](const std::shared_ptr<state>& s) {
            return isGoal(s->condition);
        });
        std::shared_ptr<state> current = nullptr;
        if (it != graph.end()) {
            current = *it;
        } else {
            std::cout << "No plan found: goal state not in graph!" << std::endl;
            return plan;
        }
    
        while (parent_map.find(current) != parent_map.end()) {
            auto [action_ptr, parent_state] = parent_map[current];
    
            if (action_map.find(*action_ptr) != action_map.end()) {
                plan.push_front(action_map[*action_ptr]); 
            } else {
                std::cerr << "Error: Grounded action not found!" << std::endl;
            }
    
            current = parent_state;
        }
    
        return plan;
    }
    
};