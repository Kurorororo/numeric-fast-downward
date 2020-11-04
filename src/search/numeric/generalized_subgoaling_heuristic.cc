#include "generalized_subgoaling_heuristic.h"
#include "../globals.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"

using namespace std;
using namespace numeric_helper;


namespace generalized_subgoaling_heuristic {
    
    
    static Heuristic *_parse(OptionParser &parser) {
        parser.document_synopsis("Generalized subgoaling heuristic", "");
        parser.document_language_support("action costs", "supported");
        parser.document_language_support("conditional effects", "supported");
        parser.document_language_support("numeric", "supported");
        parser.document_language_support(
                                         "axioms",
                                         "supported (in the sense that the planner won't complain -- "
                                         "handling of axioms might be very stupid "
                                         "and even render the heuristic unsafe)");
        parser.document_property("admissible", "yes for tasks without axioms");
        parser.document_property("consistent", "yes for tasks without axioms");
        parser.document_property("safe", "yes for tasks without axioms");
        parser.document_property("preferred operators", "no");

        Heuristic::add_options_to_parser(parser);
        Options opts = parser.parse();
        
        if (parser.dry_run())
            return 0;
        else
            return new GeneralizedSubgoalingHeuristic(opts);
    }
    
    void GeneralizedSubgoalingHeuristic::initialize() {
        if(DEBUG) cout << "Initializing genealized subgoaling heuristic..." << endl;
    }
    
    ap_float GeneralizedSubgoalingHeuristic::compute_heuristic(
                                              const GlobalState& global_state) {
        State state = convert_global_state(global_state);
        OperatorsProxy ops = task_proxy.get_operators();

        HeapQueue<int> q; // cannot use adaptive queue when costs are non integer
        open.assign(preconditions_to_id.size(), false);
        closed.assign(preconditions_to_id.size(), false);
        dist.assign(preconditions_to_id.size(), max_float);
        active_actions.assign(ops.size(), false);
        
        //update initial state
        size_t n_propositions = numeric_task.get_n_propositions();
        vector<int> initial_conditions;
        for (size_t var = 0; var < numeric_task.get_n_vars(); ++var) {
            int val = state[var].get_value();
            int condition = numeric_task.get_proposition(var,val);
            initial_conditions.push_back(condition);
        }
        for (size_t var = 0; var < numeric_task.get_n_conditions(); ++var) {
            LinearNumericCondition &num_values = numeric_task.get_condition(var);
            double lower_bound = - num_values.constant + numeric_task.get_epsilon(var);
            for (size_t i = 0; i < numeric_task.get_n_numeric_variables(); i++){
                int id_num = numeric_task.get_numeric_variable(i).id_abstract_task;
                lower_bound -= (state.nval(id_num) * num_values.coefficients[i]);
            }
            if (lower_bound <= 0){
                initial_conditions.push_back(n_propositions+var);
            }
        }
        sort(initial_conditions.begin(), initial_conditions.end());

        for (auto &entry : preconditions_to_id) {
            size_t i = 0;
            size_t j = 0;
            bool satisfied = false;
            while (i < entry.first.size() && j < initial_conditions.size()) {
                if (entry.first[i] < initial_conditions[j]) {
                    break;
                } else if (entry.first[i] > initial_conditions[j]) {
                    ++j;
                } else {
                    ++j;
                    if (++i == entry.first.size())
                        satisfied = true;
                }
            }
            if (satisfied) {
                dist[entry.second] = 0;
                q.push(0, dist[entry.second]);
                open[entry.second] = true;
                closed[entry.second] = true;
            }
        }
        
        // explore
        while(!q.empty()){
            ap_float first = -1;

            while (!q.empty()){
                pair<ap_float, int> top_pair = q.pop();
                int condition_id = top_pair.second;
                if (first < 0.0) {
                    first = top_pair.first;
                } else if (first < dist[condition_id]) {
                    open[condition_id] = true;
                    q.push(top_pair.first, condition_id);
                    first = -1.0;
                }

                closed[condition_id] = true;

                // check goal
                if (condition_id == preconditions_to_id.size() - 1)
                    return dist[condition_id];

                set<int> &actions = condition_to_action[condition_id];

                for (auto gr : actions) {
                    active_actions[gr] = true;
                }
            }
        }

        return dist[preconditions_to_id.size() - 1];
    }
    
    void GeneralizedSubgoalingHeuristic::generate_preconditions(){
        size_t n_propositions = numeric_task.get_n_propositions();
        action_to_preconditions_id.assign(task_proxy.get_operators().size()+1,-1);
        int num_preconditions = 0;
        for (size_t op_id = 0; op_id < task_proxy.get_operators().size(); ++op_id){
            vector<int> preconditions;
            for (int c : numeric_task.get_action_pre_list(op_id))
                preconditions.push_back(c);
            for (int i : numeric_task.get_action_num_list(op_id)) {
                for (int c : numeric_task.get_numeric_conditions_id(i))
                    preconditions.push_back(c+n_propositions);
            }
            sort(preconditions.begin(), preconditions.end());

            if (preconditions_to_id.find(preconditions) == preconditions_to_id.end()){
                preconditions_to_id[preconditions] = num_preconditions++;
                condition_to_action.push_back(set<int>());
            }

            int preconditions_id = preconditions_to_id[preconditions];
            condition_to_action[preconditions_id].insert(op_id);
            action_to_preconditions_id[op_id] = preconditions_id;
        }

        // add goal operator
        vector<int> goal_preconditions;
        for (size_t id_goal = 0; id_goal < task_proxy.get_goals().size(); ++id_goal) {
            FactProxy goal = task_proxy.get_goals()[id_goal];
            int var = goal.get_variable().get_id();
            int val = goal.get_value();
            int c = numeric_task.get_proposition(var,val);
            if (!numeric_task.numeric_goals_empty(id_goal)) continue; // this is a numeric goal
            goal_preconditions.push_back(c);
        }
        for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
            list<int> goals = numeric_task.get_numeric_goals(id_goal);
            for (int c : goals)
                goal_preconditions.push_back(c+n_propositions);
        }
        sort(goal_preconditions.begin(), goal_preconditions.end());
        preconditions_to_id[goal_preconditions] = num_preconditions;
        condition_to_action.push_back(set<int>());
        condition_to_action[num_preconditions].insert(task_proxy.get_operators().size());
        action_to_preconditions_id[task_proxy.get_operators().size()] = num_preconditions;
    }
    
    void GeneralizedSubgoalingHeuristic::generate_possible_achievers(){
        possible_achievers.assign(task_proxy.get_operators().size(),set<int>());
        net_effects.assign(task_proxy.get_operators().size(),vector<double>(numeric_task.get_n_conditions(),0.));
        size_t n_numeric_variables = numeric_task.get_n_numeric_variables();
        for (size_t nc_id = 0; nc_id < numeric_task.get_n_conditions(); ++nc_id){
            for (size_t op_id = 0; op_id < task_proxy.get_operators().size(); ++op_id){
                LinearNumericCondition &nc = numeric_task.get_condition(nc_id);
                double cumulative_effect = 0;//nc.constant;
                for (size_t v = 0; v < n_numeric_variables; ++v){
                    cumulative_effect+=(nc.coefficients[v]*numeric_task.get_action_eff_list(op_id)[v]);
                }
                net_effects[op_id][nc_id] = cumulative_effect;
                
                if (cumulative_effect>0)
                    possible_achievers[op_id].insert(nc_id);
            }
        }

        possible_preconditions_achievers.assign(task_proxy.get_operators().size(),set<int>());
        size_t n_propositions = numeric_task.get_n_propositions();
        for (size_t op_id = 0; op_id < task_proxy.get_operators().size(); ++op_id){
            OperatorProxy gr = task_proxy.get_operators()[op_id];
            for (auto &entry : preconditions_to_id) {
                int condition_id = entry.second;
                if (action_to_preconditions_id[op_id] != condition_id) {
                    for (int c : entry.first) {
                        if (c < n_propositions) {
                            for (EffectProxy effect_proxy : gr.get_effects()) {
                                FactProxy effect = effect_proxy.get_fact();
                                int var = effect.get_variable().get_id();
                                int post = effect.get_value();
                                int proposition = numeric_task.get_proposition(var,post);

                                if (c == proposition) {
                                    possible_preconditions_achievers[op_id].insert(condition_id);
                                    break;
                                }
                            }
                        } else if (possible_achievers[op_id].find(c - n_propositions) != possible_achievers[op_id].end()) {
                            possible_preconditions_achievers[op_id].insert(condition_id);
                            break;
                        }
                    }
                }
            }
        }
    }
    
    GeneralizedSubgoalingHeuristic::GeneralizedSubgoalingHeuristic(const options::Options& options)
    : Heuristic(options)
    {
        numeric_task = NumericTaskProxy(task_proxy);
        max_float = 999999;
        generate_possible_achievers();
        generate_preconditions();

    }
    
    GeneralizedSubgoalingHeuristic::~GeneralizedSubgoalingHeuristic() {
    }
    
    static Plugin<Heuristic> _plugin("hgen", _parse);
    
}


