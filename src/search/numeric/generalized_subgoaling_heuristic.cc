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

        vector<int> temp_conditions;
        
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

                    for (auto c : possible_achievers[gr])
                        temp_conditions.push_back(c);
                }

                for (auto c : temp_conditions) {
                    if (!closed[c]) {
                        update_constraints(c, state);
                        lps[c]->solve();
                        double result = lps[c]->get_infinity();
                        if (lps[c]->has_optimal_solution()) {
                            double epsilon = 0.01;
                            double objective_value = lps[c]->get_objective_value();
                            result = ceil(objective_value - epsilon);
                            update_cost_if_necessary(c, q, result);
                        }
                    }
                }
            }
        }

        return dist[preconditions_to_id.size() - 1];
    }

    void GeneralizedSubgoalingHeuristic::update_constraints(int preconditions_id, const State &state) {
        auto lp = lps[preconditions_id];
        for (auto &entry : conjnct_to_constraint_index[preconditions_id]) {
            int c = entry.first;
            LinearNumericCondition &num_values = numeric_task.get_condition(c);
            double lower_bound = - num_values.constant + numeric_task.get_epsilon(c);
            for (size_t i = 0; i < numeric_task.get_n_numeric_variables(); i++){
                int id_num = numeric_task.get_numeric_variable(i).id_abstract_task;
                lower_bound -= state.nval(id_num) * num_values.coefficients[i];
            }
            lp->set_constraint_lower_bound(entry.second, lower_bound);
        }
        for (auto &entry : action_to_variable_index[preconditions_id]) {
            if (active_actions[entry.first]) {
                lp->set_constraint_upper_bound(entry.second, lp->get_infinity());
            } else {
                lp->set_constraint_upper_bound(entry.second, 0.0);
            }
        }
    }

    void GeneralizedSubgoalingHeuristic::update_cost_if_necessary(int cond, HeapQueue<int> &q, double current_cost) {
        if (current_cost == lps[cond]->get_infinity())
            return;
        if (open[cond]) {
            if (dist[cond] > current_cost) {
                q.push(current_cost, cond);
                dist[cond] = current_cost;
            }
        } else {
            dist[cond] = current_cost;
            open[cond] = true;
            q.push(current_cost, cond);
        }
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
        size_t n_propositions = numeric_task.get_n_propositions();
        effect_of.assign(n_propositions,set<int>());
        auto ops = task_proxy.get_operators();
        for (size_t op_id = 0; op_id < task_proxy.get_operators().size(); ++op_id){
            for (EffectProxy effect_proxy : ops[op_id].get_effects()) {
                FactProxy fact = effect_proxy.get_fact();
                int var = fact.get_variable().get_id();
                int val = fact.get_value();
                int c = numeric_task.get_proposition(var,val);
                effect_of[c].insert(op_id);
            }
        }

        possible_achievers.assign(task_proxy.get_operators().size(),set<int>());
        possible_achievers_inverted.assign(numeric_task.get_n_conditions(),set<int>());
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
                
                if (cumulative_effect>0) {
                    possible_achievers[op_id].insert(nc_id);
                    possible_achievers_inverted[nc_id].insert(op_id);
                }
            }
        }

        possible_preconditions_achievers.assign(task_proxy.get_operators().size(),set<int>());
        possible_preconditions_achievers_inverted.assign(preconditions_to_id.size(),set<int>());
        for (size_t op_id = 0; op_id < task_proxy.get_operators().size(); ++op_id){
            OperatorProxy gr = task_proxy.get_operators()[op_id];
            for (auto &entry : preconditions_to_id) {
                int condition_id = entry.second;
                if (action_to_preconditions_id[op_id] != condition_id) {
                    for (int c : entry.first) {
                        if (c < n_propositions) {
                            if (effect_of[c].find(op_id) != effect_of[c].end()) {
                                possible_preconditions_achievers[op_id].insert(condition_id);
                                possible_preconditions_achievers_inverted[condition_id].insert(op_id);
                                break;
                            }
                        } else if (possible_achievers[op_id].find(c - n_propositions) != possible_achievers[op_id].end()) {
                            possible_preconditions_achievers[op_id].insert(condition_id);
                            possible_preconditions_achievers_inverted[condition_id].insert(op_id);
                            break;
                        }
                    }
                }
            }
        }
        possible_preconditions_achievers_inverted[preconditions_to_id.size() - 1].insert(task_proxy.get_operators().size());
    }

    void GeneralizedSubgoalingHeuristic::generate_linear_programs() {
        lps.assign(preconditions_to_id.size(),nullptr);
        action_to_variable_index.assign(preconditions_to_id.size(),unordered_map<int, int>());
        conjnct_to_constraint_index.assign(preconditions_to_id.size(),unordered_map<int, int>());
        auto ops = task_proxy.get_operators();
        size_t n_propositions = numeric_task.get_n_propositions();
        for (auto &entry : preconditions_to_id) {
            int c = entry.second;
            auto lp = make_shared<lp::LPSolver>();
            vector<lp::LPVariable> variables;
            vector<lp::LPConstraint> constraints;
            for (int conjunct : entry.first) {
                lp::LPConstraint constraint(-lp->get_infinity(),lp->get_infinity());
                if (conjunct < n_propositions) {
                    for (int gr : effect_of[conjunct]) {
                        if (action_to_variable_index[c].find(gr) == action_to_variable_index[c].end()) {
                            int cost = ops[gr].get_cost();
                            lp::LPVariable m_a(0.0, 0.0, cost, "m_a_" + ops[gr].get_name());
                            action_to_variable_index[c][gr] = variables.size();
                            variables.push_back(m_a);
                        }
                        constraint.insert(action_to_variable_index[c][gr], 1.0);
                    }
                } else {
                    int numeric_conjunct = conjunct - n_propositions;
                    for (int gr : possible_achievers_inverted[conjunct]) {
                        if (action_to_variable_index[c].find(gr) == action_to_variable_index[c].end()) {
                            int cost = ops[gr].get_cost();
                            lp::LPVariable m_a(0.0, 0.0, cost, "m_a_" + ops[gr].get_name());
                            action_to_variable_index[c][gr] = variables.size();
                            variables.push_back(m_a);
                        }
                        constraint.insert(action_to_variable_index[c][gr], net_effects[gr][numeric_conjunct]);
                    }
                }
                conjnct_to_constraint_index[c][conjunct] = constraints.size();
                constraints.push_back(constraint);
            }
            lp->load_problem(lp::LPObjectiveSense::MINIMIZE, variables, constraints);
            lps[c] = lp;
        }
    }
    
    GeneralizedSubgoalingHeuristic::GeneralizedSubgoalingHeuristic(const options::Options& options)
    : Heuristic(options)
    {
        numeric_task = NumericTaskProxy(task_proxy);
        max_float = 999999;
        generate_possible_achievers();
        generate_preconditions();
        generate_linear_programs();
    }
    
    GeneralizedSubgoalingHeuristic::~GeneralizedSubgoalingHeuristic() {
    }
    
    static Plugin<Heuristic> _plugin("hgen", _parse);
    
}


