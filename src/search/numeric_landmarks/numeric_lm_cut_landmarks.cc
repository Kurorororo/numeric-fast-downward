#include "numeric_lm_cut_landmarks.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <unordered_map>
#include <unordered_set>

#include "../numeric_operator_counting/numeric_helper.h"


using namespace std;
using namespace numeric_helper;

bool debug = false;
double precision = 0.000001;
double epsilon = 0.001;

namespace numeric_lm_cut_heuristic {
    // construction and destruction
    LandmarkCutLandmarks::LandmarkCutLandmarks(const TaskProxy &task_proxy, bool ceiling_less_than_one, bool ignore_numeric,
                                               bool use_random_pcf, bool use_irmax, bool disable_ma, bool use_linear_effects,
                                               bool use_second_order_simple)
        : numeric_task(NumericTaskProxy(task_proxy, false, use_linear_effects)),
          n_infinite_operators(0),
          n_second_order_simple_operators(0),
          ceiling_less_than_one(ceiling_less_than_one),
          ignore_numeric_conditions(ignore_numeric),
          use_random_pcf(use_random_pcf),
          use_irmax(use_irmax),
          disable_ma(disable_ma),
          use_linear_effects(use_linear_effects),
          use_second_order_simple(use_second_order_simple) {
        //verify_no_axioms(task_proxy);
        verify_no_conditional_effects(task_proxy);
        // Build propositions.
        num_propositions = 2; // artificial goal and artificial precondition
        artificial_precondition.is_numeric_condition = false;
        artificial_precondition.name = "artificial";
        artificial_goal.is_numeric_condition = false;
        artificial_goal.name = "goal";
        VariablesProxy variables = task_proxy.get_variables();
        n_var = variables.size();
        propositions.resize(n_var + numeric_task.get_n_numeric_conditions());
        for (FactProxy fact : variables.get_facts()) {
            int var_id = fact.get_variable().get_id();
            RelaxedProposition prop;
            prop.is_numeric_condition = false;//numeric_task.is_numeric_axiom(fact.get_variable().get_id());
            prop.id_numeric_condition = -1;
            prop.name = fact.get_name();
            propositions[var_id].push_back(prop);
            ++num_propositions;
        }

        if (!ignore_numeric_conditions) {
            // add numeric conditions
            for (int i = 0; i < numeric_task.get_n_numeric_conditions(); i++){
                //LinearNumericCondition &num_values = numeric_task.get_condition(i);
                int var_id = n_var + i;
                RelaxedProposition prop;
                prop.is_numeric_condition = true;
                prop.id_numeric_condition = i;
                stringstream name;
                LinearNumericCondition lnc = numeric_task.get_condition(i);
                name << "numeric (" << lnc << ")";
                conditions.push_back(std::move(lnc));
                epsilons.push_back(numeric_task.get_epsilon(i));
                prop.name =  name.str();
                propositions[var_id].push_back(prop);
                ++num_propositions;
                //cout << "adding numeric precondition " << num_values << " : " << num_propositions << " " << var_id << endl;
            }
        }

        // Build relaxed operators for operators and axioms.
        for (OperatorProxy op : task_proxy.get_operators())
            build_relaxed_operator(op);

        if (!ignore_numeric && use_linear_effects) {
            for (OperatorProxy op : task_proxy.get_operators())
                build_linear_operators(task_proxy, op);
            std::cout << "Infinite operators: " << n_infinite_operators << std::endl;
            std::cout << "Second-order simple operators: " << n_second_order_simple_operators << std::endl;

            if (n_second_order_simple_operators == 0) use_second_order_simple = false;
        }

        original_to_relaxed_operators.resize(task_proxy.get_operators().size(), vector<RelaxedOperator*>());
        
        if (!ignore_numeric) build_numeric_effects();

        // Simplify relaxed operators.
        // simplify();
        /* TODO: Put this back in and test if it makes sense,
         but only after trying out whether and how much the change to
         unary operators hurts. */
        
        // Build artificial goal proposition and operator.
        vector<RelaxedProposition *> goal_op_pre, goal_op_eff;
        for (FactProxy goal : task_proxy.get_goals()) {
            if(!numeric_task.is_numeric_axiom(goal.get_variable().get_id())){
                goal_op_pre.push_back(get_proposition(goal));
            }
        }

        if (!ignore_numeric_conditions) {
            // add numeric goal conditions
            for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
                list<int> numeric_goals = numeric_task.get_numeric_goals(id_goal);
                if (numeric_goals.empty()) continue; // this is not a numeric goal
                for (int id_n_con : numeric_goals){
                    //LinearNumericCondition &num_values = numeric_task.get_condition(id_n_con);
                    goal_op_pre.push_back(get_proposition(id_n_con));
                    //cout << "Goal : " << num_values << " is a goal condition" << endl;
                }
            }
        }
        
        goal_op_eff.push_back(&artificial_goal);
        /* Use the invalid operator id -1 so accessing
         the artificial operator will generate an error. */
        string name_goal = "goal";
        add_relaxed_operator(move(goal_op_pre), move(goal_op_eff), -1, 0, name_goal);
        relaxed_operators.back().numeric_effects = std::vector<ap_float>(conditions.size(), 0);

        for (RelaxedOperator &relaxed_op : relaxed_operators) {
            if (relaxed_op.original_op_id_1 != -1)
                original_to_relaxed_operators[relaxed_op.original_op_id_1].push_back(&relaxed_op);
            if (relaxed_op.original_op_id_2 != -1)
                original_to_relaxed_operators[relaxed_op.original_op_id_2].push_back(&relaxed_op);
        }

        // Cross-reference relaxed operators.
        for (RelaxedOperator &op : relaxed_operators) {
            for (RelaxedProposition *pre : op.preconditions){
                pre->precondition_of.push_back(&op);
            }
            for (RelaxedProposition *eff : op.effects)
                eff->effect_of.push_back(&op);
        }
        cout << "ops " <<  task_proxy.get_operators().size() << ", prop: " << num_propositions << ", numeric conditions " <<  conditions.size() << endl;
    }
    
    LandmarkCutLandmarks::~LandmarkCutLandmarks() {
    }
    
    void LandmarkCutLandmarks::build_relaxed_operator(const OperatorProxy &op) {
        vector<RelaxedProposition *> precondition;
        vector<RelaxedProposition *> effects;
        for (FactProxy pre : op.get_preconditions()) {
            if(!numeric_task.is_numeric_axiom(pre.get_variable().get_id())){
                precondition.push_back(get_proposition(pre));
            }
        }

        if (!ignore_numeric_conditions) {
            // numeric precondition
            for (int pre : numeric_task.get_action_num_list(op.get_id())){
                for (int i : numeric_task.get_numeric_conditions_id(pre)){
                    precondition.push_back(get_proposition(i));
                    //LinearNumericCondition &num_values = numeric_task.get_condition(i);
                    //cout << "adding precondition " << num_values << " to action " << op.get_name() << endl;
                }
            }
        }
        
        for (EffectProxy eff : op.get_effects()) {
            // check if it's numeric axiom
            if(!numeric_task.is_numeric_axiom(eff.get_fact().get_variable().get_id())){
                effects.push_back(get_proposition(eff.get_fact()));
            }
        }

        string name = op.get_name();
        add_relaxed_operator(move(precondition), move(effects), op.get_id(), op.get_cost(), name);
    }

    void LandmarkCutLandmarks::add_relaxed_operator(vector<RelaxedProposition *> &&precondition,
                                                    vector<RelaxedProposition *> &&effects,
                                                    int op_id, ap_float base_cost, string &n) {
        int id = relaxed_operators.size();
        RelaxedOperator relaxed_op(id, move(precondition), move(effects), op_id, base_cost,n);
        if (relaxed_op.preconditions.empty())
            relaxed_op.preconditions.push_back(&artificial_precondition);
        relaxed_operators.push_back(relaxed_op);
    }

    void LandmarkCutLandmarks::build_linear_operators(const TaskProxy &task_proxy, const OperatorProxy &op_2) {
        if (numeric_task.get_action_n_linear_eff(op_2.get_id()) == 0) return;

        vector<RelaxedProposition *> precondition;
        for (FactProxy pre : op_2.get_preconditions()) {
            if(!numeric_task.is_numeric_axiom(pre.get_variable().get_id())){
                precondition.push_back(get_proposition(pre));
            }
        }

        // numeric precondition
        for (int pre : numeric_task.get_action_num_list(op_2.get_id())){
            for (int i : numeric_task.get_numeric_conditions_id(pre)){
                precondition.push_back(get_proposition(i));
            }
        }

        std::unordered_set<int> op_1_ids;
        string name = op_2.get_name();

        for (int i = 0; i < numeric_task.get_action_n_linear_eff(op_2.get_id()); ++i) {
            bool second_order_simple = false;
            std::vector<ap_float> coeff = numeric_task.get_action_linear_coefficients(op_2.get_id())[i];
            ap_float constant = numeric_task.get_action_linear_constants(op_2.get_id())[i];
            int lhs_id_2 = numeric_task.get_action_linear_lhs(op_2.get_id())[i];
            coeff[lhs_id_2] -= 1.0;
            if (use_second_order_simple) {
                // no self-loop
                if (coeff[lhs_id_2] <= precision && coeff[lhs_id_2] >= -precision) {
                    second_order_simple = true;
                    std::vector<int> tmp_op_1_ids;
                    for (OperatorProxy op_1 : task_proxy.get_operators()) {
                        // no linear effect
                        const std::vector<int> lhs_ids_1 = numeric_task.get_action_linear_lhs(op_1.get_id());
                        for (int j = 0; j < numeric_task.get_action_n_linear_eff(op_1.get_id()); ++j) {
                            if (coeff[lhs_ids_1[j]] > precision || coeff[lhs_ids_1[j]] < -precision) {
                                second_order_simple = false;
                                break;
                            }
                        }
                        if (second_order_simple) {
                            ap_float net = 0;
                            for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id) {
                                net += coeff[n_id] * numeric_task.get_action_eff_list(op_1.get_id())[n_id];
                            }
                            if (net > precision || net < -precision) {
                                // no parallel simple effect
                                ap_float simple_e = numeric_task.get_action_eff_list(op_1.get_id())[lhs_id_2];
                                if (simple_e > precision || simple_e < -precision) {
                                    second_order_simple = false;
                                    break;
                                } else {
                                    tmp_op_1_ids.push_back(op_1.get_id());
                                }
                            }
                        } else {
                            break;
                        }
                    }
                    if (second_order_simple) {
                        for (auto op_1_id : tmp_op_1_ids)
                            op_1_ids.insert(op_1_id);
                    }
                }
            }
            if (!second_order_simple) {
                std::vector<ap_float> coefficient_plus(coeff);
                LinearNumericCondition lnc_plus(coefficient_plus, constant);
                lnc_plus.is_strictly_greater = true;
                add_infinite_operator(precondition, std::move(lnc_plus), lhs_id_2, true, op_2.get_id(), op_2.get_cost(), name);
                ++n_infinite_operators;

                std::vector<ap_float> coefficient_minus(coeff);
                for (auto &c : coefficient_minus) {
                    if (c > precision || c < -precision) c = -c;
                }
                LinearNumericCondition lnc_minus(coefficient_minus, -constant);
                lnc_minus.is_strictly_greater = true;
                add_infinite_operator(precondition, std::move(lnc_minus), lhs_id_2, false, op_2.get_id(), op_2.get_cost(), name);
                ++n_infinite_operators;
            }
        }

        for (int op_id_1 : op_1_ids) {
            OperatorProxy op_1 = task_proxy.get_operators()[op_id_1];
            vector<RelaxedProposition *> precondition_1;
            for (FactProxy pre : op_1.get_preconditions()) {
                if(!numeric_task.is_numeric_axiom(pre.get_variable().get_id())){
                    precondition.push_back(get_proposition(pre));
                }
            }
            string name_1 = op_1.get_name();
            add_second_order_simple_operator(move(precondition_1), precondition, op_1.get_id(), op_2.get_id(), op_1.get_cost(),
                                             op_2.get_cost(), name_1, name);
            ++n_second_order_simple_operators;
        }
    }

    void LandmarkCutLandmarks::add_second_order_simple_operator(std::vector<RelaxedProposition *> &&precondition_1,
                                                                const std::vector<RelaxedProposition *> precondition_2,
                                                                int op_id_1, int op_id_2, ap_float base_cost_1, ap_float base_cost_2,
                                                                string &n_1, string &n_2) {
        int id = relaxed_operators.size();
        RelaxedOperator relaxed_op(id, move(precondition_1), precondition_2, op_id_1, op_id_2, base_cost_1, base_cost_2, n_1, n_2);
        if (relaxed_op.preconditions.empty())
            relaxed_op.preconditions.push_back(&artificial_precondition);
        relaxed_operators.push_back(relaxed_op);
    }

    void LandmarkCutLandmarks::add_infinite_operator(const std::vector<RelaxedProposition *> &precondition,
                                                     LinearNumericCondition &&lnc, int lhs, bool plus_infinity,
                                                     int op_id, ap_float base_cost, string &n) {
        std::vector<RelaxedProposition *> new_precondition(precondition);
        if (numeric_task.redundant_constraints) {
            for (RelaxedProposition *pre : precondition) {
                if (pre->is_numeric_condition) {
                    int red_id = conditions.size();
                    LinearNumericCondition red = conditions[pre->id_numeric_condition] + lnc;
                    int red_var_id = propositions.size();
                    RelaxedProposition red_prop;
                    red_prop.is_numeric_condition = true;
                    red_prop.id_numeric_condition = conditions.size();
                    stringstream red_name;
                    red_name << "numeric (" << red << ")";
                    red_prop.name = red_name.str();
                    propositions.push_back(std::vector<RelaxedProposition>());
                    propositions[red_var_id].push_back(red_prop);
                    new_precondition.push_back(get_proposition(red_id));
                    ++num_propositions;
                    conditions.push_back(std::move(red));
                    epsilons.push_back(epsilon);
                }
            }
        }
        int prop_id = conditions.size();
        int var_id = propositions.size();
        RelaxedProposition new_prop;
        new_prop.is_numeric_condition = true;
        new_prop.id_numeric_condition = prop_id;
        stringstream prop_name;
        prop_name << "numeric (" << lnc << ")";
        new_prop.name = prop_name.str();
        propositions.push_back(std::vector<RelaxedProposition>());
        propositions[var_id].push_back(new_prop);
        ++num_propositions;
        conditions.push_back(std::move(lnc));
        epsilons.push_back(epsilon);
        new_precondition.push_back(get_proposition(prop_id));

        int relaxed_op_id = relaxed_operators.size();
        string relaxed_op_name = n + " " + std::to_string(lhs);
        if (plus_infinity)
            relaxed_op_name += " +inf";
        else
            relaxed_op_name += " -inf";
        RelaxedOperator relaxed_op(relaxed_op_id, move(new_precondition), lhs, plus_infinity, op_id, base_cost, relaxed_op_name);
        relaxed_operators.push_back(relaxed_op);
    }

    void LandmarkCutLandmarks::build_numeric_effects() {
        for (auto itr = relaxed_operators.begin(); itr != relaxed_operators.end();) {
            RelaxedOperator &relaxed_op = *itr;
            int op_id_1 = relaxed_op.original_op_id_1;
            int op_id_2 = relaxed_op.original_op_id_2;
            const std::vector<int> &lhs_ids = numeric_task.get_action_linear_lhs(op_id_2);
            vector<ap_float> numeric_effects(conditions.size(), 0);
            for (size_t i = 0; i < conditions.size(); ++i){
                LinearNumericCondition& lnc = conditions[i];
                if (relaxed_op.infinite_lhs != -1) {
                    // operator with a linear effect
                    if ((relaxed_op.plus_infinity && lnc.coefficients[relaxed_op.infinite_lhs] > precision)
                        || (!relaxed_op.plus_infinity && lnc.coefficients[relaxed_op.infinite_lhs] < -precision)) {
                        numeric_effects[i] = numeric_limits<ap_float>::max();
                        relaxed_op.effects.push_back(get_proposition(i));
                    }
                } else if (op_id_1 != -1) {
                    // operator with second-order simple effects
                    ap_float net = 0;
                    for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                        net += lnc.coefficients[n_id]*numeric_task.get_action_eff_list(op_id_2)[n_id];
                    }
                    for (int j = 0; j < numeric_task.get_action_n_linear_eff(op_id_2); ++j){
                        const std::vector<ap_float> &linear_coeff = numeric_task.get_action_linear_coefficients(op_id_2)[j];
                        ap_float w = lnc.coefficients[lhs_ids[j]];
                        for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id) {
                            ap_float k = linear_coeff[n_id];
                            if (n_id == lhs_ids[j]) k -= 1.0;
                            net += w * k * numeric_task.get_action_eff_list(op_id_1)[n_id];
                        }
                    }
                    if (net > precision) {
                        //cout << "adding effect of action " << op.get_name() << " " <<net << " on " << lnc << endl;
                        numeric_effects[i] = net;
                        relaxed_op.effects.push_back(get_proposition(i));
                    }
                } else {
                    // operator with simple effects
                    ap_float net = 0;
                    for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                        net += lnc.coefficients[n_id]*numeric_task.get_action_eff_list(op_id_2)[n_id];
                    }
                    if (net > precision) {
                        //cout << "adding effect of action " << op.get_name() << " " <<net << " on " << lnc << endl;
                        numeric_effects[i] = net;
                        relaxed_op.effects.push_back(get_proposition(i));
                    } else if (use_second_order_simple) {
                        for (int j = 0; j < numeric_task.get_action_n_linear_eff(op_id_2); ++j){
                            int lhs_id = numeric_task.get_action_linear_lhs(op_id_2)[j];
                            if (lnc.coefficients[lhs_id] > precision || lnc.coefficients[lhs_id] < -precision) {
                                relaxed_op.effects.push_back(get_proposition(i));
                                break;
                            }
                        }
                    }
                }
            }
            relaxed_op.numeric_effects = numeric_effects;

            if (relaxed_op.effects.empty()) {
                itr = relaxed_operators.erase(itr);
            } else {
                ++itr;
            }
        }
    }

    RelaxedProposition *LandmarkCutLandmarks::get_proposition(
                                                              const FactProxy &fact) {
        int var_id = fact.get_variable().get_id();
        int val = fact.get_value();
        return &propositions[var_id][val];
    }
    
    RelaxedProposition *LandmarkCutLandmarks::get_proposition(
                                                              const int &n_condition) {
        int propositions_size = propositions.size();
        if (propositions_size <= n_condition + n_var) cout << "wrong vector size " << propositions.size() << " " << n_var << " " << n_condition << endl;
        if (propositions[n_condition + n_var].size() < 1) cout << "no proposition " << endl;
        return &propositions[n_condition + n_var][0];
    }
    
    // heuristic computation
    void LandmarkCutLandmarks::setup_exploration_queue() {
        priority_queue.clear();
        
        for (auto &var_props : propositions) {
            for (RelaxedProposition &prop : var_props) {
                prop.status = UNREACHED;
                prop.explored = false;
            }
        }
        
        artificial_goal.status = UNREACHED;
        artificial_precondition.status = UNREACHED;
        artificial_goal.explored = false;
        artificial_precondition.explored = false;
        
        for (RelaxedOperator &op : relaxed_operators) {
            op.unsatisfied_preconditions = op.preconditions.size();
            op.h_max_supporter = 0;
            op.h_max_supporter_cost = numeric_limits<int>::max();
        }
    }
    
    void LandmarkCutLandmarks::setup_exploration_queue_state(const State &state) {
        // propositions
        for (FactProxy init_fact : state) {
            if (numeric_task.is_numeric_axiom(init_fact.get_variable().get_id())) continue;
            enqueue_if_necessary(get_proposition(init_fact), 0);
            if (debug) cout << "initial state: " << get_proposition(init_fact)->name << endl;
        }
        numeric_initial_state.assign(conditions.size(),0);
        
        if (!ignore_numeric_conditions) {
            // numeric_conditions
            for (size_t i = 0; i < conditions.size(); ++i){
                LinearNumericCondition& lnc = conditions[i];
                ap_float net = lnc.constant - epsilons[i];
                for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                    int id_num = numeric_task.get_numeric_variable(n_id).id_abstract_task;
                    net += lnc.coefficients[n_id]*state.nval(id_num);
                    //cout << n_id << " " << state.nval(n_id) << " " << lnc.coefficients[n_id] << endl;
                }
                //cout << lnc << " evaluated in the initial state " << net << endl;
                numeric_initial_state[i] = -net;
                if (net > -precision) {
                        if (debug) cout << lnc << " is satisfied in initial state " << net << endl;
                    enqueue_if_necessary(get_proposition(i), 0);
                }else{
                    if (debug) cout << lnc << " not satisfied " << net << endl;
                }
            }
        }
        
        enqueue_if_necessary(&artificial_precondition, 0);
    }
    
    void LandmarkCutLandmarks::first_exploration(const State &state) {
        if (debug) cout << "  first exploration : " <<endl;
        assert(priority_queue.empty());
        setup_exploration_queue();
        setup_exploration_queue_state(state);
        while (!priority_queue.empty()) {
            pair<ap_float, RelaxedProposition *> top_pair = priority_queue.pop();
            ap_float popped_cost = top_pair.first;
            RelaxedProposition *prop = top_pair.second;
            ap_float prop_cost = prop->h_max_cost;
            assert(prop_cost <= popped_cost);
            if (prop_cost < popped_cost)
                continue;
            prop->explored = true;
            const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
            for (RelaxedOperator *relaxed_op : triggered_operators) {
                --relaxed_op->unsatisfied_preconditions;
                assert(relaxed_op->unsatisfied_preconditions >= 0);
                if (relaxed_op->unsatisfied_preconditions == 0) {
                    if (use_random_pcf) {
                        relaxed_op->select_random_supporter();
                    } else {
                        relaxed_op->h_max_supporter = prop;
                        relaxed_op->h_max_supporter_cost = prop_cost;
                    }
                    for (RelaxedProposition *effect : relaxed_op->effects)
                        update_queue(state, prop, effect, relaxed_op);
                }
            }
        }
    }
    
    void LandmarkCutLandmarks::first_exploration_incremental(const State &state, vector<RelaxedOperator *> &cut) {
        assert(priority_queue.empty());
        if (debug) cout << "  incremental exploration : " << endl;
        for (RelaxedOperator *relaxed_op : cut) {
            if (relaxed_op->original_op_id_1 != -1) {
                for (RelaxedOperator *relaxed_op_2 : original_to_relaxed_operators[relaxed_op->original_op_id_1]) {
                    if (relaxed_op_2->unsatisfied_preconditions == 0) {
                        for (RelaxedProposition *effect : relaxed_op_2->effects)
                            update_queue(state, relaxed_op_2->h_max_supporter, effect, relaxed_op_2);
                    }
                }
            }
            for (RelaxedOperator *relaxed_op_2 : original_to_relaxed_operators[relaxed_op->original_op_id_2]) {
                if (relaxed_op_2->unsatisfied_preconditions == 0) {
                    for (RelaxedProposition *effect : relaxed_op_2->effects)
                        update_queue(state, relaxed_op_2->h_max_supporter, effect, relaxed_op_2);
                }
            }
        }
        if (debug) cout << "  pushed operators in the cut" << endl;
        while (!priority_queue.empty()) {
            pair<ap_float, RelaxedProposition *> top_pair = priority_queue.pop();
            ap_float popped_cost = top_pair.first;
            RelaxedProposition *prop = top_pair.second;
            ap_float prop_cost = prop->h_max_cost;
            assert(prop_cost <= popped_cost);
            if (prop_cost < popped_cost)
                continue;
            const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
            for (RelaxedOperator *relaxed_op : triggered_operators) {
                if (relaxed_op->h_max_supporter == prop) {
                    ap_float old_supp_cost = relaxed_op->h_max_supporter_cost;
                    if (old_supp_cost > prop_cost) {
                        if (use_random_pcf)
                            relaxed_op->select_random_supporter();
                        else
                            relaxed_op->update_h_max_supporter();
                        ap_float new_supp_cost = relaxed_op->h_max_supporter_cost;
                        if (new_supp_cost != old_supp_cost) {
                            if (debug) cout << "\t  " << prop->name <<" "<< relaxed_op->name << " " << new_supp_cost << " " << old_supp_cost << " " << prop_cost << endl;
                            for (RelaxedProposition *effect : relaxed_op->effects)
                                update_queue(state, relaxed_op->h_max_supporter, effect, relaxed_op);
                        }
                    }
                }
            }
        }
    }
    
    void LandmarkCutLandmarks::second_exploration(const State &state, vector<RelaxedProposition *> &second_exploration_queue,
                                                  vector<RelaxedOperator *> &cut, vector<pair<ap_float, ap_float>> &m_list) {
        assert(second_exploration_queue.empty());
        assert(cut.empty());
        if (debug) cout << "  second exploration" << endl;
        artificial_precondition.status = BEFORE_GOAL_ZONE;
        second_exploration_queue.push_back(&artificial_precondition);
        
        for (FactProxy init_fact : state) {
            if (numeric_task.is_numeric_axiom(init_fact.get_variable().get_id())) continue;
            RelaxedProposition *init_prop = get_proposition(init_fact);
            init_prop->status = BEFORE_GOAL_ZONE;
            if (debug) cout << "\t\t  adding " << init_prop->name << " to the queue " << endl;
            second_exploration_queue.push_back(init_prop);
        }

        if (!ignore_numeric_conditions) { 
            for (size_t i = 0; i < conditions.size(); ++i) {
                if (numeric_initial_state[i] < precision) {
                    RelaxedProposition *init_prop = get_proposition(i);
                    init_prop->status = BEFORE_GOAL_ZONE;
                    if (debug) cout << "\t\t  adding " << init_prop->name << " to the queue " << endl;
                    second_exploration_queue.push_back(init_prop);
                }
            }
        }
        
        int n_iterations = 0;
        while (!second_exploration_queue.empty()) {
            n_iterations++;
            RelaxedProposition *prop = second_exploration_queue.back();
            second_exploration_queue.pop_back();
            const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
            for (RelaxedOperator *relaxed_op : triggered_operators) {
                bool furhter_exploration = true;
                if (relaxed_op->h_max_supporter == prop && std::find(cut.begin(), cut.end(), relaxed_op) == cut.end()) {
                    for (RelaxedProposition *effect : relaxed_op->effects) {
                        if (effect->status == GOAL_ZONE) {
                            if (debug) cout << "\t" << prop->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << relaxed_op->cost_2 << endl;
                            std::pair<ap_float, ap_float> ms = calculate_numeric_times(state, effect, relaxed_op, !disable_ma);

                            if (ms.second > precision || ms.first > precision) {
                                if (debug) cout << "\t\t  adding " << relaxed_op->name << " to the cut " << endl;
                                cut.push_back(relaxed_op);
                                m_list.push_back(ms);

                                if (ms.second <= 1.0 && (relaxed_op->original_op_id_1 == -1 || ms.first <= 1.0)) {
                                    furhter_exploration = false;
                                }
                            }
                        }
                    }
                    if (furhter_exploration) {
                        for (RelaxedProposition *effect : relaxed_op->effects) {
                            if (effect->status != BEFORE_GOAL_ZONE && effect->status != GOAL_ZONE) {
                                assert(effect->status == REACHED);
                                effect->status = BEFORE_GOAL_ZONE;
                                if (debug) cout << "\t\t  adding " << effect->name << " to the queue " << endl;
                                second_exploration_queue.push_back(effect);
                            }
                        }
                    }
                }
            }
        }
        //cout << "\tsecond it " << n_iterations << endl;
    }
    
    void LandmarkCutLandmarks::mark_goal_plateau(RelaxedProposition *subgoal) {
        // NOTE: subgoal can be null if we got here via recursion through
        // a zero-cost action that is relaxed unreachable. (This can only
        // happen in domains which have zero-cost actions to start with.)
        // For example, this happens in pegsol-strips #01.
        if (subgoal && subgoal->status != GOAL_ZONE) {
            subgoal->status = GOAL_ZONE;
            for (RelaxedOperator *achiever : subgoal->effect_of) {
                if (achiever->cost_1 < precision && achiever->cost_2 < precision && achiever->unsatisfied_preconditions == 0) {
                    if (debug) cout << "\tadding subgoal " <<  achiever->h_max_supporter->name << " from precondition of " << achiever->name << " which has effect " << subgoal->name << endl;
                    mark_goal_plateau(achiever->h_max_supporter);
                }
            }
        }
    }
    
    void LandmarkCutLandmarks::validate_h_max() const {
#ifndef NDEBUG
        // Using conditional compilation to avoid complaints about unused
        // variables when using NDEBUG. This whole code does nothing useful
        // when assertions are switched off anyway.
        for (const RelaxedOperator &op : relaxed_operators) {
            if (op.unsatisfied_preconditions) {
                bool reachable = true;
                for (RelaxedProposition *pre : op.preconditions) {
                    if (pre->status == UNREACHED) {
                        reachable = false;
                        break;
                    }
                }
                assert(!reachable);
                assert(!op.h_max_supporter);
            } else {
                assert(op.h_max_supporter);
                int h_max_cost = op.h_max_supporter_cost;
                assert(h_max_cost == op.h_max_supporter->h_max_cost);
                for (RelaxedProposition *pre : op.preconditions) {
                    assert(pre->status != UNREACHED);
                    assert(pre->h_max_cost <= h_max_cost);
                }
            }
        }
#endif
    }
    
    bool LandmarkCutLandmarks::compute_landmarks(
                                                 State state, CostCallback cost_callback,
                                                 LandmarkCallback landmark_callback) {
        for (RelaxedOperator &op : relaxed_operators) {
            op.cost_1 = op.base_cost_1;
            op.cost_2 = op.base_cost_2;
        }
        vector<RelaxedOperator *> cut;
        vector<pair<ap_float, ap_float>> m_list;
        unordered_map<int, ap_float> operator_to_min_cut_cost;
        unordered_map<int, ap_float> operator_to_m;
        Landmark landmark;
        vector<RelaxedProposition *> second_exploration_queue;
        first_exploration(state);
        if (artificial_goal.status == UNREACHED)
            return true;
        
        int num_iterations = 0;
        while (artificial_goal.h_max_cost > precision) {
            ++num_iterations;
            mark_goal_plateau(&artificial_goal);
            assert(cut.empty());
            second_exploration(state, second_exploration_queue, cut, m_list);
            assert(!cut.empty());
            ap_float cut_cost = numeric_limits<ap_float>::max();
            for (size_t i = 0; i < cut.size(); ++i) {
                ap_float current_cut_cost = m_list[i].second * cut[i]->cost_2;

                if (cut[i]->original_op_id_1 != -1) {
                    current_cut_cost += m_list[i].first * cut[i]->cost_1;
                    auto itr = operator_to_min_cut_cost.find(cut[i]->original_op_id_1);
                    if (itr == operator_to_min_cut_cost.end() || current_cut_cost < itr->second)
                        operator_to_min_cut_cost[cut[i]->original_op_id_1] = current_cut_cost;
                }

                auto itr = operator_to_min_cut_cost.find(cut[i]->original_op_id_2);
                if (itr == operator_to_min_cut_cost.end() || current_cut_cost < itr->second)
                    operator_to_min_cut_cost[cut[i]->original_op_id_2] = current_cut_cost;
                cut_cost = std::min(cut_cost, current_cut_cost);
            }
            if (debug) cout << "  cut cost " << artificial_goal.h_max_cost << " " << cut_cost << endl;
            
            for (auto itr : operator_to_min_cut_cost) {
                for (RelaxedOperator *relaxed_op : original_to_relaxed_operators[itr.first]) {
                    ap_float m = itr.second;
                    if (relaxed_op->original_op_id_1 == itr.first && relaxed_op->cost_1 > precision) {
                        if (debug) cout << "\tcut " << relaxed_op->name << " cost1: " << relaxed_op->cost_1;
                        m /= relaxed_op->cost_1;
                        relaxed_op->cost_1 -= cut_cost / m;
                        if (relaxed_op->cost_1 < precision) relaxed_op->cost_1 = 0;
                        if (debug) cout << " -> " << relaxed_op->cost_1<< " m1: " << m << endl;
                        operator_to_m[itr.first] = m;
                    }
                    if (relaxed_op->original_op_id_2 == itr.first && relaxed_op->cost_2 > precision) {
                        if (debug) cout << "\tcut " << relaxed_op->name << " cost2: " << relaxed_op->cost_2;
                        m /= relaxed_op->cost_2;
                        relaxed_op->cost_2 -= cut_cost / m;
                        if (relaxed_op->cost_2 < precision) relaxed_op->cost_2 = 0;
                        if (debug) cout << " -> " << relaxed_op->cost_2 << " m2: " << m << endl;
                        operator_to_m[itr.first] = m;
                    }
                }
            }

            if (cost_callback) {
                cost_callback(cut_cost);
            }
            if (debug) cout << "  cut cost " << cut_cost << endl;
            if (landmark_callback) {
                landmark.clear();
                for (auto itr : operator_to_m) {
                    landmark.push_back({itr.second, itr.first});
                }
                
                landmark_callback(landmark, cut_cost);
            }
            
            first_exploration_incremental(state, cut);
            // validate_h_max();  // too expensive to use even in regular debug mode
            cut.clear();
            m_list.clear();
            operator_to_m.clear();
            operator_to_min_cut_cost.clear();
            
            /*
             Note: This could perhaps be made more efficient, for example by
             using a round-dependent counter for GOAL_ZONE and BEFORE_GOAL_ZONE,
             or something based on total_cost, so that we don't need a per-round
             reinitialization.
             */
            for (auto &var_props : propositions) {
                for (RelaxedProposition &prop : var_props) {
                    if (prop.status == GOAL_ZONE || prop.status == BEFORE_GOAL_ZONE)
                        prop.status = REACHED;
                }
            }
            artificial_goal.status = REACHED;
            artificial_precondition.status = REACHED;
        }
        return false;
    }

    void LandmarkCutLandmarks::update_queue(const State &state, RelaxedProposition *prop, RelaxedProposition *effect, RelaxedOperator *relaxed_op) {
        if (effect->is_numeric_condition) {
            int id_effect = effect->id_numeric_condition;
            if (numeric_initial_state[id_effect] > precision) {
                std::pair<ap_float, ap_float> ms = calculate_numeric_times(state, effect, relaxed_op, !use_irmax);

                if (relaxed_op->original_op_id_1 != -1 && ms.first > precision) {
                    ap_float target_cost = prop->h_max_cost + ms.first * relaxed_op->cost_1 + ms.second * relaxed_op->cost_2;
                    if (debug) cout << "\t  " << relaxed_op->h_max_supporter->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << target_cost << endl;
                    enqueue_if_necessary(effect, target_cost);
                } else if (relaxed_op->original_op_id_1 == -1 && ms.second > precision) {
                    ap_float target_cost = prop->h_max_cost + ms.second * relaxed_op->cost_2;
                    if (debug) cout << "\t  " << relaxed_op->h_max_supporter->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << target_cost << endl;
                    enqueue_if_necessary(effect, target_cost);
                }
            }
        } else {
            ap_float target_cost = prop->h_max_cost + relaxed_op->cost_2;
            if(debug) cout << "\t  " << relaxed_op->h_max_supporter->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << target_cost << endl;
            enqueue_if_necessary(effect, target_cost);
        }
    }
    
    std::pair<ap_float, ap_float> LandmarkCutLandmarks::calculate_numeric_times(const State &state, RelaxedProposition *effect,
                                                                                RelaxedOperator *relaxed_op, bool use_ma){
        if (use_ma && effect->is_numeric_condition && relaxed_op->infinite_lhs == -1){
            int id_effect = effect->id_numeric_condition;

            if (relaxed_op->original_op_id_1 == -1) {
                ap_float net = relaxed_op->numeric_effects[id_effect];

                if (use_linear_effects && use_second_order_simple) {
                    const LinearNumericCondition &lnc = conditions[id_effect];
                    int op_id = relaxed_op->original_op_id_2;

                    for (int i = 0; i < numeric_task.get_action_n_linear_eff(op_id); ++i) {
                        int lhs = numeric_task.get_action_linear_lhs(op_id)[i];
                        ap_float w = lnc.coefficients[lhs];
                        net += w * numeric_task.get_action_linear_constants(op_id)[i];
                        for (int n_id = 0, n_vars = numeric_task.get_n_numeric_variables(); n_id < n_vars; ++n_id) {
                            ap_float k = numeric_task.get_action_linear_coefficients(op_id)[i][n_id];
                            if (n_id == numeric_task.get_action_linear_lhs(op_id)[i]) k -= 1.0;
                            int id_num = numeric_task.get_numeric_variable(n_id).id_abstract_task;
                            net += w * k * state.nval(id_num);
                        }
                    }
                }

                if (net < precision) return std::make_pair(0, 0);

                ap_float m = numeric_initial_state[id_effect]/ net;

                if (m < precision) return std::make_pair(0, 0);

                if (ceiling_less_than_one) return std::make_pair(0, std::max(m, 1.0));

                return std::make_pair(0, m);
            } else {
                if (relaxed_op->cost_1 < precision || relaxed_op->cost_2 < precision) return std::make_pair(1, 1);

                const LinearNumericCondition &lnc = conditions[id_effect];
                ap_float k_0 = 0;
                int op_id = relaxed_op->original_op_id_2;

                for (int i = 0; i < numeric_task.get_action_n_linear_eff(op_id); ++i) {
                    int lhs = numeric_task.get_action_linear_lhs(op_id)[i];
                    ap_float w = lnc.coefficients[lhs];
                    k_0 += w * numeric_task.get_action_linear_constants(op_id)[i];
                    for (int n_id = 0, n_vars = numeric_task.get_n_numeric_variables(); n_id < n_vars; ++n_id) {
                        ap_float k = numeric_task.get_action_linear_coefficients(op_id)[i][n_id];
                        if (n_id == numeric_task.get_action_linear_lhs(op_id)[i]) k -= 1.0;
                        int id_num = numeric_task.get_numeric_variable(n_id).id_abstract_task;
                        k_0 += w * k * state.nval(id_num);
                    }
                }

                if (k_0 > numeric_initial_state[id_effect]) return std::make_pair(0, 1);

                ap_float m = sqrt(numeric_initial_state[id_effect] / relaxed_op->numeric_effects[id_effect]);
                ap_float m_1 = m * sqrt(relaxed_op->cost_2 / relaxed_op->cost_1) - k_0 / relaxed_op->numeric_effects[id_effect];
                ap_float m_2 = m * sqrt(relaxed_op->cost_1 / relaxed_op->cost_2);

                if (m_1 < precision) {
                    m_1 = 0;
                    m_2 = numeric_initial_state[id_effect] / k_0;
                }

                return std::make_pair(m_1, m_2);
            }
        }
        return std::make_pair(0, 1);
    }
}

