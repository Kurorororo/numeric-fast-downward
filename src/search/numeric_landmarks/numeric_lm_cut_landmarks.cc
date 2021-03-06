#include "numeric_lm_cut_landmarks.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <algorithm>

#include "../numeric_operator_counting/numeric_helper.h"


using namespace std;
using namespace numeric_helper;

bool debug = false;
double precision = 0.001;

namespace numeric_lm_cut_heuristic {
    // construction and destruction
    LandmarkCutLandmarks::LandmarkCutLandmarks(const TaskProxy &task_proxy, bool ceiling_less_than_one, bool ignore_numeric,
                                               bool use_random_pcf, bool use_irmax, bool disable_ma)
        : numeric_task(NumericTaskProxy(task_proxy, false)),
          ceiling_less_than_one(ceiling_less_than_one),
          ignore_numeric_conditions(ignore_numeric),
          use_random_pcf(use_random_pcf),
          use_irmax(use_irmax),
          disable_ma(disable_ma) {
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
                name << "numeric (" << numeric_task.get_condition(i) << ")";
                prop.name =  name.str();
                propositions[var_id].push_back(prop);
                ++num_propositions;
                //cout << "adding numeric precondition " << num_values << " : " << num_propositions << " " << var_id << endl;
            }
        }
        
        // Build relaxed operators for operators and axioms.
        for (OperatorProxy op : task_proxy.get_operators())
            build_relaxed_operator(op);
        
        // Simplify relaxed operators.
        // simplify();
        /* TODO: Put this back in and test if it makes sense,
         but only after trying out whether and how much the change to
         unary operators hurts. */
        
        // Build artificial goal proposition and operator.
        vector<RelaxedProposition *> goal_op_pre, goal_op_eff;
        vector<ap_float> numeric_eff(numeric_task.get_n_numeric_conditions(),0);
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
        add_relaxed_operator(move(goal_op_pre), move(goal_op_eff), move(numeric_eff), -1, 0, name_goal);
        
        // Cross-reference relaxed operators.
        for (RelaxedOperator &op : relaxed_operators) {
            for (RelaxedProposition *pre : op.preconditions){
                pre->precondition_of.push_back(&op);
            }
            for (RelaxedProposition *eff : op.effects)
                eff->effect_of.push_back(&op);
        }
        cout << "ops " <<  task_proxy.get_operators().size() << ", prop: " << num_propositions << ", numeric conditions " <<  numeric_task.get_n_numeric_conditions() << endl;
    }
    
    LandmarkCutLandmarks::~LandmarkCutLandmarks() {
    }
    
    void LandmarkCutLandmarks::build_relaxed_operator(const OperatorProxy &op) {
        vector<RelaxedProposition *> precondition;
        vector<RelaxedProposition *> effects;
        vector<ap_float> numeric_effect(numeric_task.get_n_numeric_conditions(),0);
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
                    LinearNumericCondition &num_values = numeric_task.get_condition(i);
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

        if (!ignore_numeric_conditions) {
            // add effects
            for (size_t i = 0; i < numeric_task.get_n_conditions(); ++i){
                ap_float net = 0;
                LinearNumericCondition& lnc = numeric_task.get_condition(i);
                for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                    net += lnc.coefficients[n_id]*numeric_task.get_action_eff_list(op.get_id())[n_id];
                }
                if (net > 0) {
                    //cout << "adding effect of action " << op.get_name() << " " <<net << " on " << lnc << endl;
                    numeric_effect[i] = net;
                    effects.push_back(get_proposition(i));
                }
            }
        }

        string name = op.get_name();
        add_relaxed_operator(
                             move(precondition), move(effects), move(numeric_effect), op.get_id(), op.get_cost(), name);
    }
    
    void LandmarkCutLandmarks::add_relaxed_operator(
                                                    vector<RelaxedProposition *> &&precondition,
                                                    vector<RelaxedProposition *> &&effects,
                                                    vector<ap_float> &&numeric_effects,
                                                    int op_id, ap_float base_cost, string &n) {
        RelaxedOperator relaxed_op(
                                   move(precondition), move(effects), move(numeric_effects), op_id, base_cost,n);
        if (relaxed_op.preconditions.empty())
            relaxed_op.preconditions.push_back(&artificial_precondition);
        relaxed_operators.push_back(relaxed_op);
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
        numeric_initial_state.assign(numeric_task.get_n_conditions(),0);
        
        if (!ignore_numeric_conditions) {
            // numeric_conditions
            for (size_t i = 0; i < numeric_task.get_n_conditions(); ++i){
                LinearNumericCondition& lnc = numeric_task.get_condition(i);
                ap_float net = lnc.constant - numeric_task.get_epsilon(i);
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
            pair<int, RelaxedProposition *> top_pair = priority_queue.pop();
            int popped_cost = top_pair.first;
            RelaxedProposition *prop = top_pair.second;
            int prop_cost = prop->h_max_cost;
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
                        update_queue(prop, effect, relaxed_op);
                }
            }
        }
    }
    
    void LandmarkCutLandmarks::first_exploration_incremental(
                                                             vector<RelaxedOperator *> &cut) {
        assert(priority_queue.empty());
        /* We pretend that this queue has had as many pushes already as we
         have propositions to avoid switching from bucket-based to
         heap-based too aggressively. This should prevent ever switching
         to heap-based in problems where action costs are at most 1.
         */
        if (debug) cout << "  incremental exploration : " << endl;
        priority_queue.add_virtual_pushes(num_propositions);
        for (RelaxedOperator *relaxed_op : cut) {
            for (RelaxedProposition *effect : relaxed_op->effects)
                update_queue(relaxed_op->h_max_supporter, effect, relaxed_op);
        }
        while (!priority_queue.empty()) {
            pair<int, RelaxedProposition *> top_pair = priority_queue.pop();
            int popped_cost = top_pair.first;
            RelaxedProposition *prop = top_pair.second;
            int prop_cost = prop->h_max_cost;
            assert(prop_cost <= popped_cost);
            if (prop_cost < popped_cost)
                continue;
            const vector<RelaxedOperator *> &triggered_operators =
            prop->precondition_of;
            for (RelaxedOperator *relaxed_op : triggered_operators) {
                if (relaxed_op->h_max_supporter == prop) {
                    int old_supp_cost = relaxed_op->h_max_supporter_cost;
                    if (old_supp_cost > prop_cost) {
                        if (use_random_pcf)
                            relaxed_op->select_random_supporter();
                        else
                            relaxed_op->update_h_max_supporter();
                        int new_supp_cost = relaxed_op->h_max_supporter_cost;
                        if (new_supp_cost != old_supp_cost) {
                            // This operator has become cheaper.
                            if (new_supp_cost >= old_supp_cost){
                               if(debug)  cout << "\t  " << prop->name <<" "<< relaxed_op->name << " " << new_supp_cost << " " << old_supp_cost << " " << prop_cost << endl;
                            }
                            if(debug)  cout << "\t  " << prop->name <<" "<< relaxed_op->name << " " << new_supp_cost << " " << old_supp_cost << " " << prop_cost << endl;
                            //cout << new_supp_cost << " " << old_supp_cost << endl;
                            //if (new_supp_cost >  old_supp_cost) continue;
                            assert(new_supp_cost < old_supp_cost);
                            for (RelaxedProposition *effect : relaxed_op->effects)
                                update_queue(relaxed_op->h_max_supporter, effect, relaxed_op);
                        }
                    }
                }
            }
        }
    }
    
    void LandmarkCutLandmarks::second_exploration(
                                                  const State &state, vector<RelaxedProposition *> &second_exploration_queue,
                                                  vector<RelaxedOperator *> &cut, unordered_map<int, ap_float> &operator_to_m) {
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
            for (size_t i = 0; i < numeric_task.get_n_conditions(); ++i) {
                if (numeric_initial_state[i] <= 0) {
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
                if (relaxed_op->h_max_supporter == prop
                    && operator_to_m.find(relaxed_op->original_op_id) == operator_to_m.end()) {
                    ap_float min_m = numeric_limits<ap_float>::max();
                    bool reached_goal_zone = false;
                    for (RelaxedProposition *effect : relaxed_op->effects) {
                        if (debug) cout << "\t" << prop->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << relaxed_op->cost << endl;
                        if (effect->status == GOAL_ZONE) {
                            assert(relaxed_op->cost > 0);
                            ap_float m = calculate_numeric_times(effect, relaxed_op, !disable_ma);
                            if (m < min_m) {
                                if (!reached_goal_zone) {
                                    if (debug) cout << "\t\t  adding " << relaxed_op->name << " to the cut " << endl;
                                    cut.push_back(relaxed_op);
                                    reached_goal_zone = true;
                                }
                                min_m = m;
                            }
                        }
                    }
                    if (reached_goal_zone)
                        operator_to_m[relaxed_op->original_op_id] = min_m;
                    if (min_m > 1.0) {
                        for (RelaxedProposition *effect : relaxed_op->effects) {
                            if (effect->status != BEFORE_GOAL_ZONE && effect->status != GOAL_ZONE && min_m > 1.0) {
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
            for (RelaxedOperator *achiever : subgoal->effect_of)
                if (achiever->cost == 0){
                    if (debug) cout << "\tadding subgoal " <<  achiever->h_max_supporter->name << " from precondition of " << achiever->name << " which has effect " << subgoal->name << endl;
                    mark_goal_plateau(achiever->h_max_supporter);
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
            op.cost = op.base_cost;
        }
        // The following three variables could be declared inside the loop
        // ("second_exploration_queue" even inside second_exploration),
        // but having them here saves reallocations and hence provides a
        // measurable speed boost.
        vector<RelaxedOperator *> cut;
        unordered_map<int, ap_float> operator_to_m;
        Landmark landmark;
        vector<RelaxedProposition *> second_exploration_queue;
        first_exploration(state);
        // validate_h_max();  // too expensive to use even in regular debug mode
        if (artificial_goal.status == UNREACHED)
            return true;
        
        int num_iterations = 0;
        while (artificial_goal.h_max_cost != 0) {
            ++num_iterations;
            mark_goal_plateau(&artificial_goal);
            assert(cut.empty());
            second_exploration(state, second_exploration_queue, cut, operator_to_m);
            assert(!cut.empty());
            ap_float cut_cost = numeric_limits<ap_float>::max();
            ap_float cut_cost_single = numeric_limits<ap_float>::max();
            ap_float m_single = numeric_limits<ap_float>::max();
            for (RelaxedOperator* op : cut){
                ap_float m = operator_to_m[op->original_op_id];
                if (m*op->cost < cut_cost){
                    cut_cost = min(cut_cost, m*op->cost);
                    cut_cost_single = op->cost;
                    m_single = m;
                }
            }
            if (debug) cout << "  cut cost " << artificial_goal.h_max_cost << " " << cut_cost << endl;
            
            for (RelaxedOperator* op : cut){
                ap_float m = operator_to_m[op->original_op_id];
                if (debug) cout << "\tcut " << op->name << " " << op->cost;
                if (m > 0){
                    op->cost -= cut_cost_single * m_single/m;
                } else {
                    op->cost = 0;
                }
                if (debug) cout << " -> " << op->cost << ", m: " << m << endl;
            }
            
            if (cost_callback) {
                cost_callback(cut_cost);
            }
            if (debug) cout << "  cut cost " << cut_cost << endl;
            if (landmark_callback) {
                landmark.clear();
                for (RelaxedOperator *op: cut)
                    landmark.push_back({operator_to_m[op->original_op_id], op->original_op_id});
                
                landmark_callback(landmark, cut_cost);
            }
            
            first_exploration_incremental(cut);
            // validate_h_max();  // too expensive to use even in regular debug mode
            cut.clear();
            operator_to_m.clear();
            
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

    void LandmarkCutLandmarks::update_queue(RelaxedProposition *prop, RelaxedProposition *effect, RelaxedOperator *relaxed_op) {
        if (effect->is_numeric_condition) {
            int id_effect = effect->id_numeric_condition;
            if (relaxed_op->numeric_effects[id_effect] > 0 && numeric_initial_state[id_effect] > 0) {
                ap_float m = calculate_numeric_times(effect, relaxed_op, !use_irmax);
                ap_float target_cost = prop->h_max_cost + m * relaxed_op->cost;
                if(debug) cout << "\t  " << relaxed_op->h_max_supporter->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << target_cost << " " << m << endl;
                enqueue_if_necessary(effect, target_cost);
            }
        } else {
            ap_float target_cost = prop->h_max_cost + relaxed_op->cost;
            if(debug) cout << "\t  " << relaxed_op->h_max_supporter->name << " -> " << effect->name <<  " : " << relaxed_op->name << " " << target_cost << endl;
            enqueue_if_necessary(effect, target_cost);
        }
    }
    
    ap_float LandmarkCutLandmarks::calculate_numeric_times(RelaxedProposition *effect, RelaxedOperator *relaxed_op, bool use_ma){
        if (use_ma && effect->is_numeric_condition){
            int id_effect = effect->id_numeric_condition;
            //ap_float m = floor(numeric_initial_state[id_effect]/relaxed_op->numeric_effects[id_effect]);
            ap_float m = numeric_initial_state[id_effect]/relaxed_op->numeric_effects[id_effect];

            if (ceiling_less_than_one)
                return std::max(m, 1.0);

            return m;
        }
        return 1;
    }
}

