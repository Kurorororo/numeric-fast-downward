#include "../globals.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"

#include "../lp/lp_solver.h"

#include "../utils/markup.h"
#include "sas_compilation.h"
#include "../numeric_operator_counting/numeric_helper.h"

using namespace std;
using namespace operator_counting;
using namespace numeric_helper;


void SASStateChangeModel::initialize_variables(const std::shared_ptr<AbstractTask> task,
                                               std::vector<lp::LPVariable> &variables,
                                               double infinity){
    cout << "initializing variables in SAS SC" << endl;
    TaskProxy task_proxy(*task);
    numeric_task = NumericTaskProxy(task_proxy);
    OperatorsProxy ops = task_proxy.get_operators();
    VariablesProxy vars = task_proxy.get_variables();
    int n_ops = ops.size();
    int n_vars = vars.size();
    
    index_opt->assign(n_ops,vector<int>(t_max,-1));
    index_var.assign(n_vars,vector<vector<vector<int>>>());
    sc_actions.assign(n_vars,vector<vector<set<int>>>());
    eff_actions.assign(n_vars,vector<set<int>>());
    effs_actions.assign(n_ops,vector<pair<int,set<int>>>(n_vars,{-1,set<int>()}));
    pre_actions.assign(n_vars,vector<set<int>>());
    sc_facts.assign(n_vars,vector<bool>());
    
    int i_var = 0;
    for (VariableProxy var : vars) {
        int n_vals = var.get_domain_size();
        index_var[i_var].assign(n_vals,vector<vector<int>>(n_vals,vector<int>(t_max,-1)));
        sc_facts[i_var].assign(n_vals,true);
        ++i_var;
    }
    
    create_sets(task);
    add_variables(task,variables,infinity,t_min,t_max);
}

void SASStateChangeModel::create_sets(const std::shared_ptr<AbstractTask> task){
    TaskProxy task_proxy(*task);
    OperatorsProxy ops = task_proxy.get_operators();
    VariablesProxy vars = task_proxy.get_variables();
    int n_vars = vars.size();
    
//    sc_actions.resize(n_vars);
//    eff_actions.resize(n_vars);
//    pre_actions.resize(n_vars);
    
    int i_var = 0;
    for (VariableProxy var : vars) {
        int n_vals = var.get_domain_size();
        sc_actions[i_var].assign(n_vals,vector<set<int>>(n_vals,set<int>()));
        eff_actions[i_var].assign(n_vals,set<int>());
        pre_actions[i_var].assign(n_vals,set<int>());
        i_var++;
    }
    
    for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
        const OperatorProxy &op = ops[op_id];
        vector<int> precondition(vars.size(), -1);
        vector<int> effects(vars.size(), -1);
        for (FactProxy condition : op.get_preconditions()) {
            int pre_var_id = condition.get_variable().get_id();
            if (numeric_task.is_numeric_axiom(pre_var_id)) continue;
                precondition[pre_var_id] = condition.get_value();
        }
        for (EffectProxy effect_proxy : op.get_effects()) {
            FactProxy effect = effect_proxy.get_fact();
            int var = effect.get_variable().get_id();
            if (numeric_task.is_numeric_axiom(var)) continue;
            int pre = precondition[var];
            int post = effect.get_value();
            assert(post != -1);
            //assert(pre != post);
            effects[var] = post;
            // if a is e
            effs_actions[op_id][var].first = post;
            //cout << "setting " << op_id << " " << var << " " << effs_actions[op_id][var].first << endl;
            if (pre != -1){
                // add but not pre
                //cout << pre << " " << post << " " << sc_actions[var].size() << endl;
                sc_actions[var][pre][post].insert(op_id);
                effs_actions[op_id][var].second.insert(pre);
            }else{
                //eff_actions[var][post].insert(op_id);
                sc_facts[var][post] = false;
                VariableProxy v = task_proxy.get_variables()[var];
                int n_vals = v.get_domain_size();
                for (int val = 0; val < n_vals; val++){
                    if (val!=post){
                        sc_actions[var][val][post].insert(op_id);
                        effs_actions[op_id][var].second.insert(val);
                    }
                }
            }
        }
        for (FactProxy condition : op.get_preconditions()) {
            int var = condition.get_variable().get_id();
            if (numeric_task.is_numeric_axiom(var)) continue;
            int pre = condition.get_value();
            int post = effects[var];
            if (post == -1 || post == pre){
                pre_actions[var][pre].insert(op_id);
            }
        }
    }
    
}


void SASStateChangeModel::add_variables(
                                        const std::shared_ptr<AbstractTask> task,
                                        std::vector<lp::LPVariable> &variables,
                                        double infinity, int t_min, int t_max){
    TaskProxy task_proxy(*task);
    numeric_task = NumericTaskProxy(task_proxy);
    OperatorsProxy ops = task_proxy.get_operators();
    VariablesProxy vars = task_proxy.get_variables();
    int n_ops = ops.size();
    
    // resize idex
    for (int op_id = 0; op_id < n_ops; ++op_id) {
        (*index_opt)[op_id].resize(t_max,-1);
    }
    int i_var = 0;
    for (VariableProxy var : vars) {
        int n_vals = var.get_domain_size();
        for (int in_value = 0; in_value < n_vals; ++in_value) {
            for (int out_value = 0; out_value < n_vals; ++out_value) {
                index_var[i_var][in_value][out_value].resize(t_max,-1);
            }
        }
        ++i_var;
    }
    
    // add variables
    for (int t = t_min; t < t_max; ++t) {
        // actions
        for (int op_id = 0; op_id < n_ops; ++op_id) {
            
            stringstream name;
            name << "x_" << ops[op_id].get_name() << "_" << t;
            //cout << name.str() << endl;
            (*index_opt)[op_id][t] = variables.size();
            variables.push_back(lp::LPVariable(0,1,ops[op_id].get_cost(), name.str(), lp::LPVariableType::binary));
        }
        // facts
        i_var = 0;
        for (VariableProxy var : vars) {
            int n_vals = var.get_domain_size();
            for (int in_value = 0; in_value < n_vals; ++in_value) {
                for (int out_value = 0; out_value < n_vals; ++out_value) {
                    stringstream name;
                    name << "y_" << i_var << "_" << in_value<< "_" << out_value<< "_" << t;
                    //cout << name.str() << endl;
                    index_var[i_var][in_value][out_value][t] = variables.size();
                    variables.push_back(lp::LPVariable(0,1,0, name.str(), lp::LPVariableType::continous));
                }
            }
            ++i_var;
        }
    }
}


void SASStateChangeModel::initialize_constraints(const std::shared_ptr<AbstractTask> task,
                                                 std::vector<lp::LPConstraint> &constraints,
                                                 double infinity){
    
    cout << "initializing constraints for SAS SC" << endl;
    initial_state_constraint(task, constraints);
    goal_state_constraint(task, constraints, t_max);
    update_state_change_constraint(task, constraints, infinity,t_min,t_max);
    precondition_effect_constraint(task, constraints, infinity,t_min,t_max);
    effect_constraint(task, constraints, infinity,t_min,t_max);
    mutex_relaxtion_constraint(task, constraints, infinity,t_min,t_max);
}

bool SASStateChangeModel::update_constraints(const State &state, lp::LPSolver &lp_solver){
    return false;
}

bool SASStateChangeModel::update_constraints(const int horizon,
                                             lp::LPSolver &lp_solver,const std::shared_ptr<AbstractTask> task,
                                             std::vector<lp::LPVariable> &variables,
                                             double infinity, std::vector<lp::LPConstraint> & constraints){
    cout << "adding constraint from SAS SC" << endl;
    t_min = t_max;
    t_max = horizon;
    add_variables(task,variables,infinity,t_min,t_max);
    update_state_change_constraint(task, constraints, infinity,t_min-1,t_max);
    precondition_effect_constraint(task, constraints, infinity,t_min,t_max);
    effect_constraint(task, constraints, infinity,t_min,t_max);
    mutex_relaxtion_constraint(task, constraints, infinity,t_min,t_max);
    goal_state_constraint(task, constraints, t_max);
    return false;
}

void SASStateChangeModel::initial_state_constraint(const std::shared_ptr<AbstractTask> task,
                                                   std::vector<lp::LPConstraint> &constraints){
    
    TaskProxy task_proxy(*task);
    VariablesProxy vars = task_proxy.get_variables();
    
    State initial_state =  task_proxy.get_initial_state();
    int i_var = 0;
    for (VariableProxy var : vars) {
        int n_vals = var.get_domain_size();
        int initial_value = initial_state[i_var].get_value();
        lp::LPConstraint constraint(1., 1.);
        for (int val = 0; val < n_vals; val++){
            constraint.insert(index_var[i_var][initial_value][val][0],1.);
        }
        constraints.push_back(constraint);
        // add 0 condition
        {
            for (int in_val = 0; in_val < n_vals; in_val++){
                if (in_val == initial_value) continue;
                for (int out_val = 0; out_val < n_vals; out_val++){
                    lp::LPConstraint constraint(0., 0.);
                    constraint.insert(index_var[i_var][in_val][out_val][0],1.);
                    constraints.push_back(constraint);
                }
            }
        }
        i_var++;
    }
}

void SASStateChangeModel::goal_state_constraint(const std::shared_ptr<AbstractTask> task,
                                                std::vector<lp::LPConstraint> &constraints,
                                                int t_max) {
    TaskProxy task_proxy(*task);
    VariablesProxy vars = task_proxy.get_variables();
    bool first = goal_index.empty();
    if (first)
        goal_index.assign(task_proxy.get_goals().size(),-1);
    for (size_t id_goal = 0; id_goal < task_proxy.get_goals().size(); ++id_goal) {
        FactProxy goal = task_proxy.get_goals()[id_goal];
        lp::LPConstraint constraint(1., 1.);
        if (numeric_task.is_numeric_axiom(goal.get_variable().get_id())) continue;
        int var = goal.get_variable().get_id();
        int g_val = goal.get_value();
        int num_values = numeric_task.get_n_proposition_value(var);
        for (int val = 0; val < num_values; val++){
            constraint.insert(index_var[var][g_val][val][t_max-1],1.);
        }
        if (!numeric_task.numeric_goals_empty(id_goal)) continue; // this is a numeric goal
        if (!constraint.empty()) {
            if (first){
                goal_index[id_goal]= constraints.size();
                constraints.push_back(constraint);
            }else{
                constraints[goal_index[id_goal]] = constraint;
            }
        }
    }
}

void SASStateChangeModel::update_state_change_constraint(const std::shared_ptr<AbstractTask> task,
                                                         std::vector<lp::LPConstraint> &constraints,
                                                         double infinity, int t_min, int t_max){
    
    TaskProxy task_proxy(*task);
    for (int t = t_min; t < t_max-1; ++t){
        VariablesProxy vars = task_proxy.get_variables();
        int i_var = 0;
        for (VariableProxy var : vars) {
            int n_vals = var.get_domain_size();
            for (int in_val = 0; in_val < n_vals; in_val++){
                lp::LPConstraint constraint(0., 0.);
                for (int out_val = 0; out_val < n_vals; out_val++){
                    constraint.insert(index_var[i_var][in_val][out_val][t+1],1.);
                    constraint.insert(index_var[i_var][out_val][in_val][t],-1.);
                }
                constraints.push_back(constraint);
            }
            ++i_var;
        }
    }
}

void SASStateChangeModel::precondition_effect_constraint(const std::shared_ptr<AbstractTask> task,
                                                         std::vector<lp::LPConstraint> &constraints,
                                                         double infinity, int t_min, int t_max){
    
    
    TaskProxy task_proxy(*task);
    for (int t = t_min; t < t_max; ++t){
        VariablesProxy vars = task_proxy.get_variables();
        int i_var = 0;
        for (VariableProxy var : vars) {
            int n_vals = var.get_domain_size();
            for (int in_val = 0; in_val < n_vals; in_val++){
                for (int out_val = 0; out_val < n_vals; out_val++){
                    if (out_val==in_val) continue;
                    set<int> opts = sc_actions[i_var][in_val][out_val];
                    //if (!opts.empty()){
                    if (sc_facts[i_var][out_val]){
                        lp::LPConstraint constraint(0., 0.);
                        constraint.insert(index_var[i_var][in_val][out_val][t],-1.);
                        for(auto opt : opts){
                            constraint.insert((*index_opt)[opt][t],1.);
                        }
                        constraints.push_back(constraint);
                    }else{
                        lp::LPConstraint constraint(0., infinity);
                        constraint.insert(index_var[i_var][in_val][out_val][t],-1.);
                        for(auto opt : opts){
                            constraint.insert((*index_opt)[opt][t],1.);
                        }
                        constraints.push_back(constraint);
                    }
                    //}
                }
                // prevail
                {
                    set<int> opts = pre_actions[i_var][in_val];
                    for(auto opt : opts){
                        lp::LPConstraint constraint(-infinity, 0.);
                        constraint.insert(index_var[i_var][in_val][in_val][t],-1.);
                        constraint.insert((*index_opt)[opt][t],1.);
                        constraints.push_back(constraint);
                    }
                }

            }
            ++i_var;
        }
    }
    
}

void SASStateChangeModel::effect_constraint(const std::shared_ptr<AbstractTask> task,
                                                         std::vector<lp::LPConstraint> &constraints,
                                                         double infinity, int t_min, int t_max){
    
    TaskProxy task_proxy(*task);
    OperatorsProxy ops = task_proxy.get_operators();
    VariablesProxy vars = task_proxy.get_variables();
    return;
    for (int t = t_min; t < t_max; ++t){
        for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
            int i_var = 0;
            for (VariableProxy var : vars) {
                int post = effs_actions[op_id][i_var].first;
                //cout << "adding " << post << " to action " << op_id << " for variable " << i_var << endl;
                if (post != -1) {
                    set<int> values = effs_actions[op_id][i_var].second;
                    lp::LPConstraint constraint(0., infinity);
                    //cout << "adding " << values.size() << " to action " << op_id << " for variable " << i_var << endl;
                    constraint.insert((*index_opt)[op_id][t],-1.);
                    for(auto val : values){
                        constraint.insert(index_var[i_var][val][post][t],1.);
                    }
                    constraints.push_back(constraint);
                }
                ++i_var;
            }
        }
    }
}

void SASStateChangeModel::mutex_proposition_constraint(const std::shared_ptr<AbstractTask> task,
                                                       std::vector<lp::LPConstraint> &constraints,
                                                       double infinity, int t_min, int t_max){
    
    
}

void SASStateChangeModel::flow_constraint(const std::shared_ptr<AbstractTask> task,
                                          std::vector<lp::LPConstraint> &constraints,
                                          double infinity, int t_min, int t_max){
    
    
}

void SASStateChangeModel::mutex_relaxtion_constraint(const std::shared_ptr<AbstractTask> task,
                                                     std::vector<lp::LPConstraint> &constraints,
                                                     double infinity, int t_min, int t_max){
    for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id){
        for (int op_mutex_id : numeric_task.get_mutex_actions(op_id)){
            for (int t = t_min; t < t_max; ++t){
                lp::LPConstraint constraint(-infinity, 1);
                constraint.insert((*index_opt)[op_id][t],1);
                constraint.insert((*index_opt)[op_mutex_id][t],1);
                constraints.push_back(constraint);
            }
        }
    }
    
}

//void SASStateChangeModel::print_solution(vector<double> &solution,const std::shared_ptr<AbstractTask> task){
//    TaskProxy task_proxy(*task);
//    cout << "solution " << endl;
//    OperatorsProxy ops = task_proxy.get_operators();
//    VariablesProxy vars = task_proxy.get_variables();
//        int i_var = 0;
//        for (VariableProxy var : vars) {
//            for (int t = 0; t < t_max; ++t){
//            int n_vals = var.get_domain_size();
//            for (int in_val = 0; in_val < n_vals; ++in_val) {
//                for (int out_val = 0; out_val < n_vals; ++out_val) {
//                    if (solution[index_var[i_var][in_val][out_val][t]] > 0){
//                        //cout << t << " : " << i_var << " " << in_val << " " << out_val << endl;
//                    }else{
//                        //cout << t << " n " << i_var << " " << in_val << " " << out_val << endl;
//
//                    }
//                }
//            }
//        }
//            ++i_var;
//    }
//    int steps = 0;
//    cout << "Solution found!" << endl;
//    cout << "Actual search time: 0.168001s [t=0.171312s]" << endl;
//    for (int t = 0; t < t_max; ++t){
//        for (int op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id) {
//            if (solution[(*index_opt)[op_id][t]] > 0){
//                //cout << t <<" " << solution[(*index_opt)[op_id][t]] << " " << ops[op_id].get_name() << endl;
//                if (ops[op_id].get_name().find("forget") == 0) continue;
//                cout << ops[op_id].get_name() << " (1)" << endl;
//                steps++;
//            }
//        }
//    }
//    cout << "Plan length: " << steps  << " step(s)." << endl;
//}


static shared_ptr<IPConstraintGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis(
                             "State based model",
                             "For each fact, a permanent constraint is added that considers the net "
                             "change of the fact, i.e., the total number of times the fact is added "
                             "minus the total number of times is removed. The bounds of each "
                             "constraint depend on the current state and the goal state and are "
                             "updated in each state. For details, see" + utils::format_paper_reference(
                                                                                                       {"Menkes van den Briel", "J. Benton", "Subbarao Kambhampati",
                                                                                                           "Thomas Vossen"},
                                                                                                       "An LP-based heuristic for optimal planning",
                                                                                                       "http://link.springer.com/chapter/10.1007/978-3-540-74970-7_46",
                                                                                                       "Proceedings of the Thirteenth International Conference on"
                                                                                                       " Principles and Practice of Constraint Programming (CP 2007)",
                                                                                                       "651-665",
                                                                                                       "2007") + utils::format_paper_reference(
                                                                                                                                               {"Blai Bonet"},
                                                                                                                                               "An admissible heuristic for SAS+ planning obtained from the"
                                                                                                                                               " state equation",
                                                                                                                                               "http://ijcai.org/papers13/Papers/IJCAI13-335.pdf",
                                                                                                                                               "Proceedings of the Twenty-Third International Joint"
                                                                                                                                               " Conference on Artificial Intelligence (IJCAI 2013)",
                                                                                                                                               "2268-2274",
                                                                                                                                               "2013") + utils::format_paper_reference(
                                                                                                                                                                                       {"Florian Pommerening", "Gabriele Roeger", "Malte Helmert",
                                                                                                                                                                                           "Blai Bonet"},
                                                                                                                                                                                       "LP-based Heuristics for Cost-optimal Planning",
                                                                                                                                                                                       "http://www.aaai.org/ocs/index.php/ICAPS/ICAPS14/paper/view/7892/8031",
                                                                                                                                                                                       "Proceedings of the Twenty-Fourth International Conference"
                                                                                                                                                                                       " on Automated Planning and Scheduling (ICAPS 2014)",
                                                                                                                                                                                       "226-234",
                                                                                                                                                                                       "AAAI Press 2014"));
    
    if (parser.dry_run())
        return nullptr;
    return make_shared<SASStateChangeModel>();
}

static PluginShared<IPConstraintGenerator> _plugin("sas", _parse);

