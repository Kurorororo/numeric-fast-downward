#include "../globals.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"

#include "../lp/lp_solver.h"

#include "../utils/markup.h"
#include "numeric_constraints.h"
#include "../numeric_operator_counting/numeric_helper.h"

using namespace std;
using namespace operator_counting;
using namespace numeric_helper;

void NumericConstraints::initialize_variables(const std::shared_ptr<AbstractTask> task,
                                           std::vector<lp::LPVariable> &variables,
                                           double infinity){
    cout << "initializing variables for numeric part" << endl;
    TaskProxy task_proxy(*task);
    numeric_task = NumericTaskProxy(task_proxy);
    int n_numeric_vars = numeric_task.get_n_numeric_variables();
    index_numeric_var.assign(n_numeric_vars, vector<int> (t_max,-1));
    
    add_variables(task,variables,infinity,t_min,t_max);
}

void NumericConstraints::add_variables(
                                    const std::shared_ptr<AbstractTask> task,
                                    std::vector<lp::LPVariable> &variables,
                                    double infinity, int t_min, int t_max){
    TaskProxy task_proxy(*task);
    numeric_task = NumericTaskProxy(task_proxy);
    int n_numeric_vars = numeric_task.get_n_numeric_variables();
    
    // resize index
    for (int nv_id = 0; nv_id < n_numeric_vars; ++nv_id) {
        index_numeric_var[nv_id].resize(t_max,-1);
    }
    
    // add variables
    for (int t = t_min; t < t_max; ++t) {
        // actions
        for (int nv_id = 0; nv_id < n_numeric_vars; ++nv_id) {

            stringstream name;
            name << "v_" << nv_id << "_" << t;
            index_numeric_var[nv_id][t] = variables.size();
            variables.push_back(lp::LPVariable(-infinity,infinity,0, name.str(), lp::LPVariableType::continous));
        }
    }
}

void NumericConstraints::initialize_constraints(const std::shared_ptr<AbstractTask> task,
                                             std::vector<lp::LPConstraint> &constraints,
                                             double infinity){
    
    cout << "initializing constraints for SB" << endl;
    initial_state_constraint(task, constraints);
    goal_state_constraint(task, constraints, infinity, t_max);
    action_precondition_constraint(task, constraints, infinity,t_min,t_max);
    action_simple_effect_constraint(task, constraints, infinity,t_min,t_max);
    action_linear_effect_constraint(task, constraints, infinity,t_min,t_max);
//    mutex_relaxtion_constraint(task, constraints, infinity,t_min,t_max);
//    noop_preconditions_constraint(task, constraints, infinity,t_min,t_max);
//    noopt_mutex_constraint(task, constraints, infinity,t_min,t_max);
}

bool NumericConstraints::update_constraints(const State &state, lp::LPSolver &lp_solver){
    return false;
}

bool NumericConstraints::update_constraints(const int horizon,
                                         lp::LPSolver &lp_solver,const std::shared_ptr<AbstractTask> task,
                                         std::vector<lp::LPVariable> &variables,
                                         double infinity, std::vector<lp::LPConstraint> & constraints){
    cout << "adding constraint from SB" << endl;
    t_min = t_max;
    t_max = horizon;
    add_variables(task,variables,infinity,t_min,t_max);
    action_precondition_constraint(task, constraints, infinity,t_min,t_max);
    action_simple_effect_constraint(task, constraints, infinity,t_min-1,t_max);
    action_linear_effect_constraint(task, constraints, infinity,t_min-1,t_max);

//    mutex_relaxtion_constraint(task, constraints, infinity,t_min,t_max);
//    noop_preconditions_constraint(task, constraints, infinity,t_min,t_max);
//    noopt_mutex_constraint(task, constraints, infinity,t_min,t_max);
    goal_state_constraint(task, constraints, infinity, t_max);
    return false;
}

void NumericConstraints::initial_state_constraint(const std::shared_ptr<AbstractTask> task,
                                               std::vector<lp::LPConstraint> &constraints){
    
    TaskProxy task_proxy(*task);
    State initial_state =  task_proxy.get_initial_state();
    int n_numeric_vars = numeric_task.get_n_numeric_variables();
    for (int nv_id = 0; nv_id < n_numeric_vars; ++nv_id) {
        int id_num = numeric_task.get_numeric_variable(nv_id).id_abstract_task;
        double initial_value = initial_state.nval(id_num);
        lp::LPConstraint constraint(initial_value, initial_value);
        constraint.insert(index_numeric_var[nv_id][0], 1.);
        constraints.push_back(constraint);
        
    }
}

void NumericConstraints::action_precondition_constraint(const std::shared_ptr<AbstractTask> task,
                                                     std::vector<lp::LPConstraint> &constraints,
                                                     double infinity, int t_min, int t_max){
    
   const double big_m = 100000;
    for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id){
        for (int pre : numeric_task.get_action_num_list(op_id)){
            for (int i : numeric_task.get_numeric_conditions_id(pre)){
                for (int t = t_min; t < t_max; ++t){

                    LinearNumericCondition& lnc = numeric_task.get_condition(i);
                    double rhs = numeric_task.get_epsilon(i) - big_m - lnc.constant;
                    lp::LPConstraint constraint(rhs,infinity);
                    constraint.insert((*index_opt)[op_id][t], -big_m);
                    for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                        double coefficient = lnc.coefficients[n_id];
                        if (fabs(coefficient)>0)
                            constraint.insert(index_numeric_var[n_id][t],coefficient);
                    }
                    if (!constraint.empty()) {
                        constraints.push_back(constraint);
                    }
                }
            }
        }
    }
}

void NumericConstraints::action_simple_effect_constraint(const std::shared_ptr<AbstractTask> task,
                                               std::vector<lp::LPConstraint> &constraints,
                                               double infinity, int t_min, int t_max){
    
    for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
        for (int t = t_min; t < t_max-1; ++t){
            lp::LPConstraint constraint_less(0., infinity);
            lp::LPConstraint constraint_great(0., infinity);
            constraint_less.insert(index_numeric_var[n_id][t+1],1);
            constraint_less.insert(index_numeric_var[n_id][t],-1);
            constraint_great.insert(index_numeric_var[n_id][t+1],-1);
            constraint_great.insert(index_numeric_var[n_id][t],1);
            for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id){
                constraint_less.insert((*index_opt)[op_id][t], -numeric_task.get_action_eff_list(op_id)[n_id]);
                constraint_great.insert((*index_opt)[op_id][t], numeric_task.get_action_eff_list(op_id)[n_id]);
            }
            constraints.push_back(constraint_less);
            constraints.push_back(constraint_great);
        }
    }
}

void NumericConstraints::action_linear_effect_constraint(const std::shared_ptr<AbstractTask> task,
                                                         std::vector<lp::LPConstraint> &constraints,
                                                         double infinity, int t_min, int t_max){
    
    //    for (size_t fact_id = 0; fact_id < numeric_task.get_n_propositions(); ++fact_id){
    //        set<int> achievers = numeric_task.get_achievers(fact_id);
    //        for(int t = t_min; t < t_max-1; ++t){
    //            lp::LPConstraint constraint(0., infinity);
    //            constraint.insert(index_fact[fact_id][t+1], -1.);
    //            for (int a : achievers) {
    //                constraint.insert((*index_opt)[a][t], 1.);
    //            }
    //            constraint.insert(index_noop_fact[fact_id][t],1.);
    //            constraints.push_back(constraint);
    //        }
    //    }
}


void NumericConstraints::goal_state_constraint(const std::shared_ptr<AbstractTask> task,
                                            std::vector<lp::LPConstraint> &constraints,double infinity,
                                            int t_max) {
    TaskProxy task_proxy(*task);
    static bool first = goal_index.empty();
    if (first){
        int n_goals = 0;
        for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
            list<int> numeric_goals = numeric_task.get_numeric_goals(id_goal);
            if (numeric_goals.empty()) continue; // this is not a numeric goal
            for (int id_n_con : numeric_goals){
                n_goals++;
            }
        }
        goal_index.assign(n_goals,-1);
    }
    int i_goal = 0;
    //cout << "numeric goals " << numeric_task.get_n_numeric_goals() << endl;
    for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
        list<int> numeric_goals = numeric_task.get_numeric_goals(id_goal);
        if (numeric_goals.empty()) continue; // this is not a numeric goal
        for (int id_n_con : numeric_goals){
            LinearNumericCondition& lnc = numeric_task.get_condition(id_n_con);
            lp::LPConstraint constraint(-lnc.constant,infinity);
            for (size_t n_id = 0; n_id < numeric_task.get_n_numeric_variables(); ++n_id){
                double coefficient = lnc.coefficients[n_id];
                //if (fabs(coefficient)>0.00001)
                constraint.insert(index_numeric_var[n_id][t_max-1],coefficient);
            }
            if (!constraint.empty()) {
                if (first){
                    goal_index[i_goal]= constraints.size();
                    constraints.push_back(constraint);
                }else{
                    constraints[goal_index[i_goal]] = constraint;
                }
            }
            i_goal++;
        }
    }
    if (first) first = false;
}

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
    return make_shared<NumericConstraints>();
}

static PluginShared<IPConstraintGenerator> _plugin("numeric", _parse);

