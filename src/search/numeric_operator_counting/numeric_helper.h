#ifndef numeric_helper_h
#define numeric_helper_h

#include <iostream>
#include <set>
#include <vector>
#include <list>
#include <unordered_map>
#include <cmath>
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"

using namespace std;

namespace numeric_helper {
    
    /* An action is an operator where effects are espressed as add and eff of proposition.
     A proposition is an atom of the form Var = Val */
    
    struct Action {
        std::set<int> pre_list;
        std::set<int> num_list; // numeric preconditions
        std::set<int> add_list;
        std::set<int> del_list;
        std::set<int> pre_del_list;
        std::vector<double> eff_list; // numeric effects
        std::set<int> possible_add_list; // numeric effects on conditions (id condition is already con + n_propositions)
        int cost = 1;
        Action(int size_eff) {
            eff_list = std::vector<double>(size_eff,0);
        }
        
        ~Action() = default;
    };
    
    /* Linear Numeric Conditions */
    
    struct LinearNumericCondition {
        LinearNumericCondition(std::vector<double> & c, double k) : coefficients(c), constant(k), is_strictly_greater(false) { }
        LinearNumericCondition(int size_coefficients) {
            coefficients.assign(size_coefficients,0);
            constant = 0;
            is_strictly_greater = false;
        }
        std::vector<double> coefficients;
        double constant;
        bool is_strictly_greater;
        LinearNumericCondition operator + (const LinearNumericCondition &lnc) const{
            std::vector<double> _coefficients(coefficients.size());
            double _constant;
            for (size_t num_id = 0; num_id < coefficients.size(); ++num_id){
                _coefficients[num_id] = coefficients[num_id] + lnc.coefficients[num_id];
            }
            _constant = constant + lnc.constant;
            return LinearNumericCondition(_coefficients,_constant);
        }
        
        LinearNumericCondition operator - (const LinearNumericCondition &lnc) const{
            std::vector<double> _coefficients(coefficients.size());
            double _constant;
            for (size_t num_id = 0; num_id < coefficients.size(); ++num_id){
                _coefficients[num_id] = coefficients[num_id] - lnc.coefficients[num_id];
            }
            _constant = constant - lnc.constant;
            return LinearNumericCondition(_coefficients,_constant);
        }
        
        bool simple_condition(int v){
            assert(v<coefficients.size());
            for (size_t c_id = 0; c_id < coefficients.size();++c_id){
                if (v != c_id && fabs(coefficients[c_id]) > 0.00001) return false;
            }
            return true;
        }
        
        bool dominate(LinearNumericCondition &other) const;
        
        bool empty() const;

        
    };
    
    std::ostream& operator<<(std::ostream& os, const LinearNumericCondition& lnc);
    
    /* Linear Numeric Conditions */
    struct NumericVariable {
        int id_var;
        int id_abstract_task;
        double upper_bound;
        double lower_bound;
        NumericVariable(int id_, int id_at, double lb_, double ub_);
    };
    
    /* NumericTaskProxy */
    class NumericTaskProxy{
    public:
        NumericTaskProxy (const TaskProxy &task, bool additional = true);
        NumericTaskProxy () { };
        size_t get_n_actions() { return n_actions; }
        size_t get_n_propositions() { return n_propositions; }
        size_t get_n_conditions() { return n_conditions; }
        size_t get_n_numeric_variables() { return n_numeric_variables; }
        size_t get_n_numeric_goals() { return numeric_goals.size(); }
        size_t get_n_vars() { return  n_vars; }
        int get_n_proposition_value(int var) { return  propositions[var].size(); }
        int get_proposition(int var, int val) { return propositions[var][val]; }
        std::pair<int,int> get_var_val(int p) { return propositions_inv[p]; }
        std::set<int> get_add_actions(int var, int val) { return add_effects[var][val]; }
        bool numeric_goals_empty(int id_goal){ return numeric_goals[id_goal].empty(); }
        std::set<int> & get_action_pre_list(int op_id) { return actions[op_id].pre_list; }
        std::set<int> & get_action_add_list(int op_id) { return actions[op_id].add_list; }
        std::set<int> & get_action_num_list(int op_id) { return actions[op_id].num_list; }
        std::set<int> & get_action_possible_add_list(int op_id) { return actions[op_id].possible_add_list; }
        std::set<int> & get_action_del_list(int op_id) { return actions[op_id].del_list; }
        std::set<int> & get_action_pre_del_list(int op_id) { return actions[op_id].pre_del_list; }
        std::vector<double> & get_action_eff_list(int op_id) { return actions[op_id].eff_list; }
        int get_action_cost(int op_id) { return actions[op_id].cost; }
        LinearNumericCondition & get_condition(int cond_id) { return numeric_conditions[cond_id]; }
        list<int> & get_numeric_goals(int id_goal) { return numeric_goals[id_goal]; }
        int get_n_numeric_conditions() { return numeric_conditions.size(); }
        list<int> & get_numeric_conditions_id(int pre_id) { return numeric_conditions_id[pre_id]; }
        NumericVariable & get_numeric_variable(int num_id) {return numeric_variables[num_id]; }
        bool is_numeric_axiom(int var_id) {
            //if (var_id >= fact_to_axiom_map.size()) exit(2);
            return fact_to_axiom_map[var_id]!=-1;
            
        }
        int get_numeric_axiom(int var_id) { return fact_to_axiom_map[var_id]; }
        int get_var(int p) { return map_vars[p];}
        set<int> & get_achievers(int fact_id) { return achievers[fact_id];}
        string & get_proposition_name(int p_id) { return proposition_names[p_id]; }
        double get_small_m(int p_id) { return small_m[p_id]; }
        double get_epsilon(int p_id) { return epsilon[p_id]; }
        set<int> & get_mutex_actions(int op_id) { return mutex_actions[op_id]; }
        bool get_dominance(int i, int j) { return dominance_conditions[i][j]; }
        static bool redundant_constraints;
  private:
        //TaskProxy task;
        void build_numeric_variables(const TaskProxy &task_proxy);
        void build_artificial_variables(const TaskProxy &task_proxy);
        void build_numeric_conditions(const TaskProxy &task_proxy);
        void build_propositions(const TaskProxy &task_proxy);
        void build_actions(const TaskProxy &task_proxy);
        void build_mutex_actions(const TaskProxy &task_proxy);
        void build_numeric_goals(const TaskProxy &task_proxy);
        void generate_possible_achievers(const TaskProxy &task_proxy);
        void calculates_bounds_numeric_variables();
        void calculates_small_m_and_epsilons();
        void calculates_dominance();

        // numeric variables
        size_t n_numeric_variables; // number of real numeric variables
        std::vector<NumericVariable> numeric_variables;
        std::vector<LinearNumericCondition> artificial_variables;
        std::vector<int> id_numeric_variable_inv; // inverse of id_numeric_variable

        // numeric conditions
        size_t n_conditions;
        std::vector<LinearNumericCondition> numeric_conditions; // normalized numeric conditions
        std::vector<std::list<int>> numeric_conditions_id; // normalized numeric conditions
        std::vector<int> fact_to_axiom_map; // check if a fact is due to a comparison axiom
        //propositions
        std::vector<std::vector<int>> propositions;
        std::vector<std::pair<int,int>> propositions_inv;
        std::vector<int> map_vars;
        size_t n_propositions;
        size_t n_vars; // number of sas+ variables

        //actions
        std::vector<Action> actions;
        std::vector<std::vector<std::set<int>>> add_effects; // given a proposition (var, value), returns the set of actions that add the proposition;
        size_t n_actions;

        // numeric variables
        std::vector<std::list<int>> numeric_goals; //indexL goal, value: -1 if it's a propositional goal, id of numeric condition if it's a goal condition
        
        // achievers
        std::vector<set<int>> achievers; // index fact, add, set of actions that are adding the fact
        
        std::vector<std::string> proposition_names;
        std::vector<double> small_m; //index condition id, small m
        std::vector<double> epsilon; // index condition id, value = 0, if strictly greater, 0, otherwise;
        
        // mutex actions
        std::vector<set<int>> mutex_actions;
        
        std::vector<std::vector<bool>> dominance_conditions;

   };
}
#endif /* numeric_helper_h */
