#ifndef TASKS_EXPLICIT_TASK_H
#define TASKS_EXPLICIT_TASK_H

#include "delegating_task.h"

#include "../utils/collections.h"

#include <cassert>
#include <set>
#include <map>

namespace tasks {
struct ExplicitEffect;
struct ExplicitOperator;
struct ExplicitVariable;

/*
  Task transformation that holds all task information itself.

  Instead of asking parent transformations for data and possibly
  converting it, this class holds all task data internally.

  When querying and transforming data from the parent(s) is more
  expensive than storing it in the task, this class can be used to
  "start over" in the task transformation hierarchy.

  While this class implements all methods for accessing task data, it
  doesn't provide implementations for the two state and operator
  conversion methods. This is to allow inheriting classes to decide how
  to make these conversions.
*/
class ExplicitTask : public DelegatingTask {
    bool run_sanity_check() const;

protected:
    const std::vector<ExplicitVariable> variables;
    const std::vector<std::vector<std::set<Fact>>> mutexes;
    const std::vector<ExplicitOperator> operators;
    const std::vector<ExplicitOperator> axioms;
    const std::vector<int> initial_state_values;
    const std::vector<Fact> goals;

    const ExplicitVariable &get_variable(int var) const;
    const ExplicitEffect &get_effect(int op_id, int effect_id, bool is_axiom) const;
    const ExplicitOperator &get_operator_or_axiom(int index, bool is_axiom) const;
    
    std::map<int,int> operator_map;
public:
    ExplicitTask(
        const std::shared_ptr<AbstractTask> &parent,
        std::vector<ExplicitVariable> &&variables,
        std::vector<std::vector<std::set<Fact>>> &&mutexes,
        std::vector<ExplicitOperator> &&operators,
        std::vector<ExplicitOperator> &&axioms,
        std::vector<int> &&initial_state_values,
        std::vector<Fact> &&goals
                 );

    virtual int get_num_variables() const override;
    virtual const std::string &get_variable_name(int var) const override;
    virtual int get_variable_domain_size(int var) const override;
    //virtual int get_variable_axiom_layer(int var) const override;
    //virtual int get_variable_default_axiom_value(int var) const override;
    virtual const std::string &get_fact_name(const Fact &fact) const override;
    virtual bool are_facts_mutex(
         const Fact &fact1, const Fact &fact2) const override;

    virtual ap_float get_operator_cost(int index, bool is_axiom) const override;
    virtual const std::string &get_operator_name(int index, bool is_axiom) const override;

    virtual int get_num_operators() const override;
    virtual int get_num_operator_preconditions(
        int index, bool is_axiom) const override;
    virtual Fact get_operator_precondition(
        int op_index, int fact_index, bool is_axiom) const override;
    virtual int get_num_operator_effects(
        int op_index, bool is_axiom) const override;
    virtual int get_num_operator_ass_effects(int op_index, bool is_axiom) const override;
    virtual int get_num_operator_ass_effect_conditions(
                                                       int op_index, int ass_eff_index, bool is_axiom) const override;
    virtual int get_num_operator_effect_conditions(
        int op_index, int eff_index, bool is_axiom) const override;
    virtual Fact get_operator_effect_condition(
        int op_index, int eff_index, int cond_index, bool is_axiom) const override;
    virtual Fact get_operator_effect(
        int op_index, int eff_index, bool is_axiom) const override;
    virtual AssEffect get_operator_ass_effect(int op_index, int eff_index, bool is_axiom) const override;
    virtual const GlobalOperator *get_global_operator(
        int index, bool is_axiom) const override = 0;

    virtual int get_num_axioms() const override;

    virtual int get_num_goals() const override;
    virtual Fact get_goal_fact(int index) const override;

    virtual std::vector<int> get_initial_state_values() const override;
    //virtual void convert_state_values_from_parent(
    //    std::vector<int> &values) const override = 0;
};


struct ExplicitVariable {
    const int domain_size;
    const std::string name;
    const std::vector<std::string> fact_names;
    const int axiom_layer;
    const int axiom_default_value;

    ExplicitVariable(
        int domain_size,
        std::string &&name,
        std::vector<std::string> &&fact_names,
        int axiom_layer,
        int axiom_default_value);
};


struct ExplicitEffect {
    const Fact fact;
    const std::vector<Fact> conditions;

    ExplicitEffect(int var, int value, std::vector<Fact> &&conditions);
};


struct ExplicitOperator {
    const std::vector<Fact> preconditions;
    const std::vector<ExplicitEffect> effects;
    const int cost;
    const std::string name;
    const bool is_an_axiom;
    const int copy_id;
    ExplicitOperator(
        std::vector<Fact> &&preconditions,
        std::vector<ExplicitEffect> &&effects,
        int cost,
        const std::string &name,
        bool is_an_axiom,
        int copy_id
        );
};


inline const ExplicitVariable &ExplicitTask::get_variable(int var) const {
    assert(utils::in_bounds(var, variables));
    return variables[var];
}

inline const ExplicitEffect &ExplicitTask::get_effect(
    int op_id, int effect_id, bool is_axiom) const {
    const ExplicitOperator &op = get_operator_or_axiom(op_id, is_axiom);
    assert(utils::in_bounds(effect_id, op.effects));
    return op.effects[effect_id];
}

inline const ExplicitOperator &ExplicitTask::get_operator_or_axiom(
    int index, bool is_axiom) const {
    if (is_axiom) {
        assert(utils::in_bounds(index, axioms));
        return axioms[index];
    } else {
        assert(utils::in_bounds(index, operators));
        return operators[index];
    }
}
}

#endif
