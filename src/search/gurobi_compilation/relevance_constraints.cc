#include "relevance_constraints.h"

#include <iostream>

#include "../option_parser.h"
#include "../plugin.h"

using namespace gurobi_ip_compilation;
using namespace numeric_helper;

void RelevanceConstraints::push_propositional(const TaskProxy &task,
                                              FactProxy f,
                                              std::queue<size_t> &open) {
  OperatorsProxy ops = task.get_operators();
  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    if (action_relevant[op_id]) continue;
    for (EffectProxy effect : ops[op_id].get_effects()) {
      if (f == effect.get_fact()) {
        action_relevant[op_id] = true;
        open.push(op_id);
        break;
      }
    }
  }
}

void RelevanceConstraints::push_numeric(NumericTaskProxy &numeric_task, int c,
                                        std::queue<size_t> &open) {
  size_t n_actions = numeric_task.get_n_actions();
  for (size_t op_id = 0; op_id < n_actions; ++op_id) {
    if (action_relevant[op_id]) continue;
    LinearNumericCondition &lnc = numeric_task.get_condition(c);
    ap_float net = 0;
    for (size_t var = 0; var < numeric_task.get_n_numeric_variables(); ++var) {
      net +=
          lnc.coefficients[var] * numeric_task.get_action_eff_list(op_id)[var];
    }

    if (net > 0.0) {
      action_relevant[op_id] = true;
      open.push(op_id);
    } else {
      for (int i = 0; i < numeric_task.get_action_num_linear_eff(op_id); ++i) {
        int lhs = numeric_task.get_action_linear_lhs(op_id)[i];
        if (fabs(lnc.coefficients[lhs]) > 0) {
          action_relevant[op_id] = true;
          open.push(op_id);
          break;
        }
      }
    }
  }
}

void RelevanceConstraints::analyze_relevance(
    const std::shared_ptr<AbstractTask> task, bool use_linear_effects) {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  NumericTaskProxy numeric_task(task_proxy, true, use_linear_effects);
  action_relevant = std::vector<bool>(ops.size(), false);

  std::queue<size_t> open;

  for (FactProxy g : task_proxy.get_goals())
    push_propositional(task_proxy, g, open);

  for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals();
       ++id_goal) {
    for (int g : numeric_task.get_numeric_goals(id_goal)) {
      push_numeric(numeric_task, g, open);
    }
  }

  while (!open.empty()) {
    size_t op_id = open.front();
    open.pop();
    for (FactProxy pre : ops[op_id].get_preconditions())
      push_propositional(task_proxy, pre, open);
    for (int pre : numeric_task.get_action_num_list(op_id))
      for (int i : numeric_task.get_numeric_conditions_id(pre))
        push_numeric(numeric_task, i, open);
  }
}

void RelevanceConstraints::initialize(
    const int horizon, const std::shared_ptr<AbstractTask> task,
    std::shared_ptr<GRBModel> model, std::vector<std::vector<GRBVar>> &x,
    std::vector<std::vector<bool>> &action_mutex, bool use_linear_effects) {
  std::cout << "initialize relevance" << std::endl;
  analyze_relevance(task, use_linear_effects);
  update(horizon, task, model, x);
}

void RelevanceConstraints::update(const int horizon,
                                  const std::shared_ptr<AbstractTask> task,
                                  std::shared_ptr<GRBModel> model,
                                  std::vector<std::vector<GRBVar>> &x) {
  std::cout << "adding costraints from relevance" << std::endl;
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();

  int t_min = current_horizon;
  int t_max = horizon;

  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    if (!action_relevant[op_id]) {
      for (int t = t_min; t < t_max; ++t) {
        model->addConstr(x[t][op_id] == 0);
      }
    }
  }

  current_horizon = horizon;
}

static shared_ptr<GurobiIPConstraintGenerator> _parse(OptionParser &parser) {
  parser.document_synopsis("Relevance constraints.", "");

  if (parser.dry_run()) return nullptr;
  return make_shared<RelevanceConstraints>();
}

static PluginShared<GurobiIPConstraintGenerator> _plugin("relevance", _parse);
