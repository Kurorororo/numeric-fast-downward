#include "sas_compilation.h"

#include "../globals.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../utils/markup.h"

using namespace std;
using namespace gurobi_ip_compilation;
using namespace numeric_helper;

GurobiSASStateChangeModel::GurobiSASStateChangeModel() : current_horizon(0) {}

GurobiSASStateChangeModel::~GurobiSASStateChangeModel() {
  for (auto v1 : y) {
    for (auto v2 : v1) {
      for (auto v3 : v2) {
        delete[] v3;
      }
    }
  }
}

void GurobiSASStateChangeModel::initialize(
    const int horizon, const std::shared_ptr<AbstractTask> task,
    std::shared_ptr<GRBModel> model, std::vector<GRBVar *> &x) {
  cout << "inigializing SAS SC" << endl;
  TaskProxy task_proxy(*task);
  numeric_task = NumericTaskProxy(task_proxy);
  OperatorsProxy ops = task_proxy.get_operators();
  VariablesProxy vars = task_proxy.get_variables();

  sc_actions.assign(vars.size(), vector<vector<set<int>>>());
  add_actions.assign(vars.size(), vector<set<int>>());
  pre_actions.assign(vars.size(), vector<set<int>>());

  int i_var = 0;
  for (VariableProxy var : vars) {
    int n_vals = var.get_domain_size();
    sc_actions[i_var].assign(n_vals, vector<set<int>>(n_vals, set<int>()));
    add_actions[i_var].assign(n_vals, set<int>());
    pre_actions[i_var].assign(n_vals, set<int>());
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
      effects[var] = post;
      if (pre != -1) {
        sc_actions[var][pre][post].insert(op_id);
      } else {
        add_actions[var][post].insert(op_id);
      }
    }
    for (FactProxy condition : op.get_preconditions()) {
      int var = condition.get_variable().get_id();
      if (numeric_task.is_numeric_axiom(var)) continue;
      int pre = condition.get_value();
      int post = effects[var];
      if (post == -1 || post == pre) {
        pre_actions[var][pre].insert(op_id);
      }
    }
  }
  update(horizon, task, model, x);
  initial_state_constraint(task, model);
}

void GurobiSASStateChangeModel::update(const int horizon,
                                       const std::shared_ptr<AbstractTask> task,
                                       std::shared_ptr<GRBModel> model,
                                       std::vector<GRBVar *> &x) {
  cout << "adding constraint from SAS SC" << endl;
  bool first = current_horizon == 0;
  int t_min = current_horizon;
  int t_max = horizon;
  add_variables(task, model, t_min, t_max);
  goal_state_constraint(task, model, t_max, first);
  update_state_change_constraint(task, model, t_min, t_max);
  precondition_constraint(task, model, x, t_min, t_max);
  effect_constraint(task, model, x, t_min, t_max);
  mutex_relaxtion_constraint(task, model, x, t_min, t_max);
  mutex_proposition_constraint(task, model, t_min, t_max);
  current_horizon = horizon;
}

void GurobiSASStateChangeModel::add_variables(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  numeric_task = NumericTaskProxy(task_proxy);
  VariablesProxy vars = task_proxy.get_variables();
  y.resize(t_max, std::vector<std::vector<GRBVar *>>(vars.size()));

  for (int t = t_min; t < t_max; ++t) {
    for (VariableProxy var : vars) {
      int n_vals = var.get_domain_size();
      std::vector<char> types(n_vals, GRB_BINARY);
      y[t][var.get_id()].resize(n_vals);

      for (int val = 0; val < n_vals; ++val) {
        y[t][var.get_id()][val] =
            model->addVars(NULL, NULL, NULL, types.data(), NULL, n_vals);
      }
    }
  }
}

void GurobiSASStateChangeModel::initial_state_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model) {
  TaskProxy task_proxy(*task);
  VariablesProxy vars = task_proxy.get_variables();

  State initial_state = task_proxy.get_initial_state();
  for (VariableProxy var : vars) {
    int i_var = var.get_id();
    int n_vals = var.get_domain_size();
    std::vector<double> coeffs(n_vals, 1);
    int initial_value = initial_state[i_var].get_value();
    GRBLinExpr lhs;
    lhs.addTerms(coeffs.data(), y[0][i_var][initial_value], n_vals);
    model->addConstr(lhs == 1);

    {
      for (int in_val = 0; in_val < n_vals; in_val++) {
        if (in_val == initial_value) continue;
        GRBLinExpr lhs;
        lhs.addTerms(coeffs.data(), y[0][i_var][in_val], n_vals);
        model->addConstr(lhs == 0);
      }
    }
  }
}

void GurobiSASStateChangeModel::goal_state_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    int t_max, bool first) {
  TaskProxy task_proxy(*task);
  VariablesProxy vars = task_proxy.get_variables();
  for (size_t id_goal = 0; id_goal < task_proxy.get_goals().size(); ++id_goal) {
    FactProxy goal = task_proxy.get_goals()[id_goal];
    if (numeric_task.is_numeric_axiom(goal.get_variable().get_id()) ||
        !numeric_task.numeric_goals_empty(id_goal))
      continue;  // this is a numeric goal
    std::string name = "SAS_goal_" + std::to_string(id_goal);
    if (!first) {
      GRBConstr constraint = model->getConstrByName(name);
      model->remove(constraint);
    }
    int var = goal.get_variable().get_id();
    int g_val = goal.get_value();
    int num_values = numeric_task.get_n_proposition_value(var);
    GRBLinExpr lhs;
    double coeff = 1;
    for (int val = 0; val < num_values; val++)
      lhs.addTerms(&coeff, &y[t_max - 1][var][val][g_val], 1);
    model->addConstr(lhs == 1, name);
  }
}

void GurobiSASStateChangeModel::update_state_change_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  for (int t = t_min; t < t_max - 1; ++t) {
    VariablesProxy vars = task_proxy.get_variables();
    for (VariableProxy var : vars) {
      int n_vals = var.get_domain_size();
      int i_var = var.get_id();
      std::vector<double> coeff(n_vals, 1);
      for (int val1 = 0; val1 < n_vals; val1++) {
        GRBLinExpr lhs;
        lhs.addTerms(coeff.data(), y[t + 1][i_var][val1], n_vals);
        GRBLinExpr rhs;
        for (int val2 = 0; val2 < n_vals; val2++) {
          rhs.addTerms(coeff.data(), &y[t][i_var][val2][val1], 1);
        }
        model->addConstr(lhs == rhs);
      }
    }
  }
}

void GurobiSASStateChangeModel::effect_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<GRBVar *> &x, int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  VariablesProxy vars = task_proxy.get_variables();
  for (VariableProxy var : vars) {
    int n_vals = var.get_domain_size();
    int i_var = var.get_id();
    for (int val1 = 0; val1 < n_vals; val1++) {
      // sc
      for (int val2 = 0; val2 < n_vals; val2++) {
        if (val1 == val2) continue;
        for (int t = t_min; t < t_max; ++t) {
          GRBLinExpr lhs;
          double coeff = 1;
          for (auto opt : sc_actions[i_var][val1][val2])
            lhs.addTerms(&coeff, &x[t][opt], 1);
          model->addConstr(lhs <= y[t][i_var][val1][val2]);
        }
      }

      // add
      for (auto opt : add_actions[i_var][val1]) {
        for (int t = t_min; t < t_max; ++t) {
          GRBLinExpr rhs;
          double coeff = 1;
          for (int val2 = 0; val2 < n_vals; val2++)
            rhs.addTerms(&coeff, &y[t][i_var][val2][val1], 1);
          model->addConstr(x[t][opt] <= rhs);
        }
      }

      // y
      for (int val2 = 0; val2 < n_vals; val2++) {
        if (val1 == val2) continue;
        for (int t = t_min; t < t_max; ++t) {
          GRBLinExpr rhs;
          double coeff = 1;
          for (auto opt : sc_actions[i_var][val1][val2])
            rhs.addTerms(&coeff, &x[t][opt], 1);
          for (auto opt : add_actions[i_var][val2])
            rhs.addTerms(&coeff, &x[t][opt], 1);
          model->addConstr(y[t][i_var][val1][val2] <= rhs);
        }
      }
    }
  }
}

void GurobiSASStateChangeModel::mutex_relaxtion_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<GRBVar *> &x, int t_min, int t_max) {
  for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id) {
    for (int op_mutex_id : numeric_task.get_mutex_actions(op_id)) {
      for (int t = t_min; t < t_max; ++t) {
        model->addConstr(x[t][op_id] + x[t][op_mutex_id] <= 1);
      }
    }
  }
}

void GurobiSASStateChangeModel::precondition_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<GRBVar *> &x, int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  VariablesProxy vars = task_proxy.get_variables();
  for (VariableProxy var : vars) {
    int n_vals = var.get_domain_size();
    int i_var = var.get_id();
    for (int val1 = 0; val1 < n_vals; val1++) {
      // prevail
      for (auto opt : pre_actions[i_var][val1]) {
        for (int t = t_min; t < t_max; ++t) {
          model->addConstr(x[t][opt] <= y[t][i_var][val1][val1]);
        }
      }
    }
  }
}

void GurobiSASStateChangeModel::mutex_proposition_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  VariablesProxy vars = task_proxy.get_variables();
  std::vector<int> var_id_to_domain_size(vars.size(), 0);
  for (VariableProxy var : vars) {
    var_id_to_domain_size[var.get_id()] = var.get_domain_size();
  }

  for (auto group : get_mutex_group()) {
    for (int t = t_min; t < t_max; ++t) {
      GRBLinExpr lhs;
      double coeff = 1;
      for (auto fact : group) {
        int var = fact.var;
        int val1 = fact.value;
        int n_val = var_id_to_domain_size[var];
        for (int val2 = 0; val2 < n_val; ++val2)
          lhs.addTerms(&coeff, &y[t][var][val2][val1], 1);
      }
      model->addConstr(lhs <= 1);
    }
  }
}

static shared_ptr<GurobiIPConstraintGenerator> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "SAS state change model",
      "For details, see" +
          utils::format_paper_reference(
              {"Menkes van den Briel", "Thomas Vossen", "Subbarao Kambhampati"},
              "Reviving integer programming approaches for AI planning: A "
              "branch-and-cut framework",
              "https://www.aaai.org/Papers/ICAPS/2005/ICAPS05-032.pdf",
              "Proceedings of the Fifteen International Conference on"
              " Automated Planning and Scheduling (ICAPS 2005)",
              "310-319", "2005"));

  if (parser.dry_run()) return nullptr;
  return make_shared<GurobiSASStateChangeModel>();
}

static PluginShared<GurobiIPConstraintGenerator> _plugin("sas", _parse);
