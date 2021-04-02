#include "numeric_constraints.h"

#include "../globals.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../utils/markup.h"

using namespace std;
using namespace gurobi_ip_compilation;
using namespace numeric_helper;

NumericConstraints::NumericConstraints(const Options &opts)
    : current_horizon(0), num_repetition(opts.get<int>("num_repetition")) {}

void NumericConstraints::initialize(const int horizon,
                                    const std::shared_ptr<AbstractTask> task,
                                    std::shared_ptr<GRBModel> model,
                                    std::vector<std::vector<GRBVar>> &x) {
  cout << "initializing numeric" << endl;
  TaskProxy task_proxy(*task);
  numeric_task = NumericTaskProxy(task_proxy);
  initialize_numeric_mutex();

  if (num_repetition > 1) initialize_repetable_actions(x);

  update(horizon, task, model, x);
  initial_state_constraint(task, model);
}

void NumericConstraints::update(const int horizon,
                                const std::shared_ptr<AbstractTask> task,
                                std::shared_ptr<GRBModel> model,
                                std::vector<std::vector<GRBVar>> &x) {
  cout << "adding constraint from numeric" << endl;
  bool first = current_horizon == 0;
  int t_min = current_horizon;
  int t_max = horizon;
  compute_big_m_values(task, t_min, t_max);
  add_variables(task, model, t_min, t_max);
  modify_x(task, model, x, t_min, t_max);
  goal_state_constraint(task, model, t_max, first);
  precondition_constraint(task, model, x, t_min, t_max);
  simple_effect_constraint(task, model, x, t_min, t_max);
  linear_effect_constraint(task, model, x, t_min, t_max);
  mutex_relaxtion_constraint(task, model, x, t_min, t_max);
  current_horizon = horizon;
}

void NumericConstraints::initialize_numeric_mutex() {
  size_t n_actions = numeric_task.get_n_actions();
  numeric_mutex.resize(n_actions, std::vector<bool>(n_actions, false));
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  for (size_t op_id1 = 0; op_id1 < n_actions; ++op_id1) {
    for (size_t op_id2 = 0; op_id2 < n_actions; ++op_id2) {
      if (op_id1 == op_id2 || numeric_mutex[op_id1][op_id2]) continue;
      for (int pre : numeric_task.get_action_num_list(op_id1)) {
        for (int i : numeric_task.get_numeric_conditions_id(pre)) {
          LinearNumericCondition &lnc = numeric_task.get_condition(i);
          ap_float net = 0;
          for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
            net += lnc.coefficients[nv_id] *
                   numeric_task.get_action_eff_list(op_id2)[nv_id];
          }

          if (net < 0.0) {
            numeric_mutex[op_id1][op_id2] = true;
            numeric_mutex[op_id2][op_id1] = true;
            break;
          }
        }
        if (numeric_mutex[op_id1][op_id2]) break;
      }
    }
  }
}

void NumericConstraints::initialize_repetable_actions(
    std::vector<std::vector<GRBVar>> &x) {
  size_t n_actions = numeric_task.get_n_actions();
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  repetable.resize(n_actions, false);

  for (size_t op_id = 0; op_id < n_actions; ++op_id) {
    if (numeric_task.get_action_pre_del_list(op_id).size() > 0) continue;
    bool is_repetable = true;
    for (int pre : numeric_task.get_action_num_list(op_id)) {
      for (int i : numeric_task.get_numeric_conditions_id(pre)) {
        LinearNumericCondition &lnc = numeric_task.get_condition(i);
        ap_float net = 0;
        ap_float coefficent_sum = 0.0;
        for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
          net += lnc.coefficients[nv_id] *
                 numeric_task.get_action_eff_list(op_id)[nv_id];
          coefficent_sum += lnc.coefficients[nv_id];
        }

        if (net < 0.0 || coefficent_sum == 0.0) {
          is_repetable = false;
          break;
        }
      }
      if (!is_repetable) break;
    }

    repetable[op_id] = is_repetable;
  }
}

void NumericConstraints::compute_big_m_values(
    const std::shared_ptr<AbstractTask> task, int t_min, int t_max) {
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  large_m.resize(t_max, std::vector<double>(n_numeric_variables, 0.0));
  small_m.resize(t_max, std::vector<double>(n_numeric_variables, 0.0));
  k_over.resize(t_max, std::vector<double>(n_numeric_variables, 0.0));
  k_under.resize(t_max, std::vector<double>(n_numeric_variables, 0.0));

  size_t n_actions = numeric_task.get_n_actions();
  TaskProxy task_proxy(*task);
  State initial_state = task_proxy.get_initial_state();
  for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
    int id_num = numeric_task.get_numeric_variable(nv_id).id_abstract_task;
    double initial_value = initial_state.nval(id_num);
    large_m[0][nv_id] = initial_value;
    small_m[0][nv_id] = initial_value;
  }

  for (int t = std::max(t_min, 1); t < t_max; ++t) {
    for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
      k_over[t][nv_id] = large_m[t - 1][nv_id];
      k_under[t][nv_id] = small_m[t - 1][nv_id];
      for (size_t op_id = 0; op_id < n_actions; ++op_id) {
        double k = numeric_task.get_action_eff_list(op_id)[nv_id];
        if (k > 0.0) {
          if (num_repetition > 1 && repetable[op_id])
            k_over[t][nv_id] += num_repetition * k;
          else
            k_over[t][nv_id] += k;
        } else {
          if (num_repetition > 1 && repetable[op_id])
            k_under[t][nv_id] += num_repetition * k;
          else
            k_under[t][nv_id] += k;
        }
      }
      large_m[t][nv_id] = k_over[t][nv_id];
      small_m[t][nv_id] = k_under[t][nv_id];
    }
  }
}

void NumericConstraints::add_variables(const std::shared_ptr<AbstractTask> task,
                                       std::shared_ptr<GRBModel> model,
                                       int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  int n_numeric_vars = numeric_task.get_n_numeric_variables();
  std::vector<char> types(n_numeric_vars, GRB_CONTINUOUS);
  y.resize(t_max, std::vector<GRBVar>(n_numeric_vars));

  // add decision variables
  for (int t = t_min; t < t_max; ++t) {
    // numeric variables
    for (int var = 0; var < n_numeric_vars; ++var) {
      std::string name = "y^" + std::to_string(var) + "_" + std::to_string(t);
      y[t][var] = model->addVar(small_m[t][var], large_m[t][var], 0,
                                GRB_CONTINUOUS, name);
    }
  }
}

void NumericConstraints::modify_x(const std::shared_ptr<AbstractTask> task,
                                  std::shared_ptr<GRBModel> model,
                                  std::vector<std::vector<GRBVar>> &x,
                                  int t_min, int t_max) {
  TaskProxy task_proxy(*task);
  if (num_repetition > 1) {
    OperatorsProxy ops = task_proxy.get_operators();
    for (int t = t_min; t < t_max; ++t) {
      for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
        if (repetable[op_id]) {
          x[t][op_id].set(GRB_CharAttr_VType, GRB_INTEGER);
          x[t][op_id].set(GRB_DoubleAttr_UB, num_repetition);
        }
      }
    }
  }
}

void NumericConstraints::initial_state_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model) {
  TaskProxy task_proxy(*task);
  State initial_state = task_proxy.get_initial_state();
  int n_numeric_vars = numeric_task.get_n_numeric_variables();
  for (int nv_id = 0; nv_id < n_numeric_vars; ++nv_id) {
    int id_num = numeric_task.get_numeric_variable(nv_id).id_abstract_task;
    double initial_value = initial_state.nval(id_num);
    model->addConstr(y[0][nv_id] == initial_value);
  }
}

void NumericConstraints::precondition_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) {
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id) {
    for (int pre : numeric_task.get_action_num_list(op_id)) {
      for (int i : numeric_task.get_numeric_conditions_id(pre)) {
        for (int t = t_min; t < t_max; ++t) {
          LinearNumericCondition &lnc = numeric_task.get_condition(i);
          double big_m = lnc.constant - numeric_task.get_epsilon(i);

          for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
            double w = lnc.coefficients[nv_id];
            if (w < 0.0) {
              big_m += w * large_m[t][nv_id];
            } else {
              big_m += w * small_m[t][nv_id];
            }
          }

          GRBLinExpr rhs(big_m * (1 - x[t][op_id]));
          GRBLinExpr lhs(lnc.constant);

          for (size_t var = 0; var < numeric_task.get_n_numeric_variables();
               ++var) {
            double coefficient = lnc.coefficients[var];
            if (fabs(coefficient) > 0)
              lhs.addTerms(&coefficient, &y[t][var], 1);
          }

          model->addConstr(lhs >= rhs + numeric_task.get_epsilon(i));
        }
      }
    }
  }
}

void NumericConstraints::simple_effect_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) {
  for (size_t var = 0; var < numeric_task.get_n_numeric_variables(); ++var) {
    for (int t = std::max(0, t_min - 1); t < t_max - 1; ++t) {
      GRBLinExpr rhs(y[t][var]);
      for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id) {
        double coefficient = numeric_task.get_action_eff_list(op_id)[var];
        if (fabs(coefficient) > 0) rhs.addTerms(&coefficient, &x[t][op_id], 1);
      }
      model->addConstr(y[t + 1][var] >= rhs);
      model->addConstr(y[t + 1][var] <= rhs);
    }
  }
}

void NumericConstraints::linear_effect_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) {}

void NumericConstraints::goal_state_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    int t_max, bool first) {
  TaskProxy task_proxy(*task);
  for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals();
       ++id_goal) {
    list<int> numeric_goals = numeric_task.get_numeric_goals(id_goal);
    if (numeric_goals.empty()) continue;  // this is not a numeric goal
    for (int id_n_con : numeric_goals) {
      std::string name = "numeric_goal_" + std::to_string(id_goal) + "_" +
                         std::to_string(id_n_con);
      if (!first) {
        GRBConstr constraint = model->getConstrByName(name);
        model->remove(constraint);
      }
      LinearNumericCondition &lnc = numeric_task.get_condition(id_n_con);
      GRBLinExpr lhs(lnc.constant);
      for (size_t var = 0; var < numeric_task.get_n_numeric_variables();
           ++var) {
        double coefficient = lnc.coefficients[var];
        if (fabs(coefficient) > 0)
          lhs.addTerms(&coefficient, &y[t_max - 1][var], 1);
      }
      model->addConstr(lhs >= numeric_task.get_epsilon(id_n_con), name);
    }
  }
}

void NumericConstraints::mutex_relaxtion_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) {
  size_t n_actions = numeric_task.get_n_actions();
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  for (size_t op_id1 = 0; op_id1 < n_actions - 1; ++op_id1) {
    for (size_t op_id2 = op_id1 + 1; op_id2 < n_actions; ++op_id2) {
      if (numeric_mutex[op_id1][op_id2]) {
        for (int t = t_min; t < t_max; ++t) {
          model->addConstr(x[t][op_id1] + x[t][op_id2] <= 1);
        }
      }
    }
  }
}

static shared_ptr<GurobiIPConstraintGenerator> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "Numeric constraints.",
      "For details, see" +
          utils::format_paper_reference(
              {"Chiara Piacentini", "Margarita P. Castro", "Andre A. Cire",
               "J. Chirstopher Beck"},
              "Compiling optimal numeric planning to mixed integer linear "
              "programming",
              "https://aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/view/17770",
              "Proceedings of the Twentyeighth International Conference on"
              " Planning and Scheduling (ICAPS 2018)",
              "383-387", "2018"));
  parser.add_option<int>(
      "num_repetition",
      "Maximum number of the same actions at the same time step", "1");
  Options opts = parser.parse();

  if (parser.dry_run()) return nullptr;
  return make_shared<NumericConstraints>(opts);
}

static PluginShared<GurobiIPConstraintGenerator> _plugin("numeric", _parse);
