#include "numeric_constraints_with_cuts.h"

#include "../globals.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../utils/markup.h"

using namespace std;
using namespace gurobi_ip_compilation;
using namespace numeric_helper;

NumericConstraintsWithCuts::NumericConstraintsWithCuts(const Options &opts)
    : NumericConstraints(opts) {}

void NumericConstraintsWithCuts::initialize(
    const int horizon, const std::shared_ptr<AbstractTask> task,
    std::shared_ptr<GRBModel> model, std::vector<std::vector<GRBVar>> &x) {
  cout << "initializing numeric with cuts" << endl;
  TaskProxy task_proxy(*task);
  numeric_task = NumericTaskProxy(task_proxy);
  initialize_numeric_mutex();
  initialize_action_precedence();

  if (num_repetition > 1) NumericConstraints::initialize_repetable_actions(x);

  NumericConstraints::update(horizon, task, model, x);
  NumericConstraints::initial_state_constraint(task, model);
}

void NumericConstraintsWithCuts::initialize_action_precedence() {
  size_t n_actions = numeric_task.get_n_actions();
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  action_precedence.resize(n_actions, std::vector<bool>(n_actions, false));
  for (size_t op_id1 = 0; op_id1 < n_actions; ++op_id1) {
    for (size_t op_id2 = 0; op_id2 < n_actions; ++op_id2) {
      if (op_id1 == op_id2) continue;
      for (int pre : numeric_task.get_action_num_list(op_id1)) {
        for (int i : numeric_task.get_numeric_conditions_id(pre)) {
          auto key = std::make_pair(i, op_id2);
          if (net_values.find(key) != net_values.end()) continue;
          if (net_positive_actions.find(i) == net_positive_actions.end())
            net_positive_actions[i] = std::vector<int>();
          LinearNumericCondition &lnc = numeric_task.get_condition(i);
          ap_float net = 0;
          for (int nv_id = 0; nv_id < n_numeric_variables; ++nv_id) {
            net += lnc.coefficients[nv_id] *
                   numeric_task.get_action_eff_list(op_id2)[nv_id];
          }
          if (net > 0.0) {
            net_positive_actions[i].push_back(op_id2);
            action_precedence[op_id2][op_id1] = true;
          }
          if (net < 0.0) {
            action_precedence[op_id1][op_id2] = true;
          }
          net_values[key] = net;
        }
      }
    }
  }
}

void NumericConstraintsWithCuts::initialize_numeric_mutex() {
  size_t n_actions = numeric_task.get_n_actions();
  numeric_mutex.resize(n_actions, std::vector<bool>(n_actions, false));
}

void NumericConstraintsWithCuts::add_action_precedence(
    const std::shared_ptr<AbstractTask> task,
    std::shared_ptr<ActionPrecedenceGraph> graph) {
  size_t n_actions = numeric_task.get_n_actions();
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  for (size_t op_id1 = 0; op_id1 < n_actions; ++op_id1) {
    for (size_t op_id2 = 0; op_id2 < n_actions; ++op_id2) {
      if (op_id1 != op_id2 && action_precedence[op_id1][op_id2]) {
        graph->add_edge(op_id1, op_id2);
      }
    }
  }
}

void NumericConstraintsWithCuts::precondition_constraint(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) {
  int n_numeric_variables = numeric_task.get_n_numeric_variables();
  for (size_t op_id = 0; op_id < numeric_task.get_n_actions(); ++op_id) {
    for (int pre : numeric_task.get_action_num_list(op_id)) {
      for (int i : numeric_task.get_numeric_conditions_id(pre)) {
        auto positive_actions = net_positive_actions.find(i);

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

          if (positive_actions != net_positive_actions.end()) {
            for (auto op_id2 : positive_actions->second) {
              if (op_id2 == op_id) continue;
              lhs.addTerms(&net_values[std::make_pair(i, op_id2)],
                           &x[t][op_id2], 1);
            }
          }

          model->addConstr(lhs >= rhs + numeric_task.get_epsilon(i));
        }
      }
    }
  }
}

static shared_ptr<GurobiIPConstraintGenerator> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "Numeric constraints with cuts.",
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
  return make_shared<NumericConstraintsWithCuts>(opts);
}

static PluginShared<GurobiIPConstraintGenerator> _plugin("numeric_cut", _parse);
