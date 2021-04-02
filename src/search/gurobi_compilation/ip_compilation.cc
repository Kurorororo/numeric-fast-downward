#include "ip_compilation.h"

#include <cmath>

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"

using namespace std;
using namespace gurobi_ip_compilation;

GurobiIPCompilation::GurobiIPCompilation(
    const options::Options &opts, const std::shared_ptr<AbstractTask> &task)
    : add_lazy_constraints(opts.get<bool>("lazy_constraints")),
      add_user_cuts(opts.get<bool>("user_cuts")),
      max_num_cuts(opts.get<int>("max_num_cuts")),
      min_action_cost(std::numeric_limits<ap_float>::max()),
      constraint_generators(
          opts.get_list<std::shared_ptr<GurobiIPConstraintGenerator>>(
              "gurobi_ipmodel")),
      task(task),
      env(new GRBEnv()),
      graph(nullptr),
      callback(nullptr) {
  if (add_user_cuts) add_lazy_constraints = true;
  env->set(GRB_IntParam_Threads, opts.get<int>("threads"));
  if (add_lazy_constraints) env->set(GRB_IntParam_LazyConstraints, 1);
  if (add_user_cuts) env->set(GRB_IntParam_PreCrush, 1);
  env->set(GRB_IntParam_OutputFlag, 0);
  model = std::make_shared<GRBModel>(*env);
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    const OperatorProxy &op = ops[op_id];
    ap_float cost = op.get_cost();
    if (cost < min_action_cost) min_action_cost = cost;
  }

  if (add_lazy_constraints || add_user_cuts)
    graph = std::make_shared<ActionPrecedenceGraph>(ops.size());
}

GurobiIPCompilation::~GurobiIPCompilation() { delete env; }

void GurobiIPCompilation::initialize(const int horizon) {
  add_variables(0, horizon);
  for (auto generator : constraint_generators) {
    generator->initialize(horizon, task, model, x);
    if (add_lazy_constraints || add_user_cuts)
      generator->add_action_precedence(task, graph);
  }

  if (add_lazy_constraints || add_user_cuts) {
    callback = std::make_shared<ActionCycleEliminationCallback>(
        max_num_cuts, add_user_cuts, x, graph);
    model->setCallback(callback.get());
  }
}

void GurobiIPCompilation::update(const int horizon) {
  int t_min = x.size();
  add_variables(t_min, horizon);
  for (auto generator : constraint_generators)
    generator->update(horizon, task, model, x);
}

void GurobiIPCompilation::add_variables(const int t_min, const int t_max) {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  x.resize(t_max, std::vector<GRBVar>(ops.size()));
  for (int t = t_min; t < t_max; ++t) {
    for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
      const OperatorProxy &op = ops[op_id];
      std::string name = "x_" + std::to_string(op_id) + "_" + std::to_string(t);
      x[t][op_id] = model->addVar(0, 1, op.get_cost(), GRB_BINARY, name);
    }
  }
}

void GurobiIPCompilation::add_sequence_constraint() {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  std::vector<double> coeff(ops.size(), 1);
  int horizon = x.size();
  for (int t = 0; t < horizon; ++t) {
    GRBLinExpr sum_t;
    sum_t.addTerms(coeff.data(), x[t].data(), ops.size());
    model->addConstr(sum_t <= 1);

    if (t > 0) {
      GRBLinExpr sum_t_1;
      sum_t_1.addTerms(coeff.data(), x[t - 1].data(), ops.size());
      model->addConstr(sum_t_1 <= sum_t);
    }
  }
}

ap_float GurobiIPCompilation::compute_plan() {
  try {
    model->optimize();
  } catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Error during optimize" << std::endl;
  }

  int status = model->get(GRB_IntAttr_Status);

  if (status == GRB_OPTIMAL) return model->get(GRB_DoubleAttr_ObjVal);

  return -1;
}

SearchEngine::Plan GurobiIPCompilation::extract_plan() {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  SearchEngine::Plan plan;
  int t_max = x.size();

  for (int t = 0; t < t_max; ++t) {
    if (add_lazy_constraints || add_user_cuts) {
      std::vector<int> nodes;
      std::unordered_map<int, int> ns;
      for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
        int n = std::round(x[t][op_id].get(GRB_DoubleAttr_X));
        if (n > 0) {
          nodes.push_back(op_id);
          ns[op_id] = n;
        }
      }
      if (nodes.size() > 0) {
        auto subplan = graph->topological_sort(nodes);

        if (subplan.size() != nodes.size()) {
          std::cout << "plan contains a cycle" << std::endl;
          return SearchEngine::Plan();
        }

        for (auto op_id : subplan) {
          for (int i = 0; i < ns[op_id]; ++i)
            plan.push_back(ops[op_id].get_global_operator());
        }
      }
    } else {
      for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
        int n = std::round(x[t][op_id].get(GRB_DoubleAttr_X));

        for (int i = 0; i < n; ++i)
          plan.push_back(ops[op_id].get_global_operator());
      }
    }
  }

  return plan;
}

ap_float GurobiIPCompilation::get_min_action_cost() { return min_action_cost; }
