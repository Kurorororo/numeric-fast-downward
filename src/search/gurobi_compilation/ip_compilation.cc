#include "ip_compilation.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace gurobi_ip_compilation;

GurobiIPCompilation::GurobiIPCompilation(
    const options::Options &opts, const std::shared_ptr<AbstractTask> &task)
    : constraint_generators(
          opts.get_list<std::shared_ptr<GurobiIPConstraintGenerator>>(
              "gurobi_ipmodel")),
      task(task),
      env(new GRBEnv()),
      min_action_cost(std::numeric_limits<ap_float>::max()) {
  env->set(GRB_IntParam_Threads, opts.get<int>("threads"));
  if (opts.get<bool>("lazy_constraints"))
    env->set(GRB_IntParam_LazyConstraints, 1);
  env->set(GRB_IntParam_OutputFlag, 0);
  model = std::make_shared<GRBModel>(*env);
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  op_costs.resize(ops.size());
  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    const OperatorProxy &op = ops[op_id];
    ap_float cost = op.get_cost();
    op_costs[op_id] = cost;
    if (cost < min_action_cost) min_action_cost = cost;
  }
}

GurobiIPCompilation::~GurobiIPCompilation() {
  for (auto v : x) delete[] v;
  delete env;
}

void GurobiIPCompilation::initialize(const int horizon) {
  std::vector<char> types(op_costs.size(), GRB_BINARY);
  int t_min = x.size();
  x.resize(horizon);
  for (int t = t_min; t < horizon; ++t) {
    x[t] = model->addVars(NULL, NULL, op_costs.data(), types.data(), NULL,
                          op_costs.size());
  }
  for (auto generator : constraint_generators)
    generator->initialize(horizon, task, model, x);
}

void GurobiIPCompilation::update(const int horizon) {
  std::vector<char> types(op_costs.size(), GRB_BINARY);
  int t_min = x.size();
  x.resize(horizon);
  for (int t = t_min; t < horizon; ++t) {
    x[t] = model->addVars(NULL, NULL, op_costs.data(), types.data(), NULL,
                          op_costs.size());
  }
  for (auto generator : constraint_generators)
    generator->update(horizon, task, model, x);
}

void GurobiIPCompilation::add_sequence_constraint() {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  std::vector<double> coeff(ops.size(), 1);
  int horizon = x.size();
  for (int t = 0; t < horizon; ++t) {
    GRBLinExpr sum_t;
    sum_t.addTerms(coeff.data(), x[t], ops.size());
    model->addConstr(sum_t <= 1);

    if (t > 0) {
      GRBLinExpr sum_t_1;
      sum_t_1.addTerms(coeff.data(), x[t - 1], ops.size());
      model->addConstr(sum_t_1 <= sum_t);
    }
  }
}

ap_float GurobiIPCompilation::compute_plan() {
  model->optimize();
  int status = model->get(GRB_IntAttr_Status);

  if (status == GRB_OPTIMAL) return model->get(GRB_DoubleAttr_ObjVal);

  return -1;
}

SearchEngine::Plan GurobiIPCompilation::extract_plan() {
  TaskProxy task_proxy(*task);
  OperatorsProxy ops = task_proxy.get_operators();
  SearchEngine::Plan plan;

  for (auto v : x) {
    double *values = model->get(GRB_DoubleAttr_X, v, ops.size());
    for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
      int n = values[op_id];
      for (int i = 0; i < n; ++i)
        plan.push_back(ops[op_id].get_global_operator());
    }
    delete[] values;
  }

  return plan;
}

ap_float GurobiIPCompilation::get_min_action_cost() { return min_action_cost; }
