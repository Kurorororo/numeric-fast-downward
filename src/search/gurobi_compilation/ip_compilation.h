#ifndef GUROBI_IP_COMPILATION_H
#define GUROBI_IP_COMPILATION_H

#include "../operator_counting/operator_counting_heuristic.h"
#include "../search_engine.h"
#include "gurobi_c++.h"
#include "ip_constraint_generator.h"

using namespace operator_counting;

namespace gurobi_ip_compilation {

/*
    this class create the model and solve one iteration of the ip compilation
    - constraint generator
 */
enum class ModelType { SC, SAS };

class GurobiIPCompilation {
  std::vector<std::shared_ptr<GurobiIPConstraintGenerator>>
      constraint_generators;

 public:
  GurobiIPCompilation(const options::Options &opts,
                      const std::shared_ptr<AbstractTask> &task);
  ~GurobiIPCompilation();
  void initialize(const int horizon);
  void update(const int horizon);
  void add_sequence_constraint();
  ap_float get_min_action_cost();
  ap_float compute_plan();
  SearchEngine::Plan extract_plan();

 protected:
  const std::shared_ptr<AbstractTask> task;
  GRBEnv *env;
  std::vector<GRBVar *> x;
  std::shared_ptr<GRBModel> model;
  ap_float min_action_cost;
  std::vector<double> op_costs;
};

}  // namespace gurobi_ip_compilation
#endif
