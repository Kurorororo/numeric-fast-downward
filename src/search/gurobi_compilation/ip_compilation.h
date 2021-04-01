#ifndef GUROBI_IP_COMPILATION_H
#define GUROBI_IP_COMPILATION_H

#include "../abstract_task.h"
#include "../search_engine.h"
#include "gurobi_c++.h"
#include "ip_constraint_generator.h"

namespace gurobi_ip_compilation {

/*
    this class create the model and solve one iteration of the ip compilation
    - constraint generator
 */
enum class ModelType { SC, SAS };

class GurobiIPCompilation {
  std::vector<std::shared_ptr<GurobiIPConstraintGenerator>>
      constraint_generators;

 private:
  void add_variables(const int t_min, const int t_max);

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
  std::vector<std::vector<GRBVar>> x;
  std::shared_ptr<GRBModel> model;
  ap_float min_action_cost;
};

}  // namespace gurobi_ip_compilation
#endif
