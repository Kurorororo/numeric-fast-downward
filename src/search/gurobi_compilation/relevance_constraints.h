#ifndef GUROBI_RELEVANCE_CONSTRAINTS_H
#define GUROBI_RELEVANCE_CONSTRAINTS_H

#include <memory>
#include <stack>
#include <vector>

#include "../numeric_operator_counting/numeric_helper.h"
#include "../task_proxy.h"
#include "gurobi_c++.h"
#include "ip_constraint_generator.h"

namespace gurobi_ip_compilation {

class RelevanceConstraints : public GurobiIPConstraintGenerator {
 private:
  void push_propositional(const TaskProxy &task, FactProxy f,
                          std::stack<size_t> &open);
  void push_numeric(numeric_helper::NumericTaskProxy &numeric_task, int c,
                    std::stack<size_t> &open);
  void analyze_relevance(const std::shared_ptr<AbstractTask> task);

  int current_horizon;
  std::vector<bool> action_relevant;

 public:
  RelevanceConstraints() : current_horizon(0) {}
  virtual void initialize(
      const int horizon, const std::shared_ptr<AbstractTask> task,
      std::shared_ptr<GRBModel> model, std::vector<std::vector<GRBVar>> &x,
      std::vector<std::vector<bool>> &action_mutex) override;
  virtual void update(const int horizon,
                      const std::shared_ptr<AbstractTask> task,
                      std::shared_ptr<GRBModel> model,
                      std::vector<std::vector<GRBVar>> &x) override;
};
}  // namespace gurobi_ip_compilation

#endif
