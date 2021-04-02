#ifndef GUROBI_NUMERIC_CONSTRAINTS_WITH_CUTS_H
#define GUROBI_NUMERIC_CONSTRAINTS_WITH_CUTS_H

#include <unordered_map>
#include <utility>

#include "../option_parser.h"
#include "numeric_constraints.h"

namespace gurobi_ip_compilation {
class NumericConstraintsWithCuts : public NumericConstraints {
 protected:
  virtual void initialize_action_precedence();
  virtual void initialize_numeric_mutex() override;
  virtual void precondition_constraint(const std::shared_ptr<AbstractTask> task,
                                       std::shared_ptr<GRBModel> model,
                                       std::vector<std::vector<GRBVar>> &x,
                                       int t_min, int t_max) override;
  std::vector<std::vector<bool>> action_precedence;
  std::unordered_map<int, std::vector<int>> net_positive_actions;
  std::unordered_map<std::pair<int, int>, double> net_values;

 public:
  NumericConstraintsWithCuts(const Options &opts);

  virtual void initialize(const int horizon,
                          const std::shared_ptr<AbstractTask> task,
                          std::shared_ptr<GRBModel> model,
                          std::vector<std::vector<GRBVar>> &x) override;
  virtual void add_action_precedence(
      const std::shared_ptr<AbstractTask> task,
      std::shared_ptr<ActionPrecedenceGraph> graph) override;
};
}  // namespace gurobi_ip_compilation
#endif
