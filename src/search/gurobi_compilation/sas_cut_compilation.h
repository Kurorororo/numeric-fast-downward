#ifndef GUROBI_SAS_CUT_STATE_CHANGE_MODEL_H
#define GUROBI_SAS_CUT_STATE_CHANGE_MODEL_H

#include <set>
#include <vector>

#include "../numeric_operator_counting/numeric_helper.h"
#include "gurobi_c++.h"
#include "sas_compilation.h"

namespace gurobi_ip_compilation {
class GurobiSASStateChangeModelWithCuts : public GurobiSASStateChangeModel {
 private:
  std::vector<std::vector<bool>> action_mutex;

  void initialize_mutex_actions(const std::shared_ptr<AbstractTask> task);

 protected:
  // constraints
  virtual void mutex_relaxtion_constraint(
      const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
      std::vector<std::vector<GRBVar>> &x, int t_min, int t_max) override;
  virtual void precondition_constraint(const std::shared_ptr<AbstractTask> task,
                                       std::shared_ptr<GRBModel> model,
                                       std::vector<std::vector<GRBVar>> &x,
                                       int t_min, int t_max) override;

 public:
  GurobiSASStateChangeModelWithCuts();

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
