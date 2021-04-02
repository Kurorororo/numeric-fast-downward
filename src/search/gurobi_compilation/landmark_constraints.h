#ifndef LANDMARK_CONSTRAINTS_H
#define LANDMARK_CONSTRAINTS_H

#include "../numeric_landmarks/landmark_factory_scala.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "ip_constraint_generator.h"

namespace numeric_helper {
class NumericTaskProxy;
}

namespace gurobi_ip_compilation {
class LandmarkConstraints : public GurobiIPConstraintGenerator {
 private:
  numeric_helper::NumericTaskProxy numeric_task;
  landmarks::LandmarkFactoryScala *factory;

  void landmark_constraints(const std::shared_ptr<AbstractTask> task,
                            std::shared_ptr<GRBModel> model,
                            std::vector<std::vector<GRBVar>> &x, int t_max,
                            bool first);
  set<int> fact_landmarks;
  set<int> action_landmarks;

 public:
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
