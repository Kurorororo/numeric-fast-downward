#ifndef GUROBI_CONSTRAINT_GENERATOR_H
#define GUROBI_CONSTRAINT_GENERATOR_H

#include <memory>
#include <vector>

#include "gurobi_c++.h"

class AbstractTask;
class State;

namespace gurobi_ip_compilation {

class GurobiIPConstraintGenerator {
 public:
  virtual void initialize(const int horizon,
                          const std::shared_ptr<AbstractTask> task,
                          std::shared_ptr<GRBModel> model,
                          std::vector<GRBVar *> &x) = 0;
  virtual void update(const int horizon,
                      const std::shared_ptr<AbstractTask> task,
                      std::shared_ptr<GRBModel> model,
                      std::vector<GRBVar *> &x) = 0;
};
}  // namespace gurobi_ip_compilation

#endif
