#include "action_cycle_elimination_callback.h"

#include <cmath>

using namespace gurobi_ip_compilation;

void ActionCycleEliminationCallback::callback() {
  try {
    if (where == GRB_CB_MIPSOL) {
      size_t num_ops = x[0].size();
      std::vector<double> x_values(num_ops);
      int num_cuts = 0;
      for (size_t t = 0; t < x.size(); ++t) {
        for (size_t op_id = 0; op_id < num_ops; ++op_id)
          x_values[op_id] = std::min(getSolution(x[t][op_id]), 1.0);
        auto cycle_nodes = graph->find_cycle(x_values);
        for (auto cycle : cycle_nodes) {
          if (num_cuts >= max_num_cuts) break;
          for (size_t t2 = 0; t2 < x.size(); ++t2) {
            GRBLinExpr lhs;
            double coefficient = 1;
            for (auto node : cycle) lhs.addTerms(&coefficient, &x[t2][node], 1);
            addLazy(lhs <= cycle.size() - 1);
          }
          if (++num_cuts >= max_num_cuts) break;
        }
        if (num_cuts >= max_num_cuts) break;
      }
    } else if (add_user_cut && where == GRB_CB_MIPNODE) {
      if (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL) {
        size_t num_ops = x[0].size();
        std::vector<double> x_values(num_ops);
        int num_cuts = 0;
        for (size_t t = 0; t < x.size(); ++t) {
          for (size_t op_id = 0; op_id < num_ops; ++op_id) {
            x_values[op_id] = std::min(getNodeRel(x[t][op_id]), 1.0);
          }
          auto cycle_nodes = graph->find_cycle(x_values);
          for (auto cycle : cycle_nodes) {
            for (size_t t2 = 0; t2 < x.size(); ++t2) {
              GRBLinExpr lhs;
              double coefficient = 1;
              for (auto node : cycle)
                lhs.addTerms(&coefficient, &x[t2][node], 1);
              addCut(lhs <= cycle.size() - 1);
            }
            if (++num_cuts >= max_num_cuts) break;
          }
          if (num_cuts >= max_num_cuts) break;
        }
      }
    }
  } catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Error during callback" << std::endl;
  }
}