#ifndef ACTION_CYCLE_ELIMINATION_CALLBACK_H_
#define ACTION_CYCLE_ELIMINATION_CALLBACK_H

#include <memory>

#include "action_precedence_graph.h"
#include "gurobi_c++.h"

namespace gurobi_ip_compilation {

class ActionCycleEliminationCallback : public GRBCallback {
 private:
  int max_num_cuts;
  bool add_user_cut;
  const std::vector<std::vector<GRBVar>> &x;
  std::shared_ptr<ActionPrecedenceGraph> graph;

 protected:
  void callback();

 public:
  ActionCycleEliminationCallback(int max_num_cuts_, bool add_user_cut_,
                                 const std::vector<std::vector<GRBVar>> &x_,
                                 std::shared_ptr<ActionPrecedenceGraph> graph_)
      : max_num_cuts(max_num_cuts_),
        add_user_cut(add_user_cut_),
        x(x_),
        graph(graph_) {}
};

}  // namespace gurobi_ip_compilation

#endif
