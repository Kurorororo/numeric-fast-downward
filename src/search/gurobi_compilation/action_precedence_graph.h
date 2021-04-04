#ifndef ACTION_PRECEDENCE_GRAPH_H_
#define ACTION_PRECEDENCE_GRAPH_H_

#include <vector>

namespace gurobi_ip_compilation {

class ActionPrecedenceGraph {
 private:
  int max_num_cuts;
  std::vector<std::vector<bool>> edges;
  std::vector<std::vector<double>> d;
  std::vector<std::vector<int>> next;
  std::vector<int> nodes;

  void floyd_warshall(const std::vector<double> &x_values);
  void construct_shortest_path(const std::vector<double> &x_values, int a,
                               int b, std::vector<int> &path);

 public:
  ActionPrecedenceGraph(size_t num_nodes)
      : max_num_cuts(1),
        edges(num_nodes, std::vector<bool>(num_nodes, false)),
        d(num_nodes, std::vector<double>(num_nodes)),
        next(num_nodes, std::vector<int>(num_nodes, -1)) {}
  void add_edge(int a, int b);
  bool is_connected(int a, int b) const { return edges[a][b]; }
  void find_cycle(const std::vector<double> &x_values,
                  std::vector<std::vector<int>> &cycles);
  std::vector<int> topological_sort(const std::vector<int> &input_nodes);
};

}  // namespace gurobi_ip_compilation

#endif