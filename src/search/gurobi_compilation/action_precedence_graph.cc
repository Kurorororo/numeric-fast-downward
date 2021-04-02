#include "action_precedence_graph.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>

using namespace gurobi_ip_compilation;

void ActionPrecedenceGraph::add_edge(int a, int b) { edges[a][b] = true; }

std::vector<std::vector<int>> ActionPrecedenceGraph::find_cycle(
    const std::vector<double> &x_values) {
  std::vector<int> nodes;
  std::vector<std::vector<int>> cycle_nodes;
  for (int i = 0, n = x_values.size(); i < n; ++i)
    if (x_values[i] > 0.0) nodes.push_back(i);
  floyd_warshall(nodes, x_values);

  for (auto a : nodes) {
    for (auto b : nodes) {
      if (a == b || next[a][b] == -1) continue;
      if (d[a][b] - x_values[b] - x_values[a] + 1 < 0) {
        cycle_nodes.push_back(construct_shortest_path(nodes, x_values, a, b));
        if (cycle_nodes.size() >= max_num_cuts) return std::move(cycle_nodes);
      }
    }
  }

  return std::move(cycle_nodes);
}

void ActionPrecedenceGraph::floyd_warshall(
    const std::vector<int> &nodes, const std::vector<double> &x_values) {
  for (auto i : nodes) {
    std::fill(d[i].begin(), d[i].end(), std::numeric_limits<double>::max());
    std::fill(next[i].begin(), next[i].end(), -1);
    for (auto j : nodes) {
      if (i != j && edges[i][j]) {
        next[i][j] = j;
        d[i][j] = 2 - x_values[i] - x_values[j];
      }
    }
  }

  for (auto k : nodes) {
    for (auto i : nodes) {
      if (k == i) continue;
      for (auto j : nodes) {
        if (i == j || k == j || next[i][k] == -1 || next[k][j] == -1) continue;
        if (d[i][j] > d[i][k] + d[k][j]) {
          d[i][j] = d[i][k] + d[k][j];
          next[i][j] = next[i][k];
        }
      }
    }
  }
}

std::vector<int> ActionPrecedenceGraph::construct_shortest_path(
    const std::vector<int> &nodes, const std::vector<double> &x_values, int a,
    int b) {
  int current_node = a;
  std::vector<int> path(1, a);
  while (current_node != b) {
    current_node = next[current_node][b];
    path.push_back(current_node);
  }
  return std::move(path);
}

std::vector<int> ActionPrecedenceGraph::topological_sort(
    const std::vector<int> &nodes) {
  std::vector<int> sorted;
  std::unordered_map<int, int> num_edges;
  std::queue<int> candidates;

  for (auto a : nodes) {
    num_edges[a] = 0;
    for (auto b : nodes) {
      if (a != b && edges[b][a]) {
        num_edges[a] += 1;
      }
    }
    if (num_edges[a] == 0) candidates.push(a);
  }

  while (!candidates.empty()) {
    int a = candidates.front();
    candidates.pop();
    for (auto b : nodes) {
      if (a != b && edges[a][b]) {
        num_edges[b] -= 1;
        if (num_edges[b] == 0) candidates.push(b);
      }
    }
    sorted.push_back(a);
  }

  return std::move(sorted);
}