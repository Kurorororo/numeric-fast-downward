#ifndef GENERALIZED_SUBGOALING_HEURISTIC_H
#define GENERALIZED_SUBGOALING_HEURISTIC_H

#include "heuristic.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../priority_queue.h"
#include <unordered_map>
#include <vector>
#include <set>

using namespace std;

namespace generalized_subgoaling_heuristic {

    //TODO do we really need the interval relaxation?

class GeneralizedSubgoalingHeuristic : public Heuristic {
protected:
	virtual void initialize();
	virtual ap_float compute_heuristic(const GlobalState &global_state);
	virtual ap_float update_cost(ap_float old_cost, ap_float new_cost) {return std::max(old_cost, new_cost);}
    
    void setup(const GlobalState &global_state);

    unordered_map<vector<int>, int> preconditions_to_id;
    vector<int, int> action_to_preconditions_id;
    vector<set<int>> condition_to_action; // index condition, value set of action with that preconditions
    vector<double> dist;
    vector<bool> open;
    vector<bool> closed;
    vector<bool> active_actions;

    vector<set<int>> possible_achievers; // index: action, value, set of numeric conditions that can be achieved by the action
    vector<set<int>> possible_preconditions_achievers; // index: action, value, set of preconditions that can be achieved by the action
    numeric_helper::NumericTaskProxy numeric_task;
    vector<vector<double>> net_effects; // index: action, index n_condition, value: net effect;
    double max_float;
    void generate_possible_achievers();
    void generate_preconditions();
    double check_goal();
public:
	GeneralizedSubgoalingHeuristic(const options::Options &options);
	~GeneralizedSubgoalingHeuristic();
    
};
}

#endif
