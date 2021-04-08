#ifndef HEURISTICS_NUMERIC_LM_CUT_LANDMARKS_H
#define HEURISTICS_NUMERIC_LM_CUT_LANDMARKS_H

#include "../globals.h"
#include "../priority_queue.h"
#include "../task_tools.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../utils/rng.h"

#include <cassert>
#include <functional>
#include <memory>
#include <vector>

namespace numeric_lm_cut_heuristic {
    // TODO: Fix duplication with the other relaxation heuristics.
    struct RelaxedProposition;
    
    enum PropositionStatus {
        UNREACHED = 0,
        REACHED = 1,
        GOAL_ZONE = 2,
        BEFORE_GOAL_ZONE = 3
    };
    
    struct RelaxedOperator {
        int original_op_id;
        std::vector<RelaxedProposition *> preconditions;
        std::vector<RelaxedProposition *> effects;
        std::vector<ap_float> numeric_effects;
        
        ap_float base_cost; // 0 for axioms, 1 for regular operators
        
        ap_float cost;
        int unsatisfied_preconditions;
        ap_float h_max_supporter_cost; // h_max_cost of h_max_supporter
        RelaxedProposition *h_max_supporter;
        
        string name;
        
        RelaxedOperator(std::vector<RelaxedProposition *> &&pre,
                        std::vector<RelaxedProposition *> &&eff,
                        std::vector<ap_float> &&num_eff,
                        int op_id, ap_float base, string &n)
        : original_op_id(op_id), preconditions(pre), effects(eff), numeric_effects(num_eff), base_cost(base), name(n) {
        }
        
        inline void update_h_max_supporter();
        inline void select_random_supporter();
    };
    
    struct RelaxedProposition {
        std::vector<RelaxedOperator *> precondition_of;
        std::vector<RelaxedOperator *> effect_of;
        
        PropositionStatus status;
        bool explored;
        bool is_numeric_condition;
        int id_numeric_condition;
        ap_float h_max_cost;
        string name;
    };
    
    class LandmarkCutLandmarks {
        numeric_helper::NumericTaskProxy numeric_task;
        std::vector<RelaxedOperator> relaxed_operators;
        std::vector<std::vector<RelaxedProposition>> propositions;
        RelaxedProposition artificial_precondition;
        RelaxedProposition artificial_goal;
        int num_propositions;
        int n_var;
        bool ceiling_less_than_one;
        bool ignore_numeric_conditions;
        bool use_random_pcf;
        std::vector<ap_float> numeric_initial_state;
        
        AdaptiveQueue<RelaxedProposition *> priority_queue;
        
        void initialize();
        void build_relaxed_operator(const OperatorProxy &op);
        void add_relaxed_operator(std::vector<RelaxedProposition *> &&precondition,
                                  std::vector<RelaxedProposition *> &&effects,
                                  std::vector<ap_float> &&numeric_effects,
                                  int op_id, ap_float base_cost, string &n);
        RelaxedProposition *get_proposition(const FactProxy &fact);
        RelaxedProposition *get_proposition(const int &n_condition);
        void setup_exploration_queue();
        void setup_exploration_queue_state(const State &state);
        void first_exploration(const State &state);
        void first_exploration_incremental(std::vector<RelaxedOperator *> &cut);
        void second_exploration(const State &state,
                                std::vector<RelaxedProposition *> &queue,
                                std::vector<RelaxedOperator *> &cut,
                                std::unordered_map<int, ap_float> &operator_to_m);
        
        bool enqueue_if_necessary(RelaxedProposition *prop, int cost) {
            assert(cost >= 0);
            if (prop->status == UNREACHED || prop->h_max_cost > cost) {
                prop->status = REACHED;
                prop->h_max_cost = cost;
                priority_queue.push(cost, prop);
                return true;
            }
            return false;
        }
        
        void update_queue(RelaxedProposition *prec, RelaxedProposition *eff, RelaxedOperator *op);
        ap_float calculate_numeric_times(RelaxedProposition *effect, RelaxedOperator *relaxed_op);
        
        void mark_goal_plateau(RelaxedProposition *subgoal);
        void validate_h_max() const;
    public:
        using Landmark = std::vector<pair<ap_float,int>>;
        using CostCallback = std::function<void (ap_float)>;
        using LandmarkCallback = std::function<void (const Landmark &, int)>;
        
        LandmarkCutLandmarks(const TaskProxy &task_proxy, bool ceiling_less_than_one = false, bool ignore_numeric = false,
                             bool use_random_pcf = false);
        virtual ~LandmarkCutLandmarks();
        
        /*
         Compute LM-cut landmarks for the given state.
         
         If cost_callback is not nullptr, it is called once with the cost of each
         discovered landmark.
         
         If landmark_callback is not nullptr, it is called with each discovered
         landmark (as a vector of operator indices) and its cost. This requires
         making a copy of the landmark, so cost_callback should be used if only the
         cost of the landmark is needed.
         
         Returns true iff state is detected as a dead end.
         */
        bool compute_landmarks(State state, CostCallback cost_callback,
                               LandmarkCallback landmark_callback);
    };
    
    inline void RelaxedOperator::update_h_max_supporter() {
        assert(!unsatisfied_preconditions);
        for (size_t i = 0; i < preconditions.size(); ++i)
            if (preconditions[i]->h_max_cost > h_max_supporter->h_max_cost)
                h_max_supporter = preconditions[i];
        h_max_supporter_cost = h_max_supporter->h_max_cost;
    }

    inline void RelaxedOperator::select_random_supporter() {
        assert(!unsatisfied_preconditions);
        size_t n_zero = 0;
        for (size_t i = 0; i < preconditions.size(); ++i)
            if (preconditions[i]->h_max_cost <= 0) ++n_zero;
        if (n_zero == preconditions.size()) {
            int index = (*g_rng())(preconditions.size());
            h_max_supporter = preconditions[index];
        } else {
            int index = (*g_rng())(preconditions.size() - n_zero);
            int counter = 0;
            for (size_t i = 0; i < preconditions.size(); ++i) {
                if (preconditions[i]->h_max_cost > 0) {
                    if (counter == index) {
                        h_max_supporter = preconditions[i];
                        break;
                    }
                    ++counter;
                }
            }

        }
        h_max_supporter_cost = h_max_supporter->h_max_cost;
    }
}

#endif

