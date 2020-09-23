#include "landmark_factory.h"

#include "util.h"

#include "../utils/timer.h"

#include <fstream>
#include <limits>

using namespace std;

namespace landmarks {
LandmarkFactory::LandmarkFactory(const Options &opts)
    : lm_graph(new LandmarkGraph(opts)) {
}

LandmarkGraph *LandmarkFactory::compute_lm_graph() {
    utils::Timer lm_generation_timer;
    generate_landmarks();

    // the following replaces the old "build_lm_graph"
    generate();
    cout << "Landmarks generation time: " << lm_generation_timer << endl;
    if (lm_graph->number_of_landmarks() == 0)
        cout << "Warning! No landmarks found. Task unsolvable?" << endl;
    else {
        cout << "Discovered " << lm_graph->number_of_landmarks()
             << " landmarks, of which " << lm_graph->number_of_disj_landmarks()
             << " are disjunctive and "
             << lm_graph->number_of_conj_landmarks() << " are conjunctive \n"
             << lm_graph->number_of_edges() << " edges\n";
    }
    //lm_graph->dump();
    return lm_graph;
}

void LandmarkFactory::generate() {
    if (lm_graph->use_only_causal_landmarks())
        discard_noncausal_landmarks();
    if (!lm_graph->use_disjunctive_landmarks())
        discard_disjunctive_landmarks();
    if (!lm_graph->use_conjunctive_landmarks())
        discard_conjunctive_landmarks();
    lm_graph->set_landmark_ids();

    if (!lm_graph->use_orders())
        discard_all_orderings();
    else if (lm_graph->is_using_reasonable_orderings()) {
        cout << "approx. reasonable orders" << endl;
        approximate_reasonable_orders(false);
        cout << "approx. obedient reasonable orders" << endl;
        approximate_reasonable_orders(true);
    }
    mk_acyclic_graph();
    lm_graph->set_landmark_cost(calculate_lms_cost());
    calc_achievers();
}

bool LandmarkFactory::achieves_non_conditional(const GlobalOperator &o,
                                               const LandmarkNode *lmp) const {
    /* Test whether the landmark is achieved by the operator unconditionally.
    A disjunctive landmarks is achieved if one of its disjuncts is achieved. */
    assert(lmp != NULL);
    const vector<GlobalEffect> &effects = o.get_effects();
    for (size_t i = 0; i < effects.size(); ++i) {
        for (size_t j = 0; j < lmp->vars.size(); ++j) {
            if (effects[i].var == lmp->vars[j] && (int) effects[i].val == lmp->vals[j])
                if (effects[i].conditions.empty())
                    return true;
        }
    }
    return false;
}

bool LandmarkFactory::is_landmark_precondition(const GlobalOperator &o,
                                               const LandmarkNode *lmp) const {
    /* Test whether the landmark is used by the operator as a precondition.
    A disjunctive landmarks is used if one of its disjuncts is used. */
    assert(lmp != NULL);
    const vector<GlobalCondition> &preconditions = o.get_preconditions();
    for (size_t i = 0; i < preconditions.size(); ++i) {
        for (size_t j = 0; j < lmp->vars.size(); ++j) {
            if (preconditions[i].var == lmp->vars[j] &&
                (int) preconditions[i].val == lmp->vals[j])
                return true;
        }
    }
    return false;
}

bool LandmarkFactory::relaxed_task_solvable(vector<vector<int> > &lvl_var,
                                            vector<unordered_map<pair<int, int>, int> > &lvl_op,
                                            bool level_out, const LandmarkNode *exclude, bool compute_lvl_op) const {
    /* Test whether the relaxed planning task is solvable without achieving the propositions in
     "exclude" (do not apply operators that would add a proposition from "exclude").
     As a side effect, collect in lvl_var and lvl_op the earliest possible point in time
     when a proposition / operator can be achieved / become applicable in the relaxed task.
     */

    // Initialize lvl_op and lvl_var to numeric_limits<int>::max()
    if (compute_lvl_op) {
        lvl_op.resize(g_operators.size() + g_axioms_as_operator.size());
        for (size_t i = 0; i < g_operators.size() + g_axioms_as_operator.size(); ++i) {
            const GlobalOperator &op = lm_graph->get_operator_for_lookup_index(i);
            lvl_op[i] = unordered_map<pair<int, int>, int> ();
            const vector<GlobalEffect> &effects = op.get_effects();
            for (size_t j = 0; j < effects.size(); ++j)
                lvl_op[i].insert(make_pair(make_pair(effects[j].var,
                                                     effects[j].val),
                                           numeric_limits<int>::max()));
        }
    }
    lvl_var.resize(g_variable_name.size());
    for (size_t var = 0; var < g_variable_name.size(); ++var) {
        lvl_var[var].resize(g_variable_domain[var],
                            numeric_limits<int>::max());
    }
    // Extract propositions from "exclude"
    unordered_set<const GlobalOperator *> exclude_ops;
    vector<pair<int, int> > exclude_props;
    if (exclude != NULL) {
        for (size_t op = 0; op < g_operators.size(); ++op) {
            if (achieves_non_conditional(g_operators[op], exclude))
                exclude_ops.insert(&g_operators[op]);
        }
        for (size_t i = 0; i < exclude->vars.size(); ++i)
            exclude_props.push_back(make_pair(exclude->vars[i],
                                              exclude->vals[i]));
    }
    // Do relaxed exploration
    lm_graph->get_exploration()->compute_reachability_with_excludes(lvl_var, lvl_op, level_out,
                                                                    exclude_props, exclude_ops, compute_lvl_op);

    // Test whether all goal propositions have a level of less than numeric_limits<int>::max()
    for (size_t i = 0; i < g_goal.size(); ++i)
        if (lvl_var[g_goal[i].first][g_goal[i].second] ==
            numeric_limits<int>::max())
            return false;

    return true;
}


bool LandmarkFactory::is_causal_landmark(const LandmarkNode &landmark) const {
    /* Test whether the relaxed planning task is unsolvable without using any operator
       that has "landmark" has a precondition.
       Similar to "relaxed_task_solvable" above.
     */

    if (landmark.in_goal)
        return true;
    vector<vector<int> > lvl_var;
    vector<unordered_map<pair<int, int>, int> > lvl_op;
    // Initialize lvl_var to numeric_limits<int>::max()
    lvl_var.resize(g_variable_name.size());
    for (size_t var = 0; var < g_variable_name.size(); ++var) {
        lvl_var[var].resize(g_variable_domain[var],
                            numeric_limits<int>::max());
    }
    unordered_set<const GlobalOperator *> exclude_ops;
    vector<pair<int, int> > exclude_props;
    for (size_t op = 0; op < g_operators.size(); ++op) {
        if (is_landmark_precondition(g_operators[op], &landmark)) {
            exclude_ops.insert(&g_operators[op]);
        }
    }
    // Do relaxed exploration
    lm_graph->get_exploration()->compute_reachability_with_excludes(lvl_var, lvl_op, true,
                                                                    exclude_props, exclude_ops, false);

    // Test whether all goal propositions have a level of less than numeric_limits<int>::max()
    for (size_t i = 0; i < g_goal.size(); ++i)
        if (lvl_var[g_goal[i].first][g_goal[i].second] ==
            numeric_limits<int>::max())
            return true;

    return false;
}

bool LandmarkFactory::effect_always_happens(const vector<GlobalEffect> &effects, set<
                                                pair<int, int> > &eff) const {
    /* Test whether the condition of a conditional effect is trivial, i.e. always true.
     We test for the simple case that the same effect proposition is triggered by
     a set of conditions of which one will always be true. This is e.g. the case in
     Schedule, where the effect
     (forall (?oldpaint - colour)
     (when (painted ?x ?oldpaint)
     (not (painted ?x ?oldpaint))))
     is translated by the translator to: if oldpaint == blue, then not painted ?x, and if
     oldpaint == red, then not painted ?x etc.
     If conditional effects are found that are always true, they are returned in "eff".
     */
    // Go through all effects of operator and collect:
    // - all variables that are set to some value in a conditional effect (effect_vars)
    // - variables that can be set to more than one value in a cond. effect (nogood_effect_vars)
    // - a mapping from cond. effect propositions to all the conditions that they appear with
    set<int> effect_vars;
    set<int> nogood_effect_vars;
    map<int, pair<int, vector<pair<int, int> > > > effect_conditions;
    for (size_t i = 0; i < effects.size(); ++i) {
        if (effects[i].conditions.empty() ||
            nogood_effect_vars.find(effects[i].var) != nogood_effect_vars.end()) {
            // Var has no condition or can take on different values, skipping
            continue;
        }
        if (effect_vars.find(effects[i].var) != effect_vars.end()) {
            // We have seen this effect var before
            assert(effect_conditions.find(effects[i].var) != effect_conditions.end());
            int old_eff = effect_conditions.find(effects[i].var)->second.first;
            if (old_eff != (int) effects[i].val) {
                // Was different effect
                nogood_effect_vars.insert(effects[i].var);
                continue;
            }
        } else {
            // We have not seen this effect var before
            effect_vars.insert(effects[i].var);
        }
        if (effect_conditions.find(effects[i].var) != effect_conditions.end()
            && effect_conditions.find(effects[i].var)->second.first
            == (int) effects[i].val) {
            // We have seen this effect before, adding conditions
            for (size_t j = 0; j < effects[i].conditions.size(); ++j) {
                vector<pair<int, int> > &vec = effect_conditions.find(effects[i].var)->second.second;
                vec.push_back(make_pair(effects[i].conditions[j].var, effects[i].conditions[j].val));
            }
        } else {
            // We have not seen this effect before, making new effect entry
            vector<pair<int, int> > &vec = effect_conditions.insert(
                make_pair(effects[i].var, make_pair(
                              effects[i].val, vector<pair<int, int> > ()))).first->second.second;
            for (size_t j = 0; j < effects[i].conditions.size(); ++j) {
                vec.push_back(make_pair(effects[i].conditions[j].var, effects[i].conditions[j].val));
            }
        }
    }

    // For all those effect propositions whose variables do not take on different values...
    map<int, pair<int, vector<pair<int, int> > > >::iterator it =
        effect_conditions.begin();
    for (; it != effect_conditions.end(); ++it) {
        if (nogood_effect_vars.find(it->first) != nogood_effect_vars.end()) {
            continue;
        }
        // ...go through all the conditions that the effect has, and map condition
        // variables to the set of values they take on (in unique_conds)
        map<int, set<int> > unique_conds;
        vector<pair<int, int> > &conds = it->second.second;
        for (size_t i = 0; i < conds.size(); ++i) {
            if (unique_conds.find(conds[i].first) != unique_conds.end()) {
                unique_conds.find(conds[i].first)->second.insert(
                    conds[i].second);
            } else {
                set<int> &the_set = unique_conds.insert(make_pair(
                                                            conds[i].first, set<int> ())).first->second;
                the_set.insert(conds[i].second);
            }
        }
        // Check for each condition variable whether the number of values it takes on is
        // equal to the domain of that variable...
        pair<int, int> effect = make_pair(it->first, it->second.first);
        bool is_always_reached = true;
        map<int, set<int> >::iterator it2 = unique_conds.begin();
        for (; it2 != unique_conds.end(); ++it2) {
            bool is_surely_reached_by_var = false;
            int num_values_for_cond = it2->second.size();
            int num_values_of_variable = g_variable_domain[it2->first];
            if (num_values_for_cond == num_values_of_variable) {
                is_surely_reached_by_var = true;
            }
            // ...or else if the condition variable is the same as the effect variable,
            // check whether the condition variable takes on all other values except the
            // effect value
            else if (it2->first == it->first &&
                     num_values_for_cond == num_values_of_variable - 1) {
                // Number of different values is correct, now ensure that the effect value
                // was the one missing
                it2->second.insert(it->second.first);
                num_values_for_cond = it2->second.size();
                if (num_values_for_cond == (int) g_variable_domain[it2->first]) {
                    is_surely_reached_by_var = true;
                }
            }
            // If one of the condition variables does not fulfill the criteria, the effect
            // is not certain to happen
            if (!is_surely_reached_by_var)
                is_always_reached = false;
        }
        if (is_always_reached)
            eff.insert(effect);
    }
    return eff.empty();
}

bool LandmarkFactory::interferes(const LandmarkNode *node_a,
                                 const LandmarkNode *node_b) const {
    /* Facts a and b interfere (i.e., achieving b before a would mean having to delete b
     and re-achieve it in order to achieve a) if one of the following condition holds:
     1. a and b are mutex
     2. All actions that add a also add e, and e and b are mutex
     3. There is a greedy necessary predecessor x of a, and x and b are mutex
     This is the definition of Hoffmann et al. except that they have one more condition:
     "all actions that add a delete b". However, in our case (SAS+ formalism), this condition
     is the same as 2.
     */
    assert(node_a != node_b);
    assert(!node_a->disjunctive && !node_b->disjunctive);

    for (size_t bi = 0; bi < node_b->vars.size(); ++bi) {
        pair<const int, int> b = make_pair(node_b->vars[bi], node_b->vals[bi]);
        for (size_t ai = 0; ai < node_a->vars.size(); ++ai) {
            pair<const int, int> a = make_pair(node_a->vars[ai], node_a->vals[ai]);

            if (a.first == b.first && a.second == b.second) {
                if (!node_a->conjunctive || !node_b->conjunctive)
                    return false;
                else
                    continue;
            }

            // 1. a, b mutex
            // TODO(issue635): Use Fact struct right away.
            if (are_mutex(Fact(a.first, a.second), Fact(b.first, b.second)))
                return true;

            // 2. Shared effect e in all operators reaching a, and e, b are mutex
            // Skip this for conjunctive nodes a, as they are typically achieved through a
            // sequence of operators successively adding the parts of a
            if (node_a->conjunctive)
                continue;

            unordered_map<int, int> shared_eff;
            bool init = true;
            const vector<int> &ops = lm_graph->get_operators_including_eff(a);
            // Intersect operators that achieve a one by one
            for (size_t i = 0; i < ops.size(); ++i) {
                const GlobalOperator &op = lm_graph->get_operator_for_lookup_index(ops[i]);
                // If no shared effect among previous operators, break
                if (!init && shared_eff.empty())
                    break;
                // Else, insert effects of this operator into set "next_eff" if
                // it is an unconditional effect or a conditional effect that is sure to
                // happen. (Such "trivial" conditions can arise due to our translator,
                // e.g. in Schedule. There, the same effect is conditioned on a disjunction
                // of conditions of which one will always be true. We test for a simple kind
                // of these trivial conditions here.)
                const vector<GlobalEffect> &effects = op.get_effects();
                set<pair<int, int> > trivially_conditioned_effects;
                bool trivial_conditioned_effects_found = effect_always_happens(effects,
                                                                               trivially_conditioned_effects);
                unordered_map<int, int> next_eff;
                for (size_t j = 0; j < effects.size(); ++j) {
                    if (effects[j].conditions.empty() && effects[j].var != a.first) {
                        next_eff.insert(make_pair(effects[j].var, effects[j].val));
                    } else if (trivial_conditioned_effects_found
                               && trivially_conditioned_effects.find(make_pair(
                                                                         effects[j].var, effects[j].val))
                               != trivially_conditioned_effects.end())
                        next_eff.insert(make_pair(effects[j].var, effects[j].val));
                }
                // Intersect effects of this operator with those of previous operators
                if (init)
                    swap(shared_eff, next_eff);
                else {
                    unordered_map<int, int> result;
                    for (const auto &eff1 : shared_eff) {
                        auto it2 = next_eff.find(eff1.first);
                        if (it2 != next_eff.end() && it2->second == eff1.second)
                            result.insert(eff1);
                    }
                    swap(shared_eff, result);
                }
                init = false;
            }
            // Test whether one of the shared effects is inconsistent with b
            for (const auto &eff : shared_eff)
                if (eff != a && eff != b && are_mutex(Fact(eff.first, eff.second), Fact(b.first, b.second)))
                    return true;
        }

        /* // Experimentally commenting this out -- see issue202.
        // 3. Exists LM x, inconsistent x, b and x->_gn a
        for (const auto &parent : node_a->parents) {
            const LandmarkNode &node = *parent.first;
            edge_type edge = parent.second;
            for (size_t i = 0; i < node.vars.size(); ++i) {
                pair<const int, int> parent_prop = make_pair(node.vars[i], node.vals[i]);
                if (edge >= greedy_necessary && parent_prop != b && are_mutex(
                        parent_prop, b))
                    return true;
            }
        }
        */
    }
    // No inconsistency found
    return false;
}

void LandmarkFactory::approximate_reasonable_orders(bool obedient_orders) {
    /* Approximate reasonable and obedient reasonable orders according to Hoffmann et al. If flag
    "obedient_orders" is true, we calculate obedient reasonable orders, otherwise reasonable orders.

    If node_p is in goal, then any node2_p which interferes with node_p can be reasonably ordered
    before node_p. Otherwise, if node_p is greedy necessary predecessor of node2, and there is another
    predecessor "parent" of node2, then parent and all predecessors of parent can be ordered reasonably
    before node_p if they interfere with node_p.
    */
    for (set<LandmarkNode *>::iterator it = lm_graph->get_nodes().begin(); it != lm_graph->get_nodes().end(); ++it) {
        LandmarkNode *node_p = *it;
        if (node_p->disjunctive)
            continue;

        if (node_p->is_true_in_state(g_initial_state()))
            return;

        if (!obedient_orders && node_p->is_goal()) {
            for (set<LandmarkNode *>::iterator it2 = lm_graph->get_nodes().begin(); it2
                 != lm_graph->get_nodes().end(); ++it2) {
                LandmarkNode *node2_p = *it2;
                if (node2_p == node_p || node2_p->disjunctive)
                    continue;
                if (interferes(node2_p, node_p)) {
                    edge_add(*node2_p, *node_p, reasonable);
                }
            }
        } else {
            // Collect candidates for reasonable orders in "interesting nodes".
            // Use hash set to filter duplicates.
            unordered_set<LandmarkNode *> interesting_nodes(g_variable_name.size());
            for (const auto &child : node_p->children) {
                const LandmarkNode &node2 = *child.first;
                const edge_type &edge2 = child.second;
                if (edge2 >= greedy_necessary) { // found node2: node_p ->_gn node2
                    for (const auto &p : node2.parents) {   // find parent
                        LandmarkNode &parent = *(p.first);
                        const edge_type &edge = p.second;
                        if (parent.disjunctive)
                            continue;
                        if ((edge >= natural || (obedient_orders && edge == reasonable)) &&
                            &parent != node_p) {  // find predecessors or parent and collect in
                            // "interesting nodes"
                            interesting_nodes.insert(&parent);
                            collect_ancestors(interesting_nodes, parent,
                                              obedient_orders);
                        }
                    }
                }
            }
            // Insert reasonable orders between those members of "interesting nodes" that interfere
            // with node_p.
            for (LandmarkNode *node : interesting_nodes) {
                if (node == node_p || node->disjunctive)
                    continue;
                if (interferes(node, node_p)) {
                    if (!obedient_orders)
                        edge_add(*node, *node_p, reasonable);
                    else
                        edge_add(*node, *node_p, obedient_reasonable);
                }
            }
        }
    }
}

void LandmarkFactory::collect_ancestors(
    unordered_set<LandmarkNode *> &result,
    LandmarkNode &node,
    bool use_reasonable) {
    /* Returns all ancestors in the landmark graph of landmark node "start" */

    // There could be cycles if use_reasonable == true
    list<LandmarkNode *> open_nodes;
    unordered_set<LandmarkNode *> closed_nodes;
    for (const auto &p : node.parents) {
        LandmarkNode &parent = *(p.first);
        const edge_type &edge = p.second;
        if (edge >= natural || (use_reasonable && edge == reasonable))
            if (closed_nodes.count(&parent) == 0) {
                open_nodes.push_back(&parent);
                closed_nodes.insert(&parent);
                result.insert(&parent);
            }

    }
    while (!open_nodes.empty()) {
        LandmarkNode &node2 = *(open_nodes.front());
        for (const auto &p : node2.parents) {
            LandmarkNode &parent = *(p.first);
            const edge_type &edge = p.second;
            if (edge >= natural || (use_reasonable && edge == reasonable)) {
                if (closed_nodes.count(&parent) == 0) {
                    open_nodes.push_back(&parent);
                    closed_nodes.insert(&parent);
                    result.insert(&parent);
                }
            }
        }
        open_nodes.pop_front();
    }
}

void LandmarkFactory::edge_add(LandmarkNode &from, LandmarkNode &to,
                               edge_type type) {
    /* Adds an edge in the landmarks graph if there is no contradicting edge (simple measure to
    reduce cycles. If the edge is already present, the stronger edge type wins.
    */
    assert(&from != &to);
    assert(from.parents.find(&to) == from.parents.end() || type <= reasonable);
    assert(to.children.find(&from) == to.children.end() || type <= reasonable);

    if (type == reasonable || type == obedient_reasonable) { // simple cycle test
        if (from.parents.find(&to) != from.parents.end()) { // Edge in opposite direction exists
            //cout << "edge in opposite direction exists" << endl;
            if (from.parents.find(&to)->second > type) // Stronger order present, return
                return;
            // Edge in opposite direction is weaker, delete
            from.parents.erase(&to);
            to.children.erase(&from);
        }
    }

    // If edge already exists, remove if weaker
    if (from.children.find(&to) != from.children.end() && from.children.find(
            &to)->second < type) {
        from.children.erase(&to);
        assert(to.parents.find(&from) != to.parents.end());
        to.parents.erase(&from);

        assert(to.parents.find(&from) == to.parents.end());
        assert(from.children.find(&to) == from.children.end());
    }
    // If edge does not exist (or has just been removed), insert
    if (from.children.find(&to) == from.children.end()) {
        assert(to.parents.find(&from) == to.parents.end());
        from.children.insert(make_pair(&to, type));
        to.parents.insert(make_pair(&from, type));
        //cout << "added parent with address " << &from << endl;
    }
    assert(from.children.find(&to) != from.children.end());
    assert(to.parents.find(&from) != to.parents.end());
}

void LandmarkFactory::discard_noncausal_landmarks() {
    int number_of_noncausal_landmarks = 0;
    bool change = true;
    while (change) {
        change = false;
        for (set<LandmarkNode *>::const_iterator it = lm_graph->get_nodes().begin(); it
             != lm_graph->get_nodes().end(); ++it) {
            LandmarkNode *n = *it;
            if (!is_causal_landmark(*n)) {
                cout << "Discarding non-causal landmark: ";
                lm_graph->dump_node(n);
                lm_graph->rm_landmark_node(n);
                ++number_of_noncausal_landmarks;
                change = true;
                break;
            }
        }
    }
    cout << "Discarded " << number_of_noncausal_landmarks
         << " non-causal landmarks" << endl;
}

void LandmarkFactory::discard_disjunctive_landmarks() {
    /* Using disjunctive landmarks during landmark generation can be
    beneficial even if we don't want to use disunctive landmarks during s
    search. This function deletes all disjunctive landmarks that have been
    found. (Note: this is implemented inefficiently because "nodes" contains
    pointers, not the actual nodes. We should change that.)
    */
    if (lm_graph->number_of_disj_landmarks() == 0)
        return;
    cout << "Discarding " << lm_graph->number_of_disj_landmarks()
         << " disjunctive landmarks" << endl;
    bool change = true;
    while (change) {
        change = false;
        for (set<LandmarkNode *>::const_iterator it = lm_graph->get_nodes().begin(); it
             != lm_graph->get_nodes().end(); ++it) {
            LandmarkNode *n = *it;
            if (n->disjunctive) {
                lm_graph->rm_landmark_node(n);
                change = true;
                break;
            }
        }
    }
    // [Malte] Commented out the following assertions because
    // the old methods for this are no longer available.
    // assert(lm_graph->number_of_disj_landmarks() == 0);
    // assert(disj_lms_to_nodes.size() == 0);
}

void LandmarkFactory::discard_conjunctive_landmarks() {
    if (lm_graph->number_of_conj_landmarks() == 0)
        return;
    cout << "Discarding " << lm_graph->number_of_conj_landmarks()
         << " conjunctive landmarks" << endl;
    bool change = true;
    while (change) {
        change = false;
        for (set<LandmarkNode *>::const_iterator it = lm_graph->get_nodes().begin(); it
             != lm_graph->get_nodes().end(); ++it) {
            LandmarkNode *n = *it;
            if (n->conjunctive) {
                lm_graph->rm_landmark_node(n);
                change = true;
                break;
            }
        }
    }
    // [Malte] Commented out the following assertion because
    // the old method for this is no longer available.
    // assert(number_of_conj_landmarks() == 0);
}

void LandmarkFactory::discard_all_orderings() {
    cout << "Removing all orderings." << endl;
    for (set<LandmarkNode *>::iterator it =
             lm_graph->get_nodes().begin(); it != lm_graph->get_nodes().end(); ++it) {
        LandmarkNode &lmn = **it;
        lmn.children.clear();
        lmn.parents.clear();
    }
}

void LandmarkFactory::mk_acyclic_graph() {
    unordered_set<LandmarkNode *> acyclic_node_set(lm_graph->number_of_landmarks());
    int removed_edges = 0;
    for (set<LandmarkNode *>::iterator it = lm_graph->get_nodes().begin(); it != lm_graph->get_nodes().end(); ++it) {
        LandmarkNode &lmn = **it;
        if (acyclic_node_set.find(&lmn) == acyclic_node_set.end())
            removed_edges += loop_acyclic_graph(lmn, acyclic_node_set);
    }
    // [Malte] Commented out the following assertion because
    // the old method for this is no longer available.
    // assert(acyclic_node_set.size() == number_of_landmarks());
    cout << "Removed " << removed_edges
         << " reasonable or obedient reasonable orders\n";
}

bool LandmarkFactory::remove_first_weakest_cycle_edge(LandmarkNode *cur,
                                                      list<pair<LandmarkNode *, edge_type>> &path, list<pair<LandmarkNode *,
                                                                                                             edge_type>>::iterator it) {
    LandmarkNode *parent_p = 0;
    LandmarkNode *child_p = 0;
    for (list<pair<LandmarkNode *, edge_type> >::iterator it2 = it; it2
         != path.end(); ++it2) {
        edge_type edge = it2->second;
        if (edge == reasonable || edge == obedient_reasonable) {
            parent_p = it2->first;
            if (*it2 == path.back()) {
                child_p = cur;
                break;
            } else {
                list<pair<LandmarkNode *, edge_type> >::iterator child_it = it2;
                ++child_it;
                child_p = child_it->first;
            }
            if (edge == obedient_reasonable)
                break;
            // else no break since o_r order could still appear in list
        }
    }
    assert(parent_p != 0 && child_p != 0);
    assert(parent_p->children.find(child_p) != parent_p->children.end());
    assert(child_p->parents.find(parent_p) != child_p->parents.end());
    parent_p->children.erase(child_p);
    child_p->parents.erase(parent_p);
    return true;
}

int LandmarkFactory::loop_acyclic_graph(LandmarkNode &lmn,
                                        unordered_set<LandmarkNode *> &acyclic_node_set) {
    assert(acyclic_node_set.find(&lmn) == acyclic_node_set.end());
    int nr_removed = 0;
    list<pair<LandmarkNode *, edge_type> > path;
    unordered_set<LandmarkNode *> visited = unordered_set<LandmarkNode *>(lm_graph->number_of_landmarks());
    LandmarkNode *cur = &lmn;
    while (true) {
        assert(acyclic_node_set.find(cur) == acyclic_node_set.end());
        if (visited.find(cur) != visited.end()) { // cycle
            // find other occurrence of cur node in path
            list<pair<LandmarkNode *, edge_type> >::iterator it;
            for (it = path.begin(); it != path.end(); ++it) {
                if (it->first == cur)
                    break;
            }
            assert(it != path.end());
            // remove edge from graph
            remove_first_weakest_cycle_edge(cur, path, it);
            //assert(removed);
            ++nr_removed;

            path.clear();
            cur = &lmn;
            visited.clear();
            continue;
        }
        visited.insert(cur);
        bool empty = true;
        for (const auto &child : cur->children) {
            LandmarkNode *child_p = child.first;
            edge_type edge = child.second;
            if (acyclic_node_set.find(child_p) == acyclic_node_set.end()) {
                path.push_back(make_pair(cur, edge));
                cur = child_p;
                empty = false;
                break;
            }
        }
        if (!empty)
            continue;

        // backtrack
        visited.erase(cur);
        acyclic_node_set.insert(cur);
        if (!path.empty()) {
            cur = path.back().first;
            path.pop_back();
            visited.erase(cur);
        } else
            break;
    }
    assert(acyclic_node_set.find(&lmn) != acyclic_node_set.end());
    return nr_removed;
}

int LandmarkFactory::calculate_lms_cost() const {
    int result = 0;
    for (set<LandmarkNode *>::const_iterator it = lm_graph->get_nodes().begin(); it
         != lm_graph->get_nodes().end(); ++it)
        result += (*it)->min_cost;

    return result;
}

void LandmarkFactory::compute_predecessor_information(
    LandmarkNode *bp,
    vector<vector<int> > &lvl_var,
    std::vector<std::unordered_map<std::pair<int, int>, int> > &lvl_op) {
    /* Collect information at what time step propositions can be reached
    (in lvl_var) in a relaxed plan that excludes bp, and similarly
    when operators can be applied (in lvl_op).  */

    relaxed_task_solvable(lvl_var, lvl_op, true, bp);
}

void LandmarkFactory::calc_achievers() {
    for (set<LandmarkNode *>::iterator node_it = lm_graph->get_nodes().begin(); node_it
         != lm_graph->get_nodes().end(); ++node_it) {
        LandmarkNode &lmn = **node_it;

        for (size_t i = 0; i < lmn.vars.size(); ++i) {
            const vector<int> &ops = lm_graph->get_operators_including_eff(
                make_pair(lmn.vars[i], lmn.vals[i]));
            lmn.possible_achievers.insert(ops.begin(), ops.end());

            if (g_axiom_layers[lmn.vars[i]] != -1)
                lmn.is_derived = true;
        }

        vector<vector<int> > lvl_var;
        vector<unordered_map<pair<int, int>, int> > lvl_op;
        compute_predecessor_information(&lmn, lvl_var, lvl_op);

        set<int>::iterator ach_it;
        for (ach_it = lmn.possible_achievers.begin(); ach_it
             != lmn.possible_achievers.end(); ++ach_it) {
            int op_id = *ach_it;
            const GlobalOperator &op = lm_graph->get_operator_for_lookup_index(op_id);

            if (_possibly_reaches_lm(op, lvl_var, &lmn)) {
                lmn.first_achievers.insert(op_id);
            }
        }
    }
}
}
