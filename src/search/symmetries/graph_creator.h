#ifndef SYMMETRIES_GRAPH_CREATOR_H
#define SYMMETRIES_GRAPH_CREATOR_H

//#include "../bliss-0.5/graph.hh"
#include <graph.hh>
#include "group.h"
#include "../plugin.h"
#include "../task_proxy.h"

enum SymmetryBasedSearchType {
    NO_SYMMETRIES,
    INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH,
    GOAL_ONLY_STABILIZED_REGULAR_SEARCH,
    GOAL_ONLY_STABILIZED_ORBIT_SEARCH,
    GOAL_ONLY_STABILIZED_NO_SEARCH
};


enum color_t {PREDICATE_VERTEX, VALUE_VERTEX, NUM_VAR_VERTEX, GOAL_VERTEX, GTE_VERTEX, GT_VERTEX, NUM_EFF_VERTEX, MAX_VALUE};

/**
 * This class will create a bliss graph which will be used to find the
 * automorphism groups
 */

class GraphCreator  {
    SymmetryBasedSearchType search_type;
    bool add_generators_powers;

    bool no_search;
    bool initialized;

    int time_bound;
    int generators_bound;
    ap_float precision;

public:

    GraphCreator(const Options &opts);
    virtual ~GraphCreator();

    void initialize(const TaskProxy &task_proxy);

    void get_canonical_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                             std::vector<int> &canonical_values, std::vector<ap_float> &canonical_num_values) const {
        group.get_canonical_state(values, num_values, canonical_values, canonical_num_values);
    }

    static void add_options_to_parser(OptionParser &parser);

    SymmetryBasedSearchType get_search_type() const { return search_type; }

    void free_memory() { group.free_memory(); }

private:
    Group group;

    int get_multiplicator(ap_float value) const;
    bliss::Graph* create_bliss_directed_graph(const TaskProxy &task_proxy) const;
};

#endif

