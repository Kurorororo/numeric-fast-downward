#include "graph_creator.h"

#include <iostream>
#include <vector>

#include "../utils/timer.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "permutation.h"

using namespace std;
using namespace numeric_helper;

GraphCreator::GraphCreator(const Options &opts)
    : search_type(SymmetryBasedSearchType(opts.get_enum("symmetries"))),
      no_search(opts.get<bool>("no_search")),
      initialized(false),
      precision(opts.get<ap_float>("precision")) {
}


void GraphCreator::initialize(const std::shared_ptr<AbstractTask> task) {
    if (initialized)
        return;

    initialized = true;
    cout << "Initializing symmetry " << endl;
    group.initialize();

    bliss::Digraph* graph = create_bliss_directed_graph(task);

    graph->set_splitting_heuristic(bliss::Digraph::shs_flm);

    bliss::Stats stats1;
    cout << "Using Bliss to find group generators" << endl;
    graph->canonical_form(stats1,&(Group::add_permutation),&group);
    cout << "Got " << group.get_num_generators() << " group generators, time step: [t=" << utils::g_timer << "]" << endl;

    group.default_direct_product();
    cout<<"Number of generators: "<< group.get_num_generators()<<endl;

    if (no_search)
        exit(0);

    // Deleting the graph
    delete graph;

    cout << "Done initializing symmetries [t=" << utils::g_timer << "]" << endl;
}



GraphCreator::~GraphCreator() {
    // Freeing the group
    free_memory();
    // Nothing to delete here
}

int GraphCreator::get_multiplicator(ap_float value) const {
    int m = 1;
    ap_float integral_part = 0;
    ap_float fractional_part = std::modf(value, &integral_part);
    while (std::fabs(fractional_part) > precision) {
        m *= 10;
        value = fractional_part * 10;
        fractional_part = std::modf(value, &integral_part);
    }

    return m;
}

bliss::Digraph* GraphCreator::create_bliss_directed_graph(const std::shared_ptr<AbstractTask> task) const {
   // initialization

    TaskProxy task_proxy(*task);
    NumericTaskProxy numeric_task(task_proxy, false, true);

    std::cout << "Computing number of vertices" << std::endl;
    VariablesProxy variables = task_proxy.get_variables();
    int num_of_vertex = 0;
    for (auto var : variables) {
        if (numeric_task.is_numeric_axiom(var.get_id())) {
            std::cout << "numeric axiom " << var.get_name() << std::endl;
            Permutation::var_to_regular_id.push_back(-1);
        } else {
            std::cout << "not numeric axiom " << var.get_name() << std::endl;
            Permutation::var_to_regular_id.push_back(num_of_vertex);
            Permutation::regular_id_to_var.push_back(var.get_id());
            ++num_of_vertex;
        }
    }
    for (auto var : variables) {
        if (!numeric_task.is_numeric_axiom(var.get_id())) {
            Permutation::dom_sum_by_regular_id.push_back(num_of_vertex);
            num_of_vertex += var.get_domain_size();
            for (int num_of_value = 0; num_of_value < var.get_domain_size(); ++num_of_value) {
                Permutation::var_by_val.push_back(var.get_id());
            }
        }
    }
    Permutation::dom_sum_num_var = num_of_vertex;
    NumericVariablesProxy num_variables = task_proxy.get_numeric_variables();
    for (auto num_var : num_variables) {
        if (num_var.get_var_type() == regular) {
            Permutation::num_var_to_regular_id.push_back(num_of_vertex - Permutation::dom_sum_num_var);
            Permutation::regular_id_to_num_var.push_back(num_var.get_id());
            ++num_of_vertex;
        } else {
            Permutation::num_var_to_regular_id.push_back(-1);
        }
    }
    Permutation::length = num_of_vertex;

    std::cout << "Building a PDG" << std::endl;
    bliss::Digraph* g = new bliss::Digraph();
    int idx = 0;
    // add vertex for each varaible
    for (auto var : variables) {
        if (!numeric_task.is_numeric_axiom(var.get_id())) {
            idx = g->add_vertex(PREDICATE_VERTEX);
        }
    }
    // now add values vertices for each predicate
    for (auto var : variables){
        if (!numeric_task.is_numeric_axiom(var.get_id())) {
            for (int j = 0; j < var.get_domain_size(); j++){
               idx = g->add_vertex(VALUE_VERTEX);
               g->add_edge(idx, Permutation::get_index_by_var(var.get_id()));
            }
        }
    }
    // add vertex for each numeric variable
    for (auto num_var : num_variables) {
        if (num_var.get_var_type() == regular) {
            idx = g->add_vertex(NUM_VAR_VERTEX);
        }
    }

    // calculate multiplicators to obtain colors from real values
    int cost_multiplicator = 1;
    int constant_multiplicator = 1;
    int constant_offset = 0;
    for (OperatorProxy op : task_proxy.get_operators()) {
        int m = get_multiplicator(op.get_cost());
        if (m > cost_multiplicator) cost_multiplicator = m;

        for (int pre : numeric_task.get_action_num_list(op.get_id())){
            for (int i : numeric_task.get_numeric_conditions_id(pre)){
                const LinearNumericCondition &lnc = numeric_task.get_condition(i);
                int m = get_multiplicator(lnc.constant);
                if (m > constant_multiplicator) constant_multiplicator = m;
                int multiplied_constant = constant_multiplicator * lnc.constant;
                if (constant_offset + multiplied_constant < 0) constant_offset = -1 * multiplied_constant;

                for (auto coeff : lnc.coefficients) {
                    int m = get_multiplicator(coeff);
                    if (m > constant_multiplicator) constant_multiplicator = m;
                    int multiplied_coeff = constant_multiplicator * coeff;
                    if (constant_offset + multiplied_coeff < 0) constant_offset = -1 * multiplied_coeff;
                }
            }

            for (auto k : numeric_task.get_action_num_list(op.get_id())) {
                int m = get_multiplicator(k);
                if (m > constant_multiplicator) constant_multiplicator = m;
                int multiplied_k = constant_multiplicator * k;
                if (constant_offset + multiplied_k < 0) constant_offset = -1 * multiplied_k;
            }

            for (int i = 0; i < numeric_task.get_action_n_linear_eff(op.get_id()); ++i) {
                ap_float constant = numeric_task.get_action_linear_constants(op.get_id())[i];
                int m = get_multiplicator(constant);
                if (m > constant_multiplicator) constant_multiplicator = m;
                int multiplied_constant = constant_multiplicator * constant;
                if (constant_offset + multiplied_constant < 0) constant_offset = -1 * multiplied_constant;

                for (auto coeff : numeric_task.get_action_linear_coefficients(op.get_id())[i]) {
                    int m = get_multiplicator(numeric_task.get_action_linear_constants(op.get_id())[i]);
                    if (m > constant_multiplicator) constant_multiplicator = m;
                    int multiplied_coeff = constant_multiplicator * coeff;
                    if (constant_offset + multiplied_coeff < 0) constant_offset = -1 * multiplied_coeff;
                }
            }
        }
    }
    for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
        for (int id_n_con : numeric_task.get_numeric_goals(id_goal)) {
            const LinearNumericCondition &lnc = numeric_task.get_condition(id_n_con);
            int m = get_multiplicator(lnc.constant);
            if (m > constant_multiplicator) constant_multiplicator = m;
            int multiplied_constant = constant_multiplicator * lnc.constant;
            if (constant_offset + multiplied_constant < 0) constant_offset = -1 * multiplied_constant;
        }
    }

    // calculate max color for operators
    int max_op_color = MAX_VALUE;
    for (OperatorProxy op : task_proxy.get_operators()) {
        int op_color = MAX_VALUE + op.get_cost() * cost_multiplicator;
        max_op_color = std::max(op_color, max_op_color);
    }
    int base_constant_color = max_op_color + 1;
    
    // now add vertices for operators
    OperatorsProxy operators = task_proxy.get_operators();
    for (auto op : operators) {
        int op_color = MAX_VALUE + op.get_cost() * cost_multiplicator;
        int op_idx = g->add_vertex(op_color);

        for (auto pre : op.get_preconditions()) {
            if (!numeric_task.is_numeric_axiom(pre.get_variable().get_id())) {
                int var = pre.get_variable().get_id();
                int pre_val = pre.get_value();
                int pre_idx = Permutation::get_index_by_var_val(var, pre_val);
                g->add_edge(pre_idx, op_idx);
            }
        }
        for (auto eff : op.get_effects()) {
            FactProxy eff_fact = eff.get_fact();
            int eff_idx = Permutation::get_index_by_var_val(eff_fact.get_variable().get_id(), eff_fact.get_value());
            g->add_edge(op_idx, eff_idx);
        }

        for (int pre : numeric_task.get_action_num_list(op.get_id())) {
            for (int i : numeric_task.get_numeric_conditions_id(pre)) {
                const LinearNumericCondition &lnc = numeric_task.get_condition(i);
                int lnc_idx = lnc.is_strictly_greater ? g->add_vertex(GT_VERTEX) : g->add_vertex(GTE_VERTEX);
                int constant_color = base_constant_color + constant_multiplicator * lnc.constant + constant_offset;
                int constant_idx = g->add_vertex(constant_color);
                g->add_edge(lnc_idx, constant_idx);
                g->add_edge(lnc_idx, op_idx);

                for (int j = 0, n = Permutation::regular_id_to_num_var.size(); j < n; ++j) {
                    int multiplied_coeff = lnc.coefficients[j] * constant_multiplicator + constant_offset;
                    if (multiplied_coeff > 0) {
                        int coeff_color = base_constant_color + multiplied_coeff;
                        int coeff_idx = g->add_vertex(coeff_color);
                        g->add_edge(coeff_idx, lnc_idx);
                        g->add_edge(Permutation::get_index_by_num_regular_id(j), coeff_idx);
                    }
                }
            }
        }

        for (int i = 0, n = Permutation::regular_id_to_num_var.size(); i < n; ++i) {
            ap_float k = numeric_task.get_action_eff_list(op.get_id())[i];
            ap_float multipleid_k = k * constant_multiplicator + constant_offset;
            if (multipleid_k > 0) {
                int k_color = base_constant_color + multipleid_k;
                int k_idx = g->add_vertex(k_color);
                int eff_idx = g->add_vertex(NUM_EFF_VERTEX);
                g->add_edge(k_idx, eff_idx);
                g->add_edge(eff_idx, Permutation::get_index_by_num_regular_id(i));
                g->add_edge(op_idx, eff_idx);
            }
        }

        for (int i = 0; i < numeric_task.get_action_n_linear_eff(op.get_id()); ++i) {
            int eff_idx = g->add_vertex(NUM_EFF_VERTEX);
            int lhs = numeric_task.get_action_linear_lhs(op.get_id())[i];
            g->add_edge(eff_idx, Permutation::get_index_by_num_regular_id(lhs));
            g->add_edge(op_idx, eff_idx);

            ap_float constant = numeric_task.get_action_linear_constants(op.get_id())[i];
            int multiplied_constant = constant * constant_multiplicator + constant_offset;
            if (multiplied_constant > 0) {
                int constant_color = base_constant_color + multiplied_constant;
                int constant_idx = g->add_vertex(constant_color);
                g->add_edge(constant_idx, eff_idx);
            }

            for (int j = 0, n = Permutation::regular_id_to_num_var.size(); j < n; ++j) {
                ap_float coeff = numeric_task.get_action_linear_coefficients(op.get_id())[i][j];
                int multiplied_coeff = coeff * constant_multiplicator + constant_offset;
                if (multiplied_coeff > 0) {
                    int coeff_color = base_constant_color + multiplied_coeff;
                    int coeff_idx = g->add_vertex(coeff_color);
                    int var_idx = Permutation::get_index_by_num_regular_id(j);
                    g->add_edge(coeff_idx, var_idx);
                    g->add_edge(var_idx, eff_idx);
                }
            }
        }
    }

    // adding edges from goal vertex
    int goal_idx = g->add_vertex(GOAL_VERTEX);
    for (size_t id_goal = 0; id_goal < numeric_task.get_n_numeric_goals(); ++id_goal) {
        for (pair<int, int> var_value: numeric_task.get_propositoinal_goals(id_goal)) {
            if (!numeric_task.is_numeric_axiom(var_value.first)) {
                int val_idx = Permutation::get_index_by_var_val(var_value.first, var_value.second);
                g->add_edge(goal_idx, val_idx);
            }
        }
        for (int id_n_con : numeric_task.get_numeric_goals(id_goal)) {
            LinearNumericCondition &lnc = numeric_task.get_condition(id_n_con);
            int lnc_idx = lnc.is_strictly_greater ? g->add_vertex(GT_VERTEX) : g->add_vertex(GTE_VERTEX);
            int constant_color = base_constant_color + constant_multiplicator * lnc.constant + constant_offset;
            int constant_idx = g->add_vertex(constant_color);
            g->add_edge(lnc_idx, constant_idx);
            g->add_edge(goal_idx, lnc_idx);

            for (int j = 0, n = Permutation::regular_id_to_num_var.size(); j < n; ++j) {
                int multiplied_coeff = lnc.coefficients[j] * constant_multiplicator + constant_offset;
                if (multiplied_coeff > 0) {
                    int coeff_color = base_constant_color + multiplied_coeff;
                    int coeff_idx = g->add_vertex(coeff_color);
                    g->add_edge(coeff_idx, lnc_idx);
                    g->add_edge(Permutation::get_index_by_num_regular_id(j), coeff_idx);
                }
            }
        }
    }

    return g;
}

void GraphCreator::add_options_to_parser(OptionParser &parser) {

    vector<string> sym_types;
    sym_types.push_back("none");
    sym_types.push_back("goal_only_orbit");
    sym_types.push_back("goal_only_no");
    parser.add_enum_option("symmetries", sym_types, "use symmetries", "none");

    parser.add_option<bool>("no_search",
                           "No search is performed, exiting after creating the symmetries",
                            "false");

    parser.add_option<ap_float>("precision", "Threshold below which is considered as zero", "0.00001");
}



static GraphCreator *_parse(OptionParser &parser) {
//	cout << "=========================================================================================================" << endl;
//	cout << "======     Running symmetry parser" << endl;
//	cout << "=========================================================================================================" << endl;
    GraphCreator::add_options_to_parser(parser);
    Options opts = parser.parse();

    SymmetryBasedSearchType type = SymmetryBasedSearchType(opts.get_enum("symmetries"));
    if (type == NO_SYMMETRIES) {
        return 0;
    }
    /*
    if (type == INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH) {
        // To enable this, just use stabilizing initial state in PDG.
        cout << "Init and goal stabilized symmetry pruning (Pochter et. al.) is disabled in this version" << endl;
        exit(1);
    }
    */
    if (!parser.dry_run()) {

    if (type == GOAL_ONLY_STABILIZED_ORBIT_SEARCH || type == GOAL_ONLY_STABILIZED_NO_SEARCH) {
        GraphCreator* gr = new GraphCreator(opts);
        if (gr) {
            cout << "Creating symmetry graph stabilizing goal only and using ";
            if (gr->get_search_type() == GOAL_ONLY_STABILIZED_ORBIT_SEARCH) {
                cout << "orbit ";
            } else
                cout << "no ";
            cout << "search" << endl;
        }

        return gr;
    }

    cout << "Invalid option for symmetry type" << endl;
    exit(1);
    } else {
        return 0;
    }

}


static Plugin<GraphCreator> _plugin("symmetry_state_pruning", _parse);
