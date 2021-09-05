#ifndef SYMMETRIES_GROUP_H
#define SYMMETRIES_GROUP_H
#include <vector>
#include "permutation.h"
#include "../task_proxy.h"
#include "../option_parser.h"

using namespace std;
using namespace __gnu_cxx;
typedef std::vector<short int> Trace;
typedef std::vector<Permutation> Gens; // the generators for the automorphism


class Group{
public:
    Group() {}
    ~Group();
    void initialize();

    static void add_permutation(void*, unsigned int, const unsigned int *);

    // Direct product creation methods.
    void default_direct_product();

    void add_generator(Permutation gen);
    int get_num_generators() const;
    void dump_generators() const;
    void get_canonical_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                             std::vector<int> &canonical_values, std::vector<ap_float> &canonical_num_values) const;
    bool to_canonical_state(PackedStateBin* buffer, std::vector<ap_float> &num_values) const;

    void free_memory();

private:
    std::vector<Trace> sub_groups;

    Trace init_group;
    Gens generators;

    std::vector<Gens> all_elements;
    static bool safe_to_add_generators;

    static int num_identity_generators;

    // Methods for finding a canonical state for each state.
    void calculate_canonical_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                                   std::vector<int> &canonical_values, std::vector<ap_float> &canonical_num_values) const;
    const Permutation& get_permutation(int) const;

    void calculate_canonical_state_subgroup(int ind, std::vector<int> &values, std::vector<ap_float> &num_values) const;
    bool to_canonical_state_subgroup(int ind, PackedStateBin* buffer, std::vector<ap_float> &num_values) const;

    void dump_subgroups() const;
    void dump_subgroups_verbous() const;
};

#endif
