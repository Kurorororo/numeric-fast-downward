#ifndef SYMMETRIES_PERMUTATION_H
#define SYMMETRIES_PERMUTATION_H
#include "../task_proxy.h"
#include <vector>
#include <string>
#include <fstream>

class Permutation{
public:
    Permutation();
    Permutation(const unsigned int*);
    Permutation(const Permutation&, bool invert=false);
    Permutation(const Permutation& perm1, const Permutation& perm2);
    ~Permutation();

    Permutation& operator =(const Permutation&);
    bool operator ==(const Permutation&) const;

    Permutation& inverse() const;
    bool identity() const;
    bool greedy_permutation_step(std::vector<int> &values, std::vector<ap_float> &num_values,
                                 std::vector<int> &new_values, std::vector<ap_float> &new_num_values) const;
    void permutation_on_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                              std::vector<int> &new_values, std::vector<ap_float> &new_num_values) const;
    void set_value(int ind, int val);
    int get_value(int ind) const { return value[ind]; }
    int get_inverse_value(int ind) const { return inverse_value[ind]; }

    static unsigned int length;
    static vector<int> var_by_val;
    static vector<int> dom_sum_by_var;
    static int dom_sum_num_var;

    bool replace_if_less(std::vector<int> &values, std::vector<ap_float> &num_values);

private:
    int* value;
    int* inverse_value;
    vector<int> vars_affected;
    vector<bool> affected;
    vector<int> num_vars_affected;
    vector<bool> num_affected;
    bool borrowed_buffer;
    // Need to keep the connection between affected vars, ie which var goes into which.
    vector<int> from_vars;
    vector<int> from_num_vars;
    // Affected vars by cycles
    vector<vector<int> > affected_vars_cycles;
    vector<vector<int> > affected_num_vars_cycles;

    bool cmp_less_short(const std::vector<int> &l_values, const std::vector<ap_float> &l_num_values,
                        const std::vector<int> &r_values, const std::vector<ap_float> &r_num_values) const;

    void set_affected(int ind, int val);
    bool is_numeric(int ind) const { return ind >= dom_sum_num_var; }
    int get_var_by_index(int ind) const;
    int get_value_by_index(int ind, int var) const;
    int get_num_var_by_index(int ind) const;
    pair<int, int> get_new_var_val_by_old_var_val(int var, int value) const;
    int get_new_num_var_by_old_num_var(int var) const;

    void finalize();
    void _allocate();
    void _deallocate();
    void _copy_value_from_permutation(const Permutation&);
    void _inverse_value_from_permutation(const Permutation &perm);
};

#endif
