#include "permutation.h"
#include "../globals.h"
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
//#include <assert.h>
#include <cassert>

using namespace std;
#include <sstream>

unsigned int Permutation::length;
vector<int> Permutation::var_by_val;
vector<int> Permutation::dom_sum_by_var;
int Permutation::dom_sum_num_var;
vector<int> Permutation::num_var_to_regular_id;
vector<int> Permutation::regular_id_to_num_var;


void Permutation::_allocate() {
    value = new int[length];
    inverse_value = new int[length];
    affected.assign(g_variable_domain.size(), false);
    num_affected.assign(g_variable_domain.size(), false);
    vars_affected.clear();
    num_vars_affected.clear();
    from_vars.assign(g_variable_domain.size(), -1);
    from_num_vars.assign(g_numeric_var_types.size(), -1);
    affected_vars_cycles.clear();
    affected_num_vars_cycles.clear();
}

void Permutation::_deallocate() {
    delete[] value;
    delete[] inverse_value;
}

void Permutation::_copy_value_from_permutation(const Permutation &perm) {
    for (int i = 0; i < length; i++)
        set_value(i, perm.get_value(i));
}

void Permutation::_inverse_value_from_permutation(const Permutation &perm) {
    for (int i = 0; i < length; i++)
        set_value(perm.get_value(i), i);
}

Permutation &Permutation::operator=(const Permutation &other) {
    if (this != &other) {
        affected.assign(g_variable_domain.size(), false);
        num_affected.assign(g_numeric_var_types.size(), false);
        vars_affected.clear();
        num_vars_affected.clear();
        from_vars.assign(g_variable_domain.size(), -1);
        from_num_vars.assign(g_numeric_var_types.size(), -1);
        affected_vars_cycles.clear();
        affected_num_vars_cycles.clear();
        _copy_value_from_permutation(other);
    }
    this->finalize();
    return *this;
}

Permutation::Permutation(){
    _allocate();
    for (int i = 0; i < length; i++)
        set_value(i,i);
    finalize();
}


Permutation::Permutation(const unsigned int* full_permutation){
    _allocate();
    for (int i = 0; i < length; i++){
        set_value(i,full_permutation[i]);
    }
    finalize();
}

Permutation::Permutation(const Permutation& perm, bool invert){
    _allocate();
    if (invert) {
        /*

        for (int i = 0; i < length; i++)
            set_value(perm.get_value(i), i);
        */
        _inverse_value_from_permutation(perm);
    } else {
        _copy_value_from_permutation(perm);
    }
    finalize();
}

// New constructor to use instead of * operator
Permutation::Permutation(const Permutation& perm1, const Permutation& perm2){
    _allocate();

    for (int i = 0; i < length; i++) {
        set_value(i, perm2.get_value(perm1.get_value(i)));
    }

//	cout << "Finalizing" << endl;
    finalize();
}



Permutation::~Permutation(){
    _deallocate();
}

void Permutation::finalize(){
    // Sorting the vector of affected variables
    ::sort(vars_affected.begin(), vars_affected.end());

    // Going over the vector from_vars of the mappings of the variables and finding cycles
//	affected_vars_cycles.clear();
    vector<bool> marked;
    marked.assign(g_variable_domain.size(), false);
    for (int i = 0, n = from_vars.size(); i < n; i++) {
        if (marked[i] || from_vars[i] == -1)
            continue;

        int current = i;
        marked[current] = true;
        vector<int> cycle;
        cycle.push_back(current);

        while (from_vars[current] != i) {
            current = from_vars[current];
            marked[current] = true;
            cycle.insert(cycle.begin(), current);
        }
        // Get here when from_vars[current] == i.
        affected_vars_cycles.push_back(cycle);
    }

    ::sort(num_vars_affected.begin(), num_vars_affected.end());

    vector<bool> num_marked;
    num_marked.assign(g_numeric_var_types.size(), false);
    for (int i = 0, n = from_num_vars.size(); i < n; ++i) {
        if (num_marked[i] || from_num_vars[i] == -1)
            continue;

        int current = i;
        num_marked[current] = true;
        vector<int> cycle;
        cycle.push_back(current);

        while (from_num_vars[current] != i) {
            current = from_num_vars[current];
            num_marked[current] = true;
            cycle.insert(cycle.begin(), current);
        }
        affected_num_vars_cycles.push_back(cycle);
    }
}


// Deprecated
Permutation& Permutation::inverse() const{
    Permutation *perm = new Permutation();
    for(int i=0; i < length; i++){
        perm->set_value(get_value(i), i);
    }
    perm->finalize();
    return *perm;
}

bool Permutation::identity() const{
    return vars_affected.size() == 0 && num_vars_affected.size() == 0;
}

bool Permutation::operator ==(const Permutation &other) const{

    for(int i = 0; i < length; i++) {
        if (get_value(i) != other.get_value(i)) return false;
    }

    return true;
}


void Permutation::permutation_on_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                                       std::vector<int> &new_values, std::vector<ap_float> &new_num_values) const {
    for (int i = 0, n = vars_affected.size(); i < n; i++) {
        int var = vars_affected[i];
        int value = values[var];
        pair<int, int> new_var_val = get_new_var_val_by_old_var_val(var, value);
        int new_var = new_var_val.first;
        int new_val = new_var_val.second;
        if (var == new_var && value == new_val){
            new_values[var] = value;
        } else {
            //cout << "Value " << new_val << " for variable " << new_var << endl;
            if (new_val< 0 || new_val >= g_variable_domain[new_var])
                cout << "Value " << new_val << " for variable " << new_var << endl;
            new_values[new_var] = new_val;
        }
    }

    for (int i = 0, n = num_vars_affected.size(); i < n; i++) {
        int var = num_vars_affected[i];
        int new_var = get_new_num_var_by_old_num_var(var);
        if (new_var == var){
            new_num_values[var] = num_values[var];
        } else {
            new_num_values[new_var] = num_values[var];
        }
    }
}

bool Permutation::cmp_less_short(const std::vector<int> &l_values, const std::vector<ap_float> &l_num_values,
                                 const std::vector<int> &r_values, const std::vector<ap_float> &r_num_values) const {
    for(int i = vars_affected.size()-1; i >= 0; --i) {
        if (l_values[vars_affected[i]] > r_values[vars_affected[i]]) return false;
        if (l_values[vars_affected[i]] < r_values[vars_affected[i]]) return true;
    }
    for(int i = num_vars_affected.size()-1; i >= 0; --i) {
        if (l_num_values[num_vars_affected[i]] > r_num_values[num_vars_affected[i]]) return false;
        if (l_num_values[num_vars_affected[i]] < r_num_values[num_vars_affected[i]]) return true;
    }
    return false;
}

int Permutation::get_var_by_index(int ind) const {
    if (ind < g_variable_domain.size()) {
        cout << "=====> WARNING!!!! Check that this is done on purpose!" << endl;
        return ind;
    }

    return var_by_val[ind - g_variable_domain.size()];
}

int Permutation::get_value_by_index(int ind, int var) const {
    if (ind < g_variable_domain.size() || ind >= dom_sum_num_var) {
        cout << "=====> WARNING!!!! Check that this is done on purpose!" << endl;
        return ind;
    }

    return ind - dom_sum_by_var[var];
}

int Permutation::get_num_var_by_index(int ind) const {
    if (ind < dom_sum_num_var) {
        cout << "=====> WARNING!!!! Check that this is done on purpose!" << endl;
        return ind;
    }

    return regular_id_to_num_var[ind - dom_sum_num_var];
}

pair<int, int> Permutation::get_new_var_val_by_old_var_val(int var, int value) const {
    int ind = get_value(g_variable_domain.size() + dom_sum_by_var[var] + value);
    int var = get_var_by_index(ind);
    int value = get_value_by_index(ind, var);

    return make_pair(var, value);
}

int Permutation::get_new_num_var_by_old_num_var(int num_var) const {
    int regular_id = num_var_to_regular_id[num_var];
    assert(regular_id != -1);
    int ind = get_value(g_variable_domain.size() + dom_sum_num_var + regular_id);

    return get_num_var_by_index(ind);
}

void Permutation::set_value(int ind, int val) {
    value[ind] = val;
    inverse_value[val] = ind;
    set_affected(ind, val);
}

void Permutation::set_affected(int ind, int val) {
    if (ind < g_variable_domain.size() || ind == val)
        return;

    if (ind < dom_sum_num_var) {
        int var = get_var_by_index(ind);
        int to_var = get_var_by_index(val);
        if (!affected[var]) {
            vars_affected.push_back(var);
            affected[var] = true;
        }
        if (!affected[to_var]) {
            vars_affected.push_back(to_var);
            affected[to_var] = true;
        }
        from_vars[to_var] = var;
    } else if (ind < dom_sum_num_var + g_numeric_var_types.size()) {
        int num_var = get_num_var_by_index(ind);
        int to_num_var = get_num_var_by_index(val);
        if (!num_affected[num_var]) {
            num_vars_affected.push_back(num_var);
            num_affected[num_var] = true;
        }
        if (!num_affected[to_num_var]) {
            num_vars_affected.push_back(to_num_var);
            num_affected[to_num_var] = true;
        }
        from_num_vars[to_num_var] = num_var;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// This method compares the state to the state resulting from permuting it.
// If the original state is bigger than the resulted one, it is rewritten with the latter and true is returned.
bool Permutation::replace_if_less(std::vector<int> &values, std::vector<ap_float> &num_values) const {
    if (identity())
        return false;

    bool values_same = true;
    // Going over the affected variables, comparing the resulted values with the state values.
    for (int i = vars_affected.size() - 1; i >= 0; i--) {
        int to_var = vars_affected[i];
        int from_var = from_vars[to_var];
        int from_val = values[from_var];
        pair<int, int> to_var_val = get_new_var_val_by_old_var_val(from_var, from_val);
        assert(to_var == to_var_val.first);
        int to_val = to_var_val.second;

        // Check if the values are the same, then continue to the next aff. var.
        if (to_val < values[to_var]) {
            values_same = false;
            break;
        } else if (to_val > values[to_var]) {
            return false;
        }
    }
    if (!values_same) {
        for (int i = 0, n = affected_vars_cycles.size(); i < n; i++) {
            if (affected_vars_cycles[i].size() == 1) {
                int var = affected_vars_cycles[i][0];
                int from_val = values[var];
                pair<int, int> to_var_val = get_new_var_val_by_old_var_val(var, from_val);
                assert(var == to_var_val.first);
                values[var] = to_var_val.second;
                continue;
            }
            // Remembering one value to be rewritten last
            int last_var = affected_vars_cycles[i][affected_vars_cycles[i].size() - 1];
            int last_val = values[last_var];

            for (int j = affected_vars_cycles[i].size() - 1; j > 0; j--) {
                // writing into variable affected_vars_cycles[i][j]
                int to_var = affected_vars_cycles[i][j];
                int from_var = affected_vars_cycles[i][j - 1];
                int from_val = values[from_var];
                pair<int, int> to_var_val = get_new_var_val_by_old_var_val(from_var, from_val);
                assert(to_var == to_var_val.first);
                values[to_var] = to_var_val.second;
            }
            // writing the last one
            pair<int, int> to_var_val = get_new_var_val_by_old_var_val(last_var, last_val);
            values[affected_vars_cycles[i][0]] = to_var_val.second;
        }
    } 

    bool num_values_same = true;
    for (int i = num_vars_affected.size() - 1; i >= 0; i--) {
        int to_var = num_vars_affected[i];
        int from_var = from_num_vars[to_var];

        // Check if the values are the same, then continue to the next aff. var.
        if (values_same && value[from_var] > values[to_var]) {
            return false;
        } else if (values[from_var] != values[to_var]) {
            num_values_same = false;
            break;
        }
    }
    if (!num_values_same) {
        for (int i = 0, n = affected_num_vars_cycles.size(); i < n; i++) {
            int last_var = affected_num_vars_cycles[i][affected_num_vars_cycles[i].size() - 1];
            int last_val = num_values[last_var];

            for (int j = affected_num_vars_cycles[i].size() - 1; j > 0; j--) {
                int to_num_var = affected_num_vars_cycles[i][j];
                int from_num_var = affected_num_vars_cycles[i][j - 1];
                num_values[to_num_var] = num_values[to_num_var];
            }

            num_values[affected_num_vars_cycles[i][0]] = last_val;
        }
    }

    return !values_same || !num_values_same;
}
