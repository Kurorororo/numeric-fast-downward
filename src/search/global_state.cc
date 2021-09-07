#include "global_state.h"

#include "globals.h"
#include "state_registry.h"

#include <algorithm>
#include <iostream>
#include <cassert>
#include <cmath>
#include <sstream>
using namespace std;


GlobalState::GlobalState(const PackedStateBin *buffer_, const StateRegistry &registry_,
             StateID id_)
    : buffer(buffer_),
      registry(&registry_),
      id(id_) {
    assert(buffer);
    assert(id != StateID::no_state);
}

GlobalState::~GlobalState() {
}

container_int GlobalState::operator[](size_t index) const {
    return g_state_packer->get(buffer, index);
}

bool GlobalState::same_values(const GlobalState &state) const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        if (this->operator[](i) != state[i]) return false;
    }
    std::vector<ap_float> this_numeric_values = this->get_numeric_vars();
    std::vector<ap_float> numeric_values = state.get_numeric_vars();
    for (size_t i = 0; i < this_numeric_values.size(); ++i) {
        if (g_numeric_var_types[i] == regular) {
            if (std::fabs(this_numeric_values[i] - numeric_values[i]) > 0.00001) return false;
        }
    }
    return true;
}

bool GlobalState::same_values(const std::vector<container_int> &values, const std::vector<ap_float> &numeric_values) const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        if (this->operator[](i) != values[i]) return false;
    }
    std::vector<ap_float> this_numeric_values = this->get_numeric_vars();
    for (size_t i = 0; i < this_numeric_values.size(); ++i) {
        if (g_numeric_var_types[i] == regular) {
            if (std::fabs(this_numeric_values[i] - numeric_values[i]) > 0.00001) return false;
        }
    }
    return true;
}

void GlobalState::dump_pddl() const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        const string &fact_name = g_fact_names[i][(*this)[i]];
        if (fact_name != "<none of those>")
            cout << fact_name << endl;
    }
}

//std::vector<ap_float> GlobalState::get_instrumentation_vars() const {
//	vector<ap_float> instvars;
//	assert(g_initial_state_data.size() == g_variable_domain.size());
//	assert(g_initial_state_numeric.size() == g_numeric_var_types.size());
//	for (size_t i = g_initial_state_data.size(); i< g_variable_domain.size() + g_initial_state_numeric.size(); ++i) {
//		if(g_numeric_var_types[i-g_initial_state_data.size()] == instrumentation) {
//			instvars.push_back(g_state_packer->unpackDouble((*this)[i]));
//		}
//	}
//	return instvars;
//}

void GlobalState::dump_fdr() const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i)
        cout << "  #" << i << " [" << g_variable_name[i] << "] -> "
             << g_fact_names[i][(*this)[i]] << " (" << (*this)[i] << ")" << endl;
    vector<ap_float> numeric_vals = registry->get_numeric_vars(*this);
    for (size_t i = 0; i < g_numeric_var_names.size(); ++i) {
    	cout << "  #" << g_variable_domain.size()+i << " [" << g_numeric_var_names[i] << "] -> "
    			<< numeric_vals[i] << endl;
    }
}

std::vector<ap_float> GlobalState::get_numeric_vars() const {
	return registry->get_numeric_vars(*this);
}

std::string GlobalState::dump_plan_vis_log() const {
	stringstream outstream;
    for (size_t i = 0; i < g_variable_domain.size(); ++i)
        outstream << "{\"" << i << "\":"
             << (*this)[i] << "},";
    vector<ap_float> numeric_vals = registry->get_numeric_vars(*this);
    for (size_t i = 0; i < g_numeric_var_names.size(); ++i) {
    	outstream << " {\"" <<  g_variable_domain.size() + i  << "\":"
    			<< numeric_vals[i] << "},";
    }
    string returnstring = outstream.str();
    returnstring.pop_back();
    return returnstring;
}

std::string GlobalState::get_numeric_state_vals_string() const {
	stringstream outstream;
  	for (size_t i = 0; i < g_numeric_var_names.size(); ++i)
    	if (g_numeric_var_types[i] == regular) {
    		outstream << fixed << g_numeric_var_names[i] << "=" << registry->get_numeric_vars(*this)[i] << ";"; }
    string returnstring = outstream.str();
    if(returnstring.length() > 0)
    	returnstring.pop_back();
    return returnstring;
}

std::string GlobalState::dump_plan_vis_log(const GlobalState& parent) const {
	stringstream outstream;
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
    	if((*this)[i] != parent[i])
    		outstream << "{\"" << i << "\":"
			<< (*this)[i] << "},";
    }
    vector<ap_float> numeric_vals = registry->get_numeric_vars(*this);
    for (size_t i = 0; i < g_numeric_var_names.size(); ++i) {
    	if(numeric_vals[i] != parent.get_numeric_vars()[i])
    	outstream << "{\"" <<  g_variable_domain.size() + i  << "\":"
    			<< numeric_vals[i] << "},";
    }
    string returnstring = outstream.str();
    returnstring.pop_back();
    return returnstring;
}
