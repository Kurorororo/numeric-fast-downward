#include "group.h"
#include "../globals.h"
#include <iostream>

#include <fstream>
#include <algorithm>

using namespace std;

//Gens Group::generators;
bool Group::safe_to_add_generators;
int Group::num_identity_generators;
int Group::stop_after_false_generated;
//Disabled
/*
bool Group::prune_operators;
std::vector<int> orb_ops;
int nof_orbits;
*/



Group::Group(const Options &opts) {
  stop_after_false_generated = opts.get<int>("stop_after_false_generated");
  //Disabled
  /*
  prune_operators = opts.get<bool>("prune_operators");
  */
}

void Group::initialize() {
  safe_to_add_generators = true;
  num_identity_generators = 0;

  //Disabled
  /*
  for (int i = 0; i < g_operators.size(); i++){
    orb_ops.push_back(i);
  }

  nof_orbits = g_operators.size();
  */
}


/*
 * Delete all dynamically allocated vectors
 */
Group::~Group(){
  //Empty - all vectors expected to be deleted by default destructor of vector
  //	sub_groups.erase(sub_groups.begin(),sub_groups.end());
}


void Group::free_memory() {
    // Removing the permutations
    sub_groups.clear();
    init_group.clear();
    generators.clear();
    all_elements.clear();

}


/**
 * Add new permutation to the list of permutations
 * The function will be called from bliss
 */
void Group::add_permutation(void* param, unsigned int, const unsigned int * full_perm){
  if (!safe_to_add_generators) {
    cout << "Not safe to add permutations at this point!" << endl;
    exit(1);
  }

  Permutation perm(full_perm);
#ifdef DEBUGMODE
  cout<<"Did we get good permutation? --- ";
#endif
  //Only if we have non-identity permutation we need to save it into the list of generators
  if(!perm.identity()){
#ifdef DEBUGMODE
    cout<<"Yes! ";
#endif
    ((Group*) param)->add_generator(perm);
    //Disabled
    /*
    if (prune_operators) {
      ((Group*) param)->set_orbit_division(full_perm);
    }
    */
  } else {
    num_identity_generators++;
#ifdef DEBUGMODE
    cout<<"No! Identity generator number " << num_identity_generators << ". Until now found " << generators.size() << " generators.";
#endif
    if (num_identity_generators > stop_after_false_generated) {
      cout << endl << "Problems with generating symmetry group! Too many false generators." << endl;
      cout<<"Number of generators: 0"<<endl;
      exit(1);
    }
  }
#ifdef DEBUGMODE
  cout<<endl;
#endif

}


void Group::add_generator(Permutation gen) {
  if (!safe_to_add_generators) {
    cout << "Not safe to add permutations at this point!" << endl;
    exit(1);
  }

  generators.push_back(gen);
  init_group.push_back(get_num_generators()-1);
#ifdef DEBUGMODE
  cout << "Added generator number " << get_num_generators();
#endif

}

int Group::get_num_generators() const {
  return generators.size();
}

void Group::default_direct_product(){
  safe_to_add_generators = false;  // From this point on it is not safe to add generators
  dump_generators();

//	cout<<"Number of generators: "<<get_num_generators()<<endl;

  Trace subgroup;
  while(init_group.size() > 0) {
    subgroup.push_back(init_group[0]);
    init_group.erase(init_group.begin());
  }
  sub_groups.push_back(subgroup);
}


const Permutation& Group::get_permutation(int index) const {
  return generators[index];
}

void Group::dump_generators() const {
//	cout<<"Number of generators: "<<get_num_generators()<<endl;
  if (generators.size() == 0)
    return;

  cout << "Extra group info:" << endl;
  cout << "Permutation length: " << Permutation::length << endl;
}

void Group::dump_subgroups() const {
  for (int i= 0; i < sub_groups.size(); i++){
    cout << "Subgroup " << i << endl;
    for (int j= 0; j < sub_groups[i].size(); j++){
      cout << sub_groups[i][j] << " " ;
    }
    cout << endl;
  }

}

////////////////////////////////////////////////////////////////////////////////////////////
void Group::get_canonical_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                                std::vector<int> &canonical_values, std::vector<ap_float> &canonical_num_values) const {

  /*
   * Disabled
  if (g_generate_whole_group)
      return get_exact_canonical(state);
  */
  calculate_canonical_state(values, num_values, canonical_values, canonical_num_values);
}

void Group::calculate_canonical_state(const std::vector<int> &values, const std::vector<ap_float> &num_values,
                                      std::vector<int> &canonical_values, std::vector<ap_float> &canonical_num_values) const {
//	cout << "Calculating canonical state for state " << endl;
    canonical_values = values;
    canonical_num_values = num_values;
    for (int i = 0; i < sub_groups.size(); i++)
      calculate_canonical_state_subgroup(i, canonical_values, canonical_num_values);
}

void Group::calculate_canonical_state_subgroup(int ind, std::vector<int> &values, std::vector<ap_float> &num_values) const {
    // Going to the best successor, continue until local minima is reached
    // Warning: before running the method, the state is copied into the original_state.
    //          after finishing the run, the minimal state is in original_state
    int size = sub_groups[ind].size();
    if (size == 0)
        return;

//	cout << "Starting canonical state calculation:" << endl;
//	cout << "---------------------------------------------------------------------------------" << endl;
//	State(original_state).dump();
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i=0; i < size; i++) {
            if (generators[sub_groups[ind][i]].replace_if_less(values, num_values)) {
                changed =  true;
//				cout << "---------------------------------------------------------------------------------" << endl;
//				State(original_state).dump();
            }
        }
    }
//	cout << "==================================================================================" << endl;

}