#include "numeric_helper.h"

#include <sstream>

#include "../axioms.h"
#include "../task_tools.h"

using namespace std;

namespace numeric_helper {

bool LinearNumericCondition::dominate(LinearNumericCondition &other) const {
  // if they are linearly dependent, than
  assert(coefficients.size() == other.coefficients.size());
  const double epsilon = 0.00001;
  double ratio = 0;
  for (size_t v = 0; v < coefficients.size(); ++v) {
    double rhs = coefficients[v];
    double lhs = other.coefficients[v];
    if (fabs(lhs) < epsilon) {
      if (abs(rhs) > epsilon) return false;
    } else {
      if (fabs(ratio) < epsilon) {
        ratio = rhs / lhs;
      }
      if (ratio < epsilon) return false;
      if (fabs(rhs / lhs - ratio) > epsilon) return false;
    }
  }
  if (fabs(constant) - fabs(ratio * other.constant) > -epsilon) return true;
  return false;
}

bool LinearNumericCondition::empty() const {
  // if they are linearly dependent, than
  // if (constant != 0) return false;
  for (size_t v = 0; v < coefficients.size(); ++v) {
    double rhs = coefficients[v];
    if (rhs != 0) return false;
  }
  return true;
}

std::ostream &operator<<(std::ostream &os, const LinearNumericCondition &lnc) {
  for (size_t num_id = 0; num_id < lnc.coefficients.size(); ++num_id) {
    os << " " << lnc.coefficients[num_id];
  }
  os << " " << lnc.constant;
  return os;
}

NumericVariable::NumericVariable(int id_, int id_at, double lb_, double ub_)
    : id_var(id_),
      id_abstract_task(id_at),
      lower_bound(lb_),
      upper_bound(ub_) {}

bool NumericTaskProxy::redundant_constraints = true;

NumericTaskProxy::NumericTaskProxy(const TaskProxy &task, bool additional,
                                   bool use_linear_effects) {
  bool numeric = true;
  if (numeric) build_numeric_variables(task);
  if (numeric) build_artificial_variables(task);
  if (numeric) build_numeric_conditions(task);
  if (numeric) build_numeric_goals(task);
  build_propositions(task);
  build_actions(task, use_linear_effects);
  if (additional) {
    build_mutex_actions(task);
  }

  if (numeric) calculates_bounds_numeric_variables();
  if (numeric) calculates_small_m_and_epsilons();
  if (additional) {
    if (numeric) calculates_dominance();
  }
}

void NumericTaskProxy::calculates_dominance() {
  int size = numeric_conditions.size();
  dominance_conditions.assign(size, vector<bool>(size, false));
  for (int i = 0; i < size; i++) {
    LinearNumericCondition &lnc_i = numeric_conditions[i];
    for (int j = i; j < size; j++) {
      LinearNumericCondition &lnc_j = numeric_conditions[j];
      if (lnc_i.dominate(lnc_j)) dominance_conditions[i][j] = true;
      if (lnc_j.dominate(lnc_i)) dominance_conditions[j][i] = true;
    }
  }
}

void NumericTaskProxy::build_numeric_variables(const TaskProxy &task_proxy) {
  // variables initialization
  // double infinity = 9999999;
  // TODO add check
  NumericVariablesProxy num_variables = task_proxy.get_numeric_variables();
  id_numeric_variable_inv.assign(num_variables.size(), -1);
  for (size_t num_id = 0; num_id < num_variables.size(); ++num_id) {
    if (num_variables[num_id].get_var_type() == regular) {
      NumericVariable nv(numeric_variables.size(), num_id,
                         num_variables[num_id].get_initial_state_value(),
                         num_variables[num_id].get_initial_state_value());
      id_numeric_variable_inv[num_id] = numeric_variables.size();
      numeric_variables.push_back(nv);
    }
  }
  n_numeric_variables = numeric_variables.size();
}

void NumericTaskProxy::build_artificial_variables(const TaskProxy &task_proxy) {
  // variables initialization
  NumericVariablesProxy num_variables = task_proxy.get_numeric_variables();
  AssignmentAxiomsProxy assignment_axioms = task_proxy.get_assignment_axioms();
  artificial_variables.assign(num_variables.size(),
                              LinearNumericCondition(n_numeric_variables));
  for (size_t num_id = 0; num_id < num_variables.size(); ++num_id) {
    // artificial_variables[num_id].coefficients.assign(n_numeric_variables,0);
    if (num_variables[num_id].get_var_type() == regular) {
      artificial_variables[num_id]
          .coefficients[id_numeric_variable_inv[num_id]] = 1;
      // cout << num_id << " regular " << artificial_variables[num_id] << " " <<
      // num_variables[num_id].get_name() << endl;
    } else if (num_variables[num_id].get_var_type()) {
      artificial_variables[num_id].constant =
          num_variables[num_id].get_initial_state_value();
      // cout << num_id << " constant : " <<
      // num_variables[num_id].get_initial_state_value() << " " <<
      // artificial_variables[num_id] << " " << num_variables[num_id].get_name()
      // << endl;
    }
  }

  // populate artificial variables using ass axioms
  // initialize artificial variables
  for (size_t ax_id = 0; ax_id < assignment_axioms.size(); ++ax_id) {
    int affected_variable =
        assignment_axioms[ax_id].get_assignment_variable().get_id();
    int lhs = assignment_axioms[ax_id].get_left_variable().get_id();
    int rhs = assignment_axioms[ax_id].get_right_variable().get_id();

    switch (assignment_axioms[ax_id].get_arithmetic_operator_type()) {
      case sum:
        for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
          artificial_variables[affected_variable].coefficients[num_id] =
              artificial_variables[lhs].coefficients[num_id] +
              artificial_variables[rhs].coefficients[num_id];
        }
        artificial_variables[affected_variable].constant =
            artificial_variables[lhs].constant +
            artificial_variables[rhs].constant;
        break;
      case diff:
        for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
          artificial_variables[affected_variable].coefficients[num_id] =
              artificial_variables[lhs].coefficients[num_id] -
              artificial_variables[rhs].coefficients[num_id];
        }
        artificial_variables[affected_variable].constant =
            artificial_variables[lhs].constant -
            artificial_variables[rhs].constant;
        break;
      case mult:
        assert((num_variables[lhs].get_var_type() != constant) ||
               (num_variables[rhs].get_var_type() != constant));
        if (num_variables[lhs].get_var_type() == constant) {
          for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
            artificial_variables[affected_variable].coefficients[num_id] =
                artificial_variables[lhs].constant *
                artificial_variables[rhs].coefficients[num_id];
          }
          artificial_variables[affected_variable].constant =
              artificial_variables[lhs].constant *
              artificial_variables[rhs].constant;
        } else {
          for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
            artificial_variables[affected_variable].coefficients[num_id] =
                artificial_variables[lhs].coefficients[num_id] *
                artificial_variables[rhs].constant;
          }
          artificial_variables[affected_variable].constant =
              artificial_variables[lhs].constant *
              artificial_variables[rhs].constant;
        }
        break;
      case divi:
        assert(num_variables[rhs].get_var_type() != constant);
        for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
          artificial_variables[affected_variable].coefficients[num_id] =
              artificial_variables[lhs].coefficients[num_id] /
              artificial_variables[rhs].constant;
        }
        artificial_variables[affected_variable].constant =
            artificial_variables[lhs].constant /
            artificial_variables[rhs].constant;
        break;
      default:
        cout << "Error: No assignment operators are allowed here." << endl;
        assert(false);
        break;
    }
  }
}

void NumericTaskProxy::build_numeric_conditions(const TaskProxy &task_proxy) {
  fact_to_axiom_map.assign(task_proxy.get_variables().size(), -1);

  // initialize numeric conditions
  ComparisonAxiomsProxy axioms = task_proxy.get_comparison_axioms();
  n_conditions = 0;
  numeric_conditions_id.resize(axioms.size());
  for (size_t num_id = 0; num_id < axioms.size(); ++num_id) {
    ComparisonAxiomProxy ax = axioms[num_id];
    int left = ax.get_left_variable().get_id();
    int right = ax.get_right_variable().get_id();
    comp_operator comp = ax.get_comparison_operator_type();
    switch (comp) {  //    lt = 0, le = 1, eq = 2, ge = 3, gt = 4, ue = 5
      case lt: {
        LinearNumericCondition lnc =
            artificial_variables[right] - artificial_variables[left];
        lnc.is_strictly_greater = true;
        numeric_conditions.push_back(lnc);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        break;
      }
      case le: {
        LinearNumericCondition lnc =
            artificial_variables[right] - artificial_variables[left];
        numeric_conditions.push_back(lnc);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        break;
      }
      case eq: {
        numeric_conditions.push_back(artificial_variables[left] -
                                     artificial_variables[right]);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        numeric_conditions.push_back(artificial_variables[right] -
                                     artificial_variables[left]);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        break;
      }
      case ge: {
        LinearNumericCondition lnc =
            artificial_variables[left] - artificial_variables[right];
        numeric_conditions.push_back(lnc);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        break;
      }
      case gt: {
        LinearNumericCondition lnc =
            artificial_variables[left] - artificial_variables[right];
        lnc.is_strictly_greater = true;
        numeric_conditions.push_back(lnc);
        numeric_conditions_id[num_id].push_back(n_conditions);
        n_conditions++;
        break;
      }
      case ue: {
        cout << "Error: Don't know this comparator." << endl;
        assert(false);
        break;
      }
      default: {
        cout << "Error: No other comparators." << endl;
        assert(false);
        break;
      }
    }
    // cout << ax.get_true_fact().get_variable().get_id() << " " << ax.get_id()
    // << endl;
    // TODO add this
    fact_to_axiom_map[ax.get_true_fact().get_variable().get_id()] = ax.get_id();
  }
}

void NumericTaskProxy::build_propositions(const TaskProxy &task) {
  VariablesProxy vars = task.get_variables();
  propositions.resize(vars.size());
  add_effects.resize(vars.size());
  n_propositions = 0;
  int i_var = 0;
  for (VariableProxy var : vars) {
    propositions[i_var] = (vector<int>(var.get_domain_size()));
    add_effects[i_var] = vector<set<int>>(var.get_domain_size());
    // index_constraints.push_back(vector<int>(var.get_domain_size()));
    for (int i = 0; i < var.get_domain_size(); ++i) {
      propositions[var.get_id()][i] = n_propositions++;
      // TODO change not pushback
      propositions_inv.push_back({i_var, i});
      map_vars.push_back(i_var);
    }
    i_var++;
  }
  n_vars = i_var;
}

void NumericTaskProxy::build_actions(const TaskProxy &task,
                                     bool use_linear_effects) {
  OperatorsProxy ops = task.get_operators();
  VariablesProxy vars = task.get_variables();
  NumericVariablesProxy num_variables = task.get_numeric_variables();
  actions.assign(ops.size(), Action(n_numeric_variables));
  achievers.assign(n_propositions + n_conditions, set<int>());
  n_actions = ops.size();
  proposition_names.assign(n_propositions + n_conditions, "");
  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    const OperatorProxy &op = ops[op_id];
    vector<int> precondition(vars.size(), -1);
    actions[op_id].cost = op.get_cost();
    // cout << op.get_id() << " " << op.get_name() <<  " " <<
    // op.get_preconditions().size() <<  " " << op.get_effects().size() << " "
    // << op.get_ass_effects().size() << endl;
    for (FactProxy condition : op.get_preconditions()) {
      int pre_var_id = condition.get_variable().get_id();
      precondition[pre_var_id] = condition.get_value();
      // cout << "\tpre " << pre_var_id << " " << condition.get_value() <<
      // endl;//effect.get_name() << endl;
      // if precondition is a numeric precondition
      if (fact_to_axiom_map[pre_var_id] == -1) {
        actions[op_id].pre_list.insert(
            propositions[pre_var_id][condition.get_value()]);
      } else {
        // cout << op.get_id() << " " << op.get_name() << endl;
        actions[op_id].num_list.insert(fact_to_axiom_map[pre_var_id]);
        // for (auto num_id :
        // numeric_conditions_id[fact_to_axiom_map[pre_var_id]])
        //    cout << "\t" << numeric_conditions[num_id] << ", fact " <<
        //    fact_to_axiom_map[pre_var_id] << endl;
      }
    }

    if (redundant_constraints) {
      set<int> original_list = actions[op_id].num_list;
      for (size_t i = 0; i < original_list.size(); ++i) {
        std::set<int>::iterator it_i = original_list.begin();
        std::advance(it_i, i);
        list<int> list_i = numeric_conditions_id[*it_i];
        list<int>::iterator nc_it_i = list_i.begin();
        list<int>::iterator nc_end_i = list_i.end();
        for (; nc_it_i != nc_end_i; ++nc_it_i) {
          int x = *nc_it_i;  //*numeric_conditions_id[*it_i].begin();
          for (size_t j = i + 1; j < original_list.size(); ++j) {
            // add the two
            std::set<int>::iterator it_j = original_list.begin();
            std::advance(it_j, j);
            list<int> list_j = numeric_conditions_id[*it_j];
            list<int>::iterator nc_it_j = list_j.begin();
            list<int>::iterator nc_end_j = list_j.end();
            for (; nc_it_j != nc_end_j; ++nc_it_j) {
              int y = *nc_it_j;
              LinearNumericCondition redundant =
                  numeric_conditions[x] + numeric_conditions[y];
              // cout << numeric_conditions[x] << " + "  <<
              // numeric_conditions[y] << " = " << redundant << endl;
              if (redundant.empty()) continue;
              numeric_conditions.push_back(redundant);
              actions[op_id].num_list.insert(numeric_conditions_id.size());
              numeric_conditions_id.push_back(list<int>(1, n_conditions));
              fact_to_axiom_map.push_back(-2);
              n_conditions++;
              achievers.push_back(set<int>());
              proposition_names.push_back("");
            }
          }
        }
      }
    }

    for (EffectProxy effect_proxy : op.get_effects()) {
      FactProxy effect = effect_proxy.get_fact();
      int var = effect.get_variable().get_id();
      int pre = precondition[var];
      int post = effect.get_value();
      // cout << "\teff " << var << " " << post << endl;//effect.get_name() <<
      // endl;
      assert(post != -1);
      // assert(pre != post);
      add_effects[var][post].insert(op_id);
      achievers[propositions[var][post]].insert(op_id);
      proposition_names[propositions[var][post]] = effect.get_name();
      // cout << "--- " << effect.get_name() << endl;
      if (pre != -1) {
        actions[op_id].add_list.insert(propositions[var][post]);
        actions[op_id].del_list.insert(propositions[var][pre]);
      } else {
        actions[op_id].add_list.insert(propositions[var][post]);
      }
    }

    // add intersection
    set<int>::iterator pre_it = actions[op_id].pre_list.begin();
    set<int>::iterator pre_end = actions[op_id].pre_list.end();
    set<int>::iterator del_it = actions[op_id].del_list.begin();
    set<int>::iterator del_end = actions[op_id].del_list.end();
    set<int> intersect;
    set_intersection(pre_it, pre_end, del_it, del_end,
                     inserter(intersect, intersect.begin()));
    actions[op_id].pre_del_list = intersect;
    // cout << "intersections " << op_id << " " << intersect.size() << " " <<
    // actions[op_id].pre_list.size() << " " << actions[op_id].del_list.size()
    // << endl; numeric effects
    // actions[op_id].eff_list.assign(n_numeric_variables, 0);
    AssEffectsProxy effs = op.get_ass_effects();
    // cout << op.get_name() << " - " << effs.size() << endl;
    for (size_t eff_id = 0; eff_id < effs.size(); ++eff_id) {
      AssEffectProxy eff = effs[eff_id];
      // cout << "\t" << eff.op_index << " " << eff.eff_index << " " <<
      // eff.is_axiom << endl;
      int lhs = eff.get_assignment().get_affected_variable().get_id();
      int rhs = eff.get_assignment().get_assigned_variable().get_id();
      f_operator oper = eff.get_assignment().get_assigment_operator_type();

      // cout << "effect " << lhs << " " << rhs << " " << oper << endl;
      if (num_variables[lhs].get_var_type() == instrumentation) {
        continue;
      }
      int id_num = id_numeric_variable_inv[lhs];
      if (id_num == -1) {
        cout << "Error: variable not constant" << endl;
        assert(false);
      }
      LinearNumericCondition &av = artificial_variables[rhs];
      bool is_simple_effect = oper == increase || oper == decrease;
      if (use_linear_effects) {
        for (int var = 0; var < n_numeric_variables; ++var) {
          if (fabs(av.coefficients[var]) >= 0.0001) {
            is_simple_effect = false;
            break;
          }
        }
      }
      if (is_simple_effect) {
        switch (oper) {
          case (increase): {
            actions[op_id].eff_list[id_num] = av.constant;
            break;
          }
          case (decrease): {
            actions[op_id].eff_list[id_num] = -av.constant;
            break;
          }
          default: {
            cout << "non-simple numeric effect";
            assert(false);
            break;
          }
        }
      } else {
        ++actions[op_id].n_linear_eff;
        actions[op_id].linear_eff_lhs.push_back(id_num);
        std::vector<ap_float> coefficients(n_numeric_variables, 0.0);
        switch (oper) {
          case (assign): {
            for (int var = 0; var < n_numeric_variables; ++var) {
              coefficients[var] = av.coefficients[var];
            }
            actions[op_id].linear_eff_coefficeints.push_back(coefficients);
            actions[op_id].linear_eff_constants.push_back(av.constant);
            break;
          }
          case (increase): {
            for (int var = 0; var < n_numeric_variables; ++var) {
              coefficients[var] = av.coefficients[var];
              if (id_num == var) coefficients[var] += 1.0;
            }
            actions[op_id].linear_eff_coefficeints.push_back(coefficients);
            actions[op_id].linear_eff_constants.push_back(av.constant);
            break;
          }
          case (decrease): {
            for (int var = 0; var < n_numeric_variables; ++var) {
              coefficients[var] = -av.coefficients[var];
              if (id_num == var) coefficients[var] += 1.0;
            }
            actions[op_id].linear_eff_coefficeints.push_back(coefficients);
            actions[op_id].linear_eff_constants.push_back(-av.constant);
            break;
          }
          default: {
            cout << "non-linear numeric effect";
            assert(false);
            break;
          }
        }
      }
    }
  }
  generate_possible_achievers(task);
}

void NumericTaskProxy::build_mutex_actions(const TaskProxy &task) {
  // cout << "---"  << endl;
  OperatorsProxy ops = task.get_operators();
  VariablesProxy vars = task.get_variables();
  mutex_actions.assign(ops.size(), set<int>());
  for (size_t op_id = 0; op_id < ops.size(); ++op_id) {
    // two actions are mutex
    const OperatorProxy &op = ops[op_id];
    vector<int> precondition(vars.size(), -1);
    vector<int> postcondition(vars.size(), -1);
    for (FactProxy condition : op.get_preconditions()) {
      int pre_var_id = condition.get_variable().get_id();
      precondition[pre_var_id] = condition.get_value();
    }
    for (EffectProxy effect_proxy : op.get_effects()) {
      FactProxy effect = effect_proxy.get_fact();
      int var = effect.get_variable().get_id();
      postcondition[var] = effect.get_value();
    }
    //
    for (size_t op_a_id = 0; op_a_id < ops.size(); ++op_a_id) {
      if (op_id == op_a_id) continue;
      const OperatorProxy &op_a = ops[op_a_id];
      for (EffectProxy effect_proxy : op_a.get_effects()) {
        FactProxy effect = effect_proxy.get_fact();
        int var = effect.get_variable().get_id();
        int post = effect.get_value();
        // This is a SHAMEFUL HACK when you apply the tnf transformation
        if (op_a.get_name().find("forget") == 0 &&
            ops[op_id].get_name().find("forget") == 0)
          continue;
        if ((precondition[var] != post && precondition[var] != -1) ||
            (postcondition[var] != post)) {
          mutex_actions[op_id].insert(op_a_id);
          // if (op_a.get_name().find("forget")==0 &&
          // ops[op_id].get_name().find("forget")==0)
          //    cout << op_a.get_name() << " " << ops[op_id].get_name() << " are
          //    mutex" << endl;
        }
      }
    }
  }
}

void NumericTaskProxy::generate_possible_achievers(const TaskProxy &task) {
  size_t n_numeric_variables = get_n_numeric_variables();
  for (size_t nc_id = 0; nc_id < get_n_conditions(); ++nc_id) {
    for (size_t op_id = 0; op_id < task.get_operators().size(); ++op_id) {
      LinearNumericCondition &nc = get_condition(nc_id);
      double cumulative_effect = 0;
      for (size_t v = 0; v < n_numeric_variables; ++v) {
        cumulative_effect +=
            (nc.coefficients[v] * get_action_eff_list(op_id)[v]);
      }

      if (cumulative_effect > 0) {
        achievers[nc_id + n_propositions].insert(op_id);
        actions[op_id].possible_add_list.insert(nc_id + n_propositions);
      }
    }
  }
}

void NumericTaskProxy::build_numeric_goals(const TaskProxy &task) {
  // check other axioms
  numeric_goals.assign(task.get_goals().size(), *(new list<int>()));
  unordered_map<int, list<int>> axiom_table;
  {
    AxiomsProxy axioms = task.get_axioms();
    for (OperatorProxy ax : axioms) {
      for (FactProxy pre : ax.get_preconditions()) {
        // cout << "\tpre" << pre.get_name() << endl;
        if (ax.get_effects().size() > 1) {
          cout << "non-numerical axioms" << endl;
          assert(false);
        }
        if (ax.get_effects().size() == 0) continue;
        axiom_table[ax.get_effects()[0].get_fact().get_variable().get_id()]
            .push_back(pre.get_variable().get_id());
      }
    }
  }

  for (size_t g_id = 0; g_id < task.get_goals().size(); ++g_id) {
    FactProxy goal = task.get_goals()[g_id];
    int goal_id = goal.get_variable().get_id();
    if (axiom_table.find(goal_id) != axiom_table.end()) {
      for (int id_g : axiom_table[goal_id]) {
        for (int id_n_con : numeric_conditions_id[fact_to_axiom_map[id_g]]) {
          // check if it's not empty
          LinearNumericCondition &check_goal = numeric_conditions[id_n_con];
          if (check_goal.empty()) continue;

          // numeric_conditions_id[fact_to_axiom_map[axiom_table[goal_id]]])
          numeric_goals[g_id].push_back(id_n_con);
          fact_to_axiom_map[goal_id] = -2;  // this is a special number to
                                            // indicate that the goal is numeric
          // cout << "\t" << numeric_conditions[id_n_con] << endl;
        }
      }
    }
  }

  if (redundant_constraints) {
    std::vector<std::list<int>> original_list = numeric_goals;
    for (size_t t = 0; t < original_list.size(); ++t) {
      for (size_t i = 0; i < original_list[t].size(); ++i) {
        std::list<int>::iterator it_i = original_list[t].begin();
        std::advance(it_i, i);
        int x = *it_i;
        for (size_t j = i + 1; j < original_list[t].size(); ++j) {
          // add the two
          list<int>::iterator it_j = original_list[t].begin();
          std::advance(it_j, j);
          int y = *it_j;
          LinearNumericCondition redundant =
              numeric_conditions[x] + numeric_conditions[y];
          if (redundant.empty()) continue;
          //                        cout << "adding "  << redundant << " from "
          //                        << numeric_conditions[x] << " and " <<
          //                        numeric_conditions[y]  << " to goal" <<
          //                        endl; cout << "\tbefore " <<
          //                        numeric_conditions.size() << " " <<
          //                        numeric_conditions_id.size() << endl;
          numeric_conditions.push_back(redundant);
          numeric_goals[t].push_back(n_conditions);
          numeric_conditions_id.push_back(list<int>(n_conditions));
          fact_to_axiom_map.push_back(-2);
          n_conditions++;
          // achievers.push_back(set<int>());
          // proposition_names.push_back("");
          //                        cout << "\tafter " <<
          //                        numeric_conditions.size() << " " <<
          //                        numeric_conditions_id.size() << " " <<
          //                        n_conditions << endl;
        }
      }
    }
  }
}

// this is for numeric simple plans only
void NumericTaskProxy::calculates_bounds_numeric_variables() {
  // check if actions with effects on a real variable x : x + c have a
  // precondition - w*x > b, with c > 0, than upperbound is max(current state,
  // b+c)
  for (size_t num_id = 0; num_id < n_numeric_variables; ++num_id) {
    // get all actions
    // cout << "check variable " << num_id << endl;
    for (size_t op_id = 0; op_id < n_actions; ++op_id) {
      double eff = actions[op_id].eff_list[num_id];
      if (fabs(eff) < 0.0001) continue;  // action is not effecting the variable
      // cout << "\teffect on action " << op_id << " " << eff << endl;
      bool bound_for_this_action = false;
      double bound = 0;
      for (size_t c_id : actions[op_id].num_list) {
        // numeric preconditions
        for (int nc_id : numeric_conditions_id[c_id]) {
          // cout <<"\t\t" << nc_id << ":" << numeric_conditions[nc_id] << endl;
          double w = numeric_conditions[nc_id].coefficients[num_id];
          double k = numeric_conditions[nc_id].constant;
          // check if it's a condition of the form x + c : all other
          // coefficients are 0
          if (!numeric_conditions[nc_id].simple_condition(num_id)) continue;
          // we have a condition of the simple type, we calculate the
          // constraints
          if (w * eff > 0) continue;  //
          bound_for_this_action = true;
          bound = -k / w + eff;
          // cout <<"\t\t\twith bound " << bound << endl;
        }
      }
      // check if bound found
      if (bound_for_this_action) {
        // if eff > 0, than the bound is an upper bound, else is a lower bound
        if (eff > 0) {
          double previous_bound = numeric_variables[num_id].upper_bound;
          numeric_variables[num_id].upper_bound = max(bound, previous_bound);
        } else {
          double previous_bound = numeric_variables[num_id].lower_bound;
          numeric_variables[num_id].lower_bound = min(bound, previous_bound);
        }
      } else {
        // bound not found, set to infinity and interrupt cycle (cannot because
        // we might find a lower/upper bound)
        if (eff > 0) {
          numeric_variables[num_id].upper_bound = 9999999;
          // cout << "upper bound of " << num_id << " cannot be calculated" <<
          // endl;
        } else {
          numeric_variables[num_id].lower_bound = -9999999;
          // cout << "lower bound of " << num_id << " cannot be calculated" <<
          // endl;
        }
        // break;
      }
    }
    // cout << "numeric variable " << num_id << " [" <<
    // numeric_variables[num_id].lower_bound
    //<< "," << numeric_variables[num_id].upper_bound << "]" << endl;
  }
}

void NumericTaskProxy::calculates_small_m_and_epsilons() {
  small_m.assign(n_conditions, -9999999);
  epsilon.assign(n_conditions, 0);
  for (size_t i = 0; i < n_conditions; ++i) {
    LinearNumericCondition &lnc = get_condition(i);
    double l_b = lnc.constant;

    for (size_t n_id = 0; n_id < get_n_numeric_variables(); ++n_id) {
      if (lnc.coefficients[n_id] > 0)
        l_b += lnc.coefficients[n_id] * get_numeric_variable(n_id).lower_bound;
      else
        l_b += lnc.coefficients[n_id] * get_numeric_variable(n_id).upper_bound;
    }
    small_m[i] = l_b;
    if (!lnc.is_strictly_greater) continue;
    double min_epsilon = 9999999;
    double default_epsilon = 0.0001;
    for (size_t op_id = 0; op_id < get_n_actions(); ++op_id) {
      double local_epsilon = 0;
      for (size_t n_id = 0; n_id < get_n_numeric_variables(); ++n_id) {
        local_epsilon +=
            lnc.coefficients[n_id] * get_action_eff_list(op_id)[n_id];
      }
      if (fabs(local_epsilon) < min_epsilon && fabs(local_epsilon) > 0)
        min_epsilon = fabs(local_epsilon);
      for (int j = 0; j < get_action_n_linear_eff(op_id); ++j) {
        int lhs = get_action_linear_lhs(op_id)[j];
        if (fabs(lnc.coefficients[lhs]) > 0.0001) {
          min_epsilon = default_epsilon;
          break;
        }
      }
    }
    epsilon[i] = min_epsilon;
  }
}
}  // namespace numeric_helper
