#include "landmark_constraints.h"

#include "../globals.h"
#include "../numeric_landmarks/landmark_factory_scala.h"
#include "../numeric_operator_counting/numeric_helper.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../utils/markup.h"

using namespace std;
using namespace numeric_helper;
using namespace gurobi_ip_compilation;

void LandmarkConstraints::initialize(const int horizon,
                                     const std::shared_ptr<AbstractTask> task,
                                     std::shared_ptr<GRBModel> model,
                                     std::vector<std::vector<GRBVar>> &x) {
  factory = new landmarks::LandmarkFactoryScala(task);
  vector<set<int>> &landmarks_table = factory->get_landmarks_table();
  TaskProxy task_proxy(*task);
  State initial_state = task_proxy.get_initial_state();

  fact_landmarks = factory->compute_landmarks(initial_state);
  action_landmarks = factory->compute_action_landmarks(fact_landmarks);
  landmark_constraints(task, model, x, horizon, true);
}

void LandmarkConstraints::update(const int horizon,
                                 const std::shared_ptr<AbstractTask> task,
                                 std::shared_ptr<GRBModel> model,
                                 std::vector<std::vector<GRBVar>> &x) {
  landmark_constraints(task, model, x, horizon, false);
}

void LandmarkConstraints::landmark_constraints(
    const std::shared_ptr<AbstractTask> task, std::shared_ptr<GRBModel> model,
    std::vector<std::vector<GRBVar>> &x, int t_max, bool first) {
  cout << "adding constraint from landmark" << endl;
  for (size_t op_id : action_landmarks) {
    std::string name = "action_landmark_" + std::to_string(op_id);
    if (!first) {
      GRBConstr constraint = model->getConstrByName(name);
      model->remove(constraint);
    }
    GRBLinExpr lhs;
    double coefficient = 1;
    for (int t = 0; t < t_max; ++t) lhs.addTerms(&coefficient, &x[t][op_id], 1);
    model->addConstr(lhs >= 1, name);
  }
}

static shared_ptr<GurobiIPConstraintGenerator> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "Landmark constraint",
      "For details, see" +
          utils::format_paper_reference(
              {"Chiara Piacentini", "Margarita P. Castro", "Andre A. Cire",
               "J. Chirstopher Beck"},
              "Compiling optimal numeric planning to mixed integer linear "
              "programming",
              "https://aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/view/17770",
              "Proceedings of the Twentyeighth International Conference on"
              " Planning and Scheduling (ICAPS 2018)",
              "383-387", "2018"));

  if (parser.dry_run()) return nullptr;
  return make_shared<LandmarkConstraints>();
}

static PluginShared<GurobiIPConstraintGenerator> _plugin("landmark", _parse);
