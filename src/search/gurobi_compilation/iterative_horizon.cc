#include "iterative_horizon.h"

#include <cmath>

#include "../evaluation_context.h"
#include "../evaluators/sum_evaluator.h"
#include "../evaluators/weighted_evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../search_engines/search_common.h"
#include "ip_compilation.h"

using namespace std;
using namespace gurobi_ip_compilation;

// TODO: put default evaluator

GurobiIterativeHorizon::GurobiIterativeHorizon(const Options &opts)
    : SearchEngine(opts),
      initial_h(-1),
      current_t(1),
      last_iteration(false),
      model(new GurobiIPCompilation(opts, get_task_from_options(opts))),
      h_evaluator(opts.get<ScalarEvaluator *>("eval")) {
  cout << "ip compilation" << endl;
}

GurobiIterativeHorizon::~GurobiIterativeHorizon() { delete h_evaluator; }

void GurobiIterativeHorizon::initialize() {
  cout << "initialize" << endl;
  GlobalState initial_state = g_initial_state();
  EvaluationContext state(initial_state);
  EvaluationResult result = h_evaluator->compute_result(state);

  if (result.is_infinite()) {
    cout << "Initial state is a dead end." << endl;
  } else {
    initial_h = result.get_h_value();
    current_t = std::ceil(initial_h) + 1;
    cout << "initial time horizon " << current_t << endl;
  }
  model->initialize(current_t);
}

SearchStatus GurobiIterativeHorizon::step() {
  if (initial_h == -1) return FAILED;

  if (last_iteration) model->add_sequence_constraint();

  std::cout << "horizon = " << current_t << std::endl;
  double plan_cost = model->compute_plan();

  if (plan_cost >= 0) {
    double min_action_cost = model->get_min_action_cost();
    double optimal_cost_bound = current_t * min_action_cost;
    if (plan_cost == initial_h || plan_cost <= optimal_cost_bound ||
        last_iteration) {
      set_plan(model->extract_plan());
      return SOLVED;
    }
    current_t = std::ceil(plan_cost / min_action_cost);
    last_iteration = true;
  } else {
    ++current_t;
  }
  model->update(current_t);

  return IN_PROGRESS;
}

static SearchEngine *_parse(OptionParser &parser) {
  parser.document_synopsis("Gurobi ip compilation", "");
  gurobi_ip_compilation::add_model_option_to_parser(parser);
  parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
  parser.add_option<shared_ptr<AbstractTask>>(
      "transform",
      "Optional task transformation for the heuristic. "
      "Currently only adapt_costs is available.",
      OptionParser::NONE);
  parser.add_option<int>("threads", "Number of threads used by Gurobi", "1");
  parser.add_option<bool>("lazy_constraints", "Whether to add lazy constraints",
                          "false");
  SearchEngine::add_options_to_parser(parser);
  Options opts = parser.parse();

  if (parser.dry_run())
    return nullptr;
  else
    return new GurobiIterativeHorizon(opts);
}

static Plugin<SearchEngine> _plugin("gurobi_ip_compilation", _parse);

void gurobi_ip_compilation::add_model_option_to_parser(OptionParser &parser) {
  parser.document_note(
      "Note",
      "o utse an IP compilation, you must build the planner with LP support. "
      "See LPBuildInstructions.");
  vector<string> ip_models;
  vector<string> ip_models_doc;
  ip_models.push_back("SAS");
  ip_models_doc.push_back("SAS+, SAS+ state-change compilation");
  parser.add_list_option<shared_ptr<GurobiIPConstraintGenerator>>(
      "gurobi_ipmodel",
      "methods that generate constraints over operator counting variables");
}
