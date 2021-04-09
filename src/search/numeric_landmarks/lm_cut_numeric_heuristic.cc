
#include "lm_cut_numeric_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"
#include "../task_tools.h"

#include "../utils/memory.h"

#include <iostream>

using namespace std;

namespace lm_cut_numeric_heuristic {
    // construction and destruction
    LandmarkCutNumericHeuristic::LandmarkCutNumericHeuristic(const Options &opts)
    : Heuristic(opts),
      landmark_generator(nullptr),
      ceiling_less_than_one(opts.get<bool>("ceiling_less_than_one")),
      ignore_numeric(opts.get<bool>("ignore_numeric")),
      use_random_pcf(opts.get<bool>("random_pcf")),
      use_irmax(opts.get<bool>("irmax")) {
    }
    
    LandmarkCutNumericHeuristic::~LandmarkCutNumericHeuristic() {
    }
    
    // initialization
    void LandmarkCutNumericHeuristic::initialize() {
        cout << "Initializing landmark cut heuristic..." << endl;
        // TODO we don't need a pointer if we initialize in the constructor.
        landmark_generator = utils::make_unique_ptr<numeric_lm_cut_heuristic::LandmarkCutLandmarks>(
            task_proxy, ceiling_less_than_one, ignore_numeric, use_random_pcf, use_irmax);
    }
    
    ap_float LandmarkCutNumericHeuristic::compute_heuristic(const GlobalState &global_state) {
        State state = convert_global_state(global_state);
        return compute_heuristic(state);
    }
    
    ap_float LandmarkCutNumericHeuristic::compute_heuristic(const State &state) {
        ap_float total_cost = 0;
        bool dead_end = landmark_generator->compute_landmarks(
                                                              state,
                                                              [&total_cost](ap_float cut_cost) {total_cost += cut_cost; },
                                                              nullptr);
        if (dead_end)
            return DEAD_END;
        return total_cost;
    }

    static Heuristic *_parse(OptionParser &parser) {
        parser.document_synopsis("Landmark-cut heuristic", "");
        parser.document_language_support("action costs", "supported");
        parser.document_language_support("conditional effects", "not supported");
        parser.document_language_support("axioms", "not supported");
        parser.document_property("admissible", "yes");
        parser.document_property("consistent", "no");
        parser.document_property("safe", "yes");
        parser.document_property("preferred operators", "no");

        parser.add_option<bool>("ceiling_less_than_one", "use 1 instead of m_a when m_a < 1", "false");
        parser.add_option<bool>("ignore_numeric", "ignore numeric conditions", "false");
        parser.add_option<bool>("random_pcf", "use randomized precondition choice function", "false");
        parser.add_option<bool>("irmax", "use repetition relaxation", "false");
        
        Heuristic::add_options_to_parser(parser);
        Options opts = parser.parse();
        if (parser.dry_run())
            return nullptr;
        else
            return new LandmarkCutNumericHeuristic(opts);
    }
    
    static Plugin<Heuristic> _plugin("lmcutnumeric", _parse);
}
