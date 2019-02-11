
#ifdef STATE_SPACE_H

#include "option_parser.h"
//#include "plugin.h"

#include "globals.h"
#include "evaluation_context.h"
#include "evaluation_result.h"

#include "utilities.h"

template<typename State, typename SearchSpace>
StateSpaceLookup<State, SearchSpace>::StateSpaceLookup(const Options &opts)
    : state_registry(new StateRegistry(g_initial_state_data)),
      _sp(NULL),
      is_budget_limited(g_budget >= 0),
      engine(NULL),
      heuristics(opts.get_list<ScalarEvaluator *>("prune")),
      _stop(heuristics.size())
{
    for (size_t i = 0; i < heuristics.size(); i++) {
        heuristics[i]->get_involved_heuristics(involved_heuristics);
    }
    if (opts.contains("engine")) {
        engine = dynamic_cast<DeterministicSearchEngine*>(opts.get<SearchEngine*>("engine"));
        if (engine == NULL) {
            cerr << "Expected DeterministicSearchEngine" << endl;
            exit_with(EXIT_INPUT_ERROR);
        }
    }
}

template<typename State, typename SearchSpace>
inline bool StateSpaceLookup<State, SearchSpace>::is_dead_end(
    const State &state, float &max)
{
    max = 0;
    GlobalState gstate = global_state(state.get_global_state_id());
    EvaluationContext ctxt(gstate);
    for (size_t i = 0; i < _stop; i++) {
        if (ctxt.is_heuristic_infinite(heuristics[i]) ||
            (is_budget_limited
             && ctxt.get_heuristic_value(heuristics[i]) > state.get_budget())) {
            max = EvaluationResult::INFINITE;
            return true;
        }
        if (ctxt.get_heuristic_value(heuristics[i]) > max) {
            max = ctxt.get_heuristic_value(heuristics[i]);
        }
    }
    return false;
}

template<typename State, typename SearchSpace>
inline float StateSpaceLookup<State, SearchSpace>::compute_goal_estimation(
    ScalarEvaluator &h, const State &state, bool &is_dead_end) const
{
    GlobalState gstate = global_state(state.get_global_state_id());
    EvaluationContext ctxt(gstate);
    is_dead_end = ctxt.is_heuristic_infinite(&h);
    return ctxt.get_heuristic_value(&h);
}

template<typename State, typename SearchSpace>
inline float StateSpaceLookup<State, SearchSpace>::compute_goal_estimation(
    ScalarEvaluator &h,
    const State &state,
    bool &is_dead_end,
    std::set<const GlobalOperator *> &preferred_operators) const
{
    GlobalState gstate = global_state(state.get_global_state_id());
    EvaluationContext ctxt(gstate);
    is_dead_end = ctxt.is_heuristic_infinite(&h);
    if (!is_dead_end) {
        const std::vector<const GlobalOperator *> &pref =
            ctxt.get_preferred_operators(&h);
        preferred_operators.insert(pref.begin(), pref.end());
    }
    return ctxt.get_heuristic_value(&h);
}

namespace state_space_lookup
{
static void copy_options(const Options &from, Options &to)
{
    to.set<std::vector<ScalarEvaluator *> >("prune", from.get_list<ScalarEvaluator *>("prune"));
    if (from.contains("engine")) {
        to.set<SearchEngine*>("engine", from.get<SearchEngine*>("engine"));
    }
}
static void add_options_to_parser(OptionParser &parser)
{
    parser.add_list_option<ScalarEvaluator *>("prune", "", "[]");
    parser.add_option<SearchEngine*>("engine", "", OptionParser::NONE);
}
}


#endif

