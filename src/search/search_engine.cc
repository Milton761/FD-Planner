#include "search_engine.h"

#include "countdown_timer.h"
#include "evaluation_context.h"
#include "globals.h"
#include "operator_cost.h"
#include "option_parser.h"

#include "abstract_task.h"

#include <cassert>
#include <iostream>
#include <limits>

using namespace std;


SearchEngine::SearchEngine(SearchEngine *copy)
    : status(IN_PROGRESS),
      solution_found(false),
      atask(copy->atask),
      max_time(copy->max_time)
{
}

SearchEngine::SearchEngine(const Options &opts)
    : status(IN_PROGRESS),
      solution_found(false),
      atask(get_task_from_options(opts)),
      max_time(opts.get<double>("max_time"))
{
}

SearchEngine::~SearchEngine()
{
}

void SearchEngine::print_statistics() const
{
}

bool SearchEngine::found_solution() const
{
    return solution_found;
}

SearchStatus SearchEngine::get_status() const
{
    return status;
}

float SearchEngine::get_adjusted_cost(const GlobalOperator &op) const
{
    return atask->get_operator_cost(op.get_id(), op.is_axiom());
}

void SearchEngine::add_options_to_parser(OptionParser &parser)
{
    ::add_cost_type_option_to_parser(parser);
    parser.add_option<shared_ptr<AbstractTask> >("transform", "", OptionParser::NONE);
    parser.add_option<double>(
        "max_time",
        "maximum time in seconds the search is allowed to run for. The "
        "timeout is only checked after each complete search step "
        "(usually a node expansion), so the actual runtime can be arbitrarily "
        "longer. Therefore, this parameter should not be used for time-limiting "
        "experiments. Timed-out searches are treated as failed searches, "
        "just like incomplete search algorithms that exhaust their search space.",
        "infinity");
}

