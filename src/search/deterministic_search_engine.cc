
#include "deterministic_search_engine.h"

#include "countdown_timer.h"
#include "evaluation_context.h"
#include "globals.h"
#include "operator_cost.h"
#include "option_parser.h"

#include "utilities.h"

#include <cassert>
#include <iostream>
#include <limits>

using namespace std;

DeterministicSearchEngine::DeterministicSearchEngine(const Options &opts)
    : SearchEngine(opts), ScalarEvaluator(),
      cost_type(OperatorCost(opts.get_enum("cost_type"))),
      replanning(opts.get<bool>("replan")),
      search_space(new SearchSpace()),
      search_progress(new SearchProgress),
      statistics(new SearchStatistics)
{
    if (opts.get<int>("bound") < 0) {
        cerr << "error: negative cost bound " << opts.get<int>("bound") << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
    bound = opts.get<int>("bound");
    upper_bound = -1;
}

DeterministicSearchEngine::~DeterministicSearchEngine()
{
    if (search_space != NULL) {
        delete(search_space);
        search_space = NULL;
    }
    if (search_progress != NULL) {
        delete(search_progress);
        search_progress = NULL;
    }
    if (statistics != NULL) {
        delete(statistics);
        statistics = NULL;
    }
}

void DeterministicSearchEngine::reset()
{
    cerr << "This search engine is currently unsupported!" << endl;
    exit_with(EXIT_UNSUPPORTED);
}

const DeterministicSearchEngine::Plan &DeterministicSearchEngine::get_plan()
const
{
    assert(solution_found);
    return plan;
}

void DeterministicSearchEngine::set_plan(const Plan &p)
{
    solution_found = true;
    plan = p;
}

EvaluationResult DeterministicSearchEngine::compute_result(
    EvaluationContext &ctxt)
{
    update_initial_state(ctxt.get_state());
    if (search_space != NULL) {
        delete(search_space);
    }
    search_space = new SearchSpace();
    if (search_progress != NULL) {
        delete(search_progress);
    }
    search_progress = new SearchProgress();
    if (statistics != NULL) {
        delete(statistics);
    }
    statistics = new SearchStatistics;

    status = IN_PROGRESS;
    solution_found = false;
    plan.clear();
    this->reset();
    EvaluationResult res;
    basic_streambuf<char, char_traits<char> > *buf = cout.rdbuf(NULL);
    this->search();
    cout.rdbuf(buf);
    if (!solution_found) {
        res.set_h_value(EvaluationResult::INFINITE);
    } else {
        float h = 0;
        for (size_t i = 0; i < plan.size(); i++) {
            h += get_adjusted_cost(*plan[i]);
        }
        res.set_h_value(h < 0 ? 0 : h);
        res.set_preferred_operators(plan);
    }
    delete(search_space);
    search_space = NULL;
    delete(search_progress);
    search_progress = NULL;
    delete(statistics);
    statistics = NULL;
    delete(g_state_registry);
    g_state_registry = NULL;
    return res;
}

bool DeterministicSearchEngine::compute_plan(const GlobalState &state, vector<int> &res) {
    update_initial_state(state);
    if (search_space != NULL) {
        delete(search_space);
    }
    search_space = new SearchSpace();
    if (search_progress != NULL) {
        delete(search_progress);
    }
    search_progress = new SearchProgress();
    if (statistics != NULL) {
        delete(statistics);
    }
    statistics = new SearchStatistics;

    status = IN_PROGRESS;
    solution_found = false;
    this->plan.clear();
    this->reset();
    basic_streambuf<char, char_traits<char> > *buf = cout.rdbuf(NULL);
    this->search();
    cout.rdbuf(buf);
    if (!solution_found) {
        return false;
    }
    res.resize(this->plan.size());
    for (size_t i = 0; i < this->plan.size(); i++) {
        res[i] = this->plan[i]->get_id();
    }

    delete(search_space);
    search_space = NULL;
    delete(search_progress);
    search_progress = NULL;
    delete(statistics);
    statistics = NULL;
    delete(g_state_registry);
    g_state_registry = NULL;

    return solution_found;
}

bool DeterministicSearchEngine::check_goal_and_set_plan(
    const GlobalState &state)
{
    if (test_goal(state)) {
        cout << "Solution found!" << endl;
        Plan plan;
        search_space->trace_path(state, plan);
        set_plan(plan);
        return true;
    }
    return false;
}

void DeterministicSearchEngine::save_plan_if_necessary() const
{
    if (found_solution()) {
        save_plan(get_plan());
    }
}

void DeterministicSearchEngine::add_options_to_parser(OptionParser &parser)
{
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<int>(
        "bound",
        "exclusive depth bound on g-values. Cutoffs are always performed according to "
        "the real cost, regardless of the cost_type parameter", "infinity");
    parser.add_option<bool>("replan", "", "true");
}

void print_initial_h_values(const EvaluationContext &eval_context)
{
    eval_context.get_cache().for_each_heuristic_value(
    [](const Heuristic * heur, const EvaluationResult & result) {
        cout << "Initial heuristic value for "
             << heur->get_description() << ": ";
        if (result.is_infinite()) {
            cout << "infinity";
        } else {
            cout << result.get_h_value();
        }
        cout << endl;
    }
    );
}

void DeterministicSearchEngine::search()
{
    initialize();
    CountdownTimer timer(max_time);
    while (status == IN_PROGRESS) {
        status = step();
        if (timer.is_expired()) {
            cout << "Time limit reached. Abort search." << endl;
            status = TIMEOUT;
            break;
        }
    }
    cout << "Actual search time: " << timer
         << " [t=" << g_timer << "]" << endl;
}


