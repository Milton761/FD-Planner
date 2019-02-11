
#ifdef VALUEITERATION_H

#include "value_iteration.h"

#include "option_parser.h"
#include "plugin.h"

#include "globals.h"
#include "prob_search_space.h"
#include "successor_generator.h"
#include "global_operator.h"
#include "task_proxy.h"

#include "timer.h"

#include <list>
#include <limits>

using namespace std;

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::ValueIteration(
    const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, Type, SSearchSpace, StateSpace>
    (opts)
{
    epsilon = opts.get<float>("epsilon");

    if (Type == OptimizationCriterion::MAXP) {
        this->h_giveup = 0;
        this->h_default = 0;
        this->h_goal = 1;
    } else {
        this->h_giveup = g_giveup;
        this->h_default = g_giveup;
        this->h_goal = 0;
    }
}

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
void ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::initialize()
{
    ProbSearchEngine<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::initialize();
    Node init = this->search_space.get_prob_state(
                    this->state_space.get_initial_state(),
                    g_budget);
    this->initialize_value(init);
    init.set_open();
    _initial_state = init.get_prob_state_id();
}

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
SearchStatus
ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::step()
{
    cout << "Starting to construct annotated state space ..." << endl;
    Timer construction_t;
    construction_t.resume();

    // generate entire search space
    list<size_t> open;
    open.push_back(_initial_state);
    typename ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::template
    OpenListCallback<list<size_t> >
    callback(open);
    while (!open.empty()) {
        size_t sid = open.front();
        open.pop_front();
        Node p_state = this->search_space.get_prob_state(sid);
        this->generate_all_successors(p_state, callback);
    }

    construction_t.stop();

    cout << "State space has been constructed in " << construction_t << ". |S| = "
         << this->search_space.size() << endl;

    float error = numeric_limits<float>::max();
    size_t num_iterations = 0;
    size_t num_updates = 0;
    pair<float, int> update;
    while (error > epsilon) {
        num_iterations++;
        error = 0;
        for (int i = this->search_space.size() - 1; i >= 0; i--) {
            Node state = this->search_space.get_prob_state(i);
            if (state.is_goal() || state.is_dead_end()) {
                continue;
            }
            num_updates++;
            this->search_space.qval(state, update);
            float tmp = fabs(update.first - state.get_v_current());
            if (tmp > error) {
                error = tmp;
            }
            state.set_v_current(update.first, update.second);
        }
        this->report_progress(this->search_space, 0);
    }

    cout << "Converged after " << num_iterations << " iterations." << endl;
    cout << "Number of Belman-Updates: " << num_updates << endl;
    cout << "Prob search space contains " << this->search_space.size() <<
         " states ("
         << g_state_registry->size() << ")" << endl;
    print_probability(this->search_space.get_prob_state(
                          _initial_state).get_v_current());

    return SOLVED;
}

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
void ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::dump_states()
{
    //for (size_t i = 0; i < this->search_space.size(); i++) {
    //    cout << "[" << i << "]: ";
    //    Node p_state = this->search_space.get_prob_state(i);
    //    GlobalState state = p_state.get_global_state();
    //    cout << "{" <<  p_state.get_budget() << "}";
    //    for (size_t var = 0; var < g_variable_domain.size(); var++) {
    //        cout << ", ";
    //        cout << g_fact_names[var][state[var]];
    //    }
    //    cout << endl;
    //}
}

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
void ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::dump_table()
{
    //for (size_t i = 0; i < this->search_space.size(); i++) {
    //    cout << "[" << i << "]: " << this->search_space.get_prob_state(
    //             i).get_v_current() << endl;
    //}
}

template<int Type>
void VIFactory<Type>::add_options_to_parser(
    OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);
    parser.add_option<float>("epsilon", "", "0");
}

//static SearchEngine *_parse(OptionParser &parser)
//{
//    ValueIteration<NodeInfo, NODE, Type, SSearchSpace, StateSpace>::add_options_to_parser(parser);
//    Options opts = parser.parse();
//    SearchEngine *engine = NULL;
//    if (!parser.dry_run()) {
//        return new ValueIteration(opts);
//    }
//    return engine;
//}
//
//static Plugin<SearchEngine> _plugin("vi", _parse);

#endif

