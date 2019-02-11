
#ifdef PROB_SEARCH_ENGINE_H

#include "prob_search_engine.h"

#include "option_parser.h"

#include "prob_search_space.h"

#include "plugin.h"
#include "task_proxy.h"
#include "successor_generator.h"
#include "state_registry.h"

#include "globals.h"

#include "state_space_factory.h"

#include "countdown_timer.h"

#include <fstream>
#include <iostream>
#include <memory>

#include <unordered_set>
#include <deque>

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::ProbSearchEngine(
    ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking> *copy)
    : SearchEngine(copy),
      m_epsilon(copy->m_epsilon),
      value_initializer(copy->value_initializer),
      value_reporter(copy->value_reporter),
      progress_reporter(value_reporter, _t),
      state_space(copy->state_space),
      search_space(copy->search_space),
      m_sub_engine(false)
{
    solution_found = true;
    initialized = false;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::ProbSearchEngine(
    const Options &opts)
    : SearchEngine(opts),
    m_epsilon(opts.get<float>("epsilon")),
      value_initializer(*opts.get<ValueInitializer *>("initializer")),
      value_reporter(*opts.get<ValueReporter *>("reporter")),
      progress_reporter(value_reporter, _t),
      state_space(*opts.get<StateSpace *>("state_space")),
      search_space(*(new SearchSpace(state_space)))
{
    solution_found = true;
    initialized = false;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::initialize()
{
    status = IN_PROGRESS;
    if (!initialized) {
        if (!m_sub_engine) {
            this->print_options();
        }
        SearchEngine::initialize();
        state_space.initialize();
        // create initial state
        Node<NodeInfo> init = this->search_space.get_prob_state(this->state_space.get_initial_state(), g_budget);
        init.set_open();
        // initialize its value
        this->initialize_value(init);
        _initial_state_id = init.get_prob_state_id();
        initialized = true;
        this->report_progress(this->search_space, this->get_initial_state_id(), true);
    }
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
inline float ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::initialize_value(
    Node<NodeInfo> &state)
{
    float res = 0;
    if (this->search_space.is_goal(state)) {
        state.set_goal();
    } else if (this->state_space.is_dead_end(state, res)) {
        state.mark_as_dead_end();
    }

    if (state.is_dead_end()) {
        value_initializer.dead_end(state);
    } else if (state.is_goal()) {
        value_initializer.goal(state);
    } else {
        value_initializer(state);
    }

    return res;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::search()
{
    CountdownTimer timer(max_time);
    // run actual search:
    search_internal(&timer);

    // output some statistics
    this->report_progress(this->search_space, this->get_initial_state_id(), true);
    cout << "Actual search time: " << timer
         << " [t=" << g_timer << "]" << endl;
    cout << "Prob search space contains " << this->search_space.size() <<
         " states ("
         << this->state_space.size() << ")" << endl;
    cout << "Number of updates: " << this->search_space.get_updates() << endl;
    cout << "Final value function: [";
    value_reporter.report_final(get_initial_state());
    cout << "]" << endl;
    cout << "QVal-Tiebreaking triggered: " << this->search_space.ASD << " / "
        << this->search_space.DSA << std::endl;

    // check if policy should be extracted and stored
    if (g_store_policy) {
        std::ofstream f;
        f.open(g_plan_filename);
        extract_policy(f);
        f.close();
    }
}


template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::search_internal(CountdownTimer *t)
{
    // initialize data structures
    initialize();
    // if initial state is dead end, just return
    if (this->get_initial_state().is_dead_end()) {
        return;
    }
    // as long as the search has not found a solution
    // call step()
    while (status == IN_PROGRESS) {
        // in step(), do actual search
        status = step();
        if (t && t->is_expired()) {
            cout << "Time limit reached. Abort search." << endl;
            status = TIMEOUT;
            break;
        }
    }
}


template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
template<typename C1, typename C2>
bool ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::generate_all_successors(
    Node<NodeInfo> &p_state, C1 &c1, C2 &c2, bool update)
{
    int defaultop = -1;
    pair<unsigned, unsigned> outcome;
    typename StateSpace::generate_applicable_op_iterator
    op = state_space.generate_applicable_operators(p_state);
    unsigned i = 0;
    float h;
    while (!op.end()) {
        if (defaultop == -1) {
            defaultop = op.get_local_ref();
        }
        i++;
        typename StateSpace::generate_successor_iterator
        succ =
            state_space.generate_successors(p_state, op);
        while (!succ.end()) {
            Node<NodeInfo> p_succ = search_space.get_prob_state(succ.state());
            bool isnew = p_succ.add_parent(p_state.get_prob_state_id());
            if (p_succ.is_new()) {
                succ.get_outcome(outcome);
                p_succ.set_open();
                h = this->initialize_value(p_succ);
                c2(this->search_space, this->state_space, &p_state, p_succ,
                   &g_operators[g_prob_operators[outcome.first][outcome.second].second],
                   h);
            }
            c1(this->search_space, this->state_space, p_state, p_succ,
               &g_operators[g_prob_operators[outcome.first][outcome.second].second],
               isnew);
            succ++;
        }
        op++;
    }

    if (defaultop >= 0 && update) {
        std::pair<float, int> newv;
        this->search_space.qval_update(p_state, newv, m_tiebreaking, m_epsilon);
    }

    return defaultop != -1;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options() {
    printf("epsilon = %.8f\n", m_epsilon);
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
inline void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::report_progress(
    SearchSpace &search_space,
    size_t state, bool force_print)
{
    progress_reporter(search_space, state, force_print);
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::extract_policy(std::ostream &out)
{
    std::deque<std::pair<StateID, int> > open;
    std::unordered_set<std::pair<int, int> > closed;
    open.push_back(std::make_pair(g_initial_state().get_id(), g_budget));
    closed.insert(std::make_pair(open.front().first.hash(), g_budget));
    while (!open.empty()) {
        std::pair<StateID, int> x = open.front();
        open.pop_front();
        GlobalState gstate = g_state_registry->lookup_state(x.first);
        State state = search_space.get_prob_state(gstate,
                x.second);
        if (state.is_dead_end() || state.is_goal()) {
            continue;
        }
        assert(state.get_policy_operator_index() >= 0);
        int prob_op = state_space.get_global_operator(state, state.get_policy_operator_index());
        assert(prob_op >= 0 && prob_op <= (int) g_prob_operators.size());
        dump_prob_state(gstate, x.second, false, out);
        out << " -> " << g_prob_operator_name[prob_op] << std::endl;
        const std::vector<std::pair<float, int> > &outcomes = g_prob_operators[prob_op];
        int new_budget = compute_budget(x.second, g_prob_operator_cost[prob_op]);
        for (const std::pair<float, int> &outcome : outcomes) {
            GlobalState succ = g_state_registry->get_successor_state(gstate, g_operators[outcome.second]);
            if (closed.insert(make_pair(succ.get_id().hash(), new_budget)).second) {
                open.push_back(make_pair(succ.get_id(), new_budget));
            }
        }
    }
}

namespace prob_search_engine
{
void add_options_to_parser(
    OptionParser &parser)
{
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<state_space_factory::StateSpaceFactory *>("state_space", "",
            "factored");
    parser.add_option<float>("epsilon", "", "0");
}
}

#endif

