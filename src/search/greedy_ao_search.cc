
#ifdef AO_SEARCH_H

#include "greedy_ao_search.h"

#include "option_parser.h"
#include "plugin.h"

#include "globals.h"
#include "prob_search_space.h"
#include "successor_generator.h"
#include "global_operator.h"
#include "task_proxy.h"

#include "optimization_criteria.h"

#include "timer.h"

#include <list>
#include <limits>
#include <set>
#include <sstream>

#include <fstream>

using namespace std;

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename, typename, typename, typename> class ValueUpdater, template<typename, typename, typename> class OutcomeSelector, template<typename> class Callback, typename CallbackRe, typename TieBreaking, bool Deterministic>
AOSearch<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, ValueUpdater, OutcomeSelector, Callback, CallbackRe, TieBreaking, Deterministic>::AOSearch(
    const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
    (opts),
    num_iterations(0),
    callback(*opts.get<Callback<Node> *>("callback")),
    callback_re(*opts.get<CallbackRe*>("callbackre")),
    vupdater(this->search_space, this->state_space, opts.get<float>("epsilon")),
    oselector(*opts.get<OutcomeSelector<Node, SearchSpace, StateSpace> *>("oselector"))
{
    oselector.initialize(this->search_space, this->state_space);
    _t.stop();
    _t.reset();
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename, typename, typename, typename> class ValueUpdater, template<typename, typename, typename> class OutcomeSelector, template<typename> class Callback, typename CallbackRe, typename TieBreaking, bool Deterministic>
void
AOSearch<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, ValueUpdater, OutcomeSelector, Callback, CallbackRe, TieBreaking, Deterministic>::expand_node(
    NODE<NodeInfo> node)
{
    node.close();
    if (!this->generate_all_successors(node, callback_re, callback)) {
        node.mark_as_dead_end();
        this->value_initializer.dead_end(node);
    }
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename, typename, typename, typename> class ValueUpdater, template<typename, typename, typename> class OutcomeSelector, template<typename> class Callback, typename CallbackRe, typename TieBreaking, bool Deterministic>
SearchStatus
AOSearch<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, ValueUpdater, OutcomeSelector, Callback, CallbackRe, TieBreaking, Deterministic>::step()
{
    //cout << "Start expansion ..." << endl;
    int sid;
    Node initial_state = this->get_initial_state();
    //cout << "Initial state: " << initial_state.get_prob_state_id() << endl;
    //size_t IIII = 0;
    while (initial_state.is_marked()) {
        //cout << "Starting with expansion #" << num_iterations << endl;
        sid = this->get_initial_state_id();
        //cout << "Selecting state to expand" << endl;
        while (true) {
            Node node = this->search_space.get_prob_state(sid);
            if (node.is_opened()) {
                break;
            }
            sid = oselector(node);
            if (sid < 0) {
                //cerr << "there is no states with id " << sid << endl;
                // THIS SHOULD ONLY HAPPEN IN AO*|L
                return SOLVED;
            }

        }

        Node state = this->search_space.get_prob_state(sid);
        assert(!state.is_goal() && !state.is_dead_end());
        //cout << "Generating successors of " << sid << endl;
        expand_node(state);
        //cout << "Doing update starting at " << sid << endl;
        vupdater(state, this->search_space.size());

        //cout << "done with expansion step " << num_iterations << endl;

        this->report_progress(this->search_space, this->get_initial_state_id());

        num_iterations++;
    }

    //this->report_progress(this->search_space, this->get_initial_state_id(), true);

    return SOLVED;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename, typename, typename, typename> class ValueUpdater, template<typename, typename, typename> class OutcomeSelector, template<typename> class Callback, typename CallbackRe, typename TieBreaking, bool Deterministic>
void
AOSearch<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, ValueUpdater, OutcomeSelector, Callback, CallbackRe, TieBreaking, Deterministic>::draw_policy_graph(
    std::ostream &out)
{
    out << "digraph {" << endl;
    std::set<size_t> drawn;
    std::list<size_t> open;
    open.push_back(this->get_initial_state_id());
    drawn.insert(open.back());
    Node istat = this->get_initial_state();
    int x = oselector(istat);
    while (!open.empty()) {
        Node node = this->search_space.get_prob_state(open.back());
        out << "  s" << open.back() << " [label=\"(" << open.back() << ") v = "
            << node.get_v_current()
            << "; p = "  << node.get_policy_operator_index();
        if (x == node.get_prob_state_id()) {
            out << "; NEXT";
        }
        out << "\"";
        if (node.is_dead_end()) {
            out << ", peripheries=2";
        } else if (node.is_goal()) {
            out << ", shape=rectangle, peripheries=2";
        } else if (node.is_opened()) {
            out << ", shape=hexagon";
        }
        out << "]" << endl;

        open.pop_back();
        if (!node.is_closed()) {
            continue;
        }
        typename StateSpace::successor_iterator it =
            this->state_space.successors(node, node.get_policy_operator_index());
        while (!it.end()) {
            if (drawn.find(*it) == drawn.end()) {
                drawn.insert(*it);
                open.push_back(*it);
            }
            it++;
        }
    }

    for (set<size_t>::iterator s = drawn.begin(); s != drawn.end(); s++) {
        Node node = this->search_space.get_prob_state(*s);
        if (!node.is_closed()) {
            continue;
        }
        typename StateSpace::successor_iterator it =
            this->state_space.successors(node, node.get_policy_operator_index());
        while (!it.end()) {
            out << "  s" << (*s) << " -> s"
                << (*it) << "[label=\""  << it.prob()
                << "\"]" << endl;
            it++;
        }
    }

    out << "}" << endl;
}

#endif
