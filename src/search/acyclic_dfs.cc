
#ifdef ACYCLIC_SEARCH_H

#include <fstream>

#include "option_parser.h"

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
AcyclicDFSEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
AcyclicDFSEngine(
    const Options &opts) :
    ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
    (opts),
    epsilon(opts.get<float>("epsilon")),
    labeling(opts.get<bool>("labeling")),
    fw_updates(opts.get<bool>("fw_updates")),
    bw_updates(opts.get<bool>("bw_updates")),
    terminate_trial(opts.get<bool>("terminate_trial")),
    stop_inconsistent(opts.get<bool>("inconsistent")),
    cache_flags(opts.get<bool>("cache_flags"))
{}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
void
AcyclicDFSEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
print_options() {
    std::cout << "Running PROB-DFS on acyclic problems" << std::endl;
    ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options();
    std::cout << "Labeling states solved: " << labeling << std::endl;
    std::cout << "Doing forward updates: " << fw_updates << std::endl;
    std::cout << "Doing backward updates: " << bw_updates << std::endl;
    std::cout << "Terminating trials: " << terminate_trial << std::endl;
    std::cout << "Caching flags: " << cache_flags << std::endl;
    std::cout << "Stopping DFS exploration at ";
    if (stop_inconsistent) {
        std::cout << "inconsistent";
    } else {
        std::cout << "terminal";
    }
    std::cout << " states" << std::endl;
}


template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
bool
AcyclicDFSEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
dfs(State state, bool &has_changed, bool &incomplete)
{
    if (state.is_opened()) {
        if (!expand(state)) {
            return true;
        }
    }

    if (state.is_goal() || state.is_dead_end() || state.is_solved()) {
        state.set_solved();
        return false;
    }

    int oldp;
    float oldv = getv(state);
    std::pair<float, int> newv;
    if (fw_updates) {
        this->search_space.qval_update(state, newv, tiebreaking, epsilon);
        assert(newv.second >= 0);
    }

    bool flag = false;
    if (fabs(oldv - getv(state)) > epsilon) {
        flag = true;
        if (stop_inconsistent) {
            incomplete = true;
            return true;
        }
    }

    state.mark();
    visited.push_back(state.get_prob_state_id());
    //std::cout << "==== START >" << state.get_prob_state_id() << "< ====" << std::endl;

    //size_t expansions = 0;
    //size_t successors = 0;

    assert(state.get_policy_operator_index() >= 0);
    typename StateSpace::successor_iterator it =
        this->state_space.successors(state, state.get_policy_operator_index());
    for (; !it.end(); it++) {
        //successors++;
        State succ = this->search_space.get_prob_state(*it);
        if (!succ.is_marked()) {
            //expansions ++;
            flag = dfs(succ, has_changed, incomplete) || flag;
        }
        else if (cache_flags) {
            flag = flag || succ.is_flagged();
        }
        if (terminate_trial && flag) {
            incomplete = true;
            break;
        }
    }

    if ((!fw_updates || flag) && bw_updates) {
        oldp = state.get_policy_operator_index();
        this->search_space.qval_update(state, newv, tiebreaking, epsilon);
        has_changed = has_changed || oldp != newv.second;
    }

    if (flag && !cache_flags) {
        state.unmark();
    }

    if (!flag && labeling) {
        state.set_solved();
    }

    state.flag(flag);

    //std::cout << "==== END >" << state.get_prob_state_id() << "< --> "
    //    << flag << " (" << expansions << "/ " << successors << ") ====" << std::endl;

    return flag;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
SearchStatus
AcyclicDFSEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
step()
{
    bool has_changed = false;
    bool incomplete = false;
    do {
        this->report_progress(this->search_space, this->get_initial_state_id());

        has_changed = false;
        incomplete = false;
        dfs(this->get_initial_state(), has_changed, incomplete);

        std::list<size_t>::iterator it = visited.begin();
        while (it != visited.end()) {
            this->search_space.get_prob_state(*it).unmark();
            this->search_space.get_prob_state(*it).unflag();
            it++;
        }
        visited.clear();
        //std::cout << std::endl;
    } while (incomplete || has_changed || !check_solved(this->search_space, this->state_space, tiebreaking, this->get_initial_state(), epsilon));

    //std::ofstream out;
    //out.open("pol.dot");
    //this->search_space.dump_policy_graph2(out);
    //out.close();

    return SOLVED;
}

#endif

