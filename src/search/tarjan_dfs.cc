
#ifdef TARJAN_SEARCH_H

#include <fstream>

#include "option_parser.h"

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
TarjanEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
TarjanEngine(
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
TarjanEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
print_options() {
    std::cout << "Conducting PROB-DFS with cycle detection" << std::endl;
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
TarjanEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
dfs(State state, bool &has_changed, int &current_index)
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

    bool flag = false;
    int oldp;
    std::pair<float, int> newv;
    if (fw_updates) {
        float oldv = getv(state);
        this->search_space.qval_update(state, newv, tiebreaking, epsilon);
        assert(newv.second >= 0);

        if (fabs(oldv - getv(state)) > epsilon) {
            flag = true;
            if (stop_inconsistent) {
                return true;
            }
        }
    }

    state.set_onstack(true);
    state.set_index(current_index);
    state.set_lowlink(current_index++);
    visited.push_back(state.get_prob_state_id());
    stack.push_front(state.get_prob_state_id());

    assert(state.get_policy_operator_index() >= 0);
    typename StateSpace::successor_iterator it =
        this->state_space.successors(state, state.get_policy_operator_index());
    for (; !it.end(); it++) {
        //successors++;
        //bool recursed = false;
        State succ = this->search_space.get_prob_state(*it);
        if (!succ.visited()) {
            //recursed = true;
            flag = dfs(succ, has_changed, current_index) || flag;
            if (succ.visited())
                state.update_lowlink(succ.get_lowlink());
        } else if (succ.is_onstack()) {
            state.update_lowlink(succ.get_index());
        }

        /*
        if (!recursed && cache_flags) {
            flag = flag || succ.is_flagged();
        }
        */

        if (terminate_trial && flag) {
            break;
        }
    }

    if ((!fw_updates || flag) && bw_updates) {
        oldp = state.get_policy_operator_index();
        this->search_space.qval_update(state, newv, tiebreaking, epsilon);
        has_changed = has_changed || oldp != newv.second;
        state.clear();
        stack.pop_front();
    }

    //if (flag && !cache_flags) {
    //    state.unmark();
    //}

    //std::cout << "{END-DFS(" << state.get_prob_state_id() << ") with"
    //    << " index=" << state.get_index()
    //    << " lowlink=" << state.get_lowlink()
    //    << " flag=" << flag
    //    << "}" << std::endl;
    if (!flag && labeling && state.get_index() == state.get_lowlink()) {
        //std::cout << "SOLVED SCC [ " << std::flush;
        std::list<size_t>::iterator it = stack.begin();
        while (true) {
            //std::cout << *it << " ";
            State s = this->search_space.get_prob_state(*it);
            s.set_solved();
            //s.flag(flag);
            s.clear();
            if ((*(it++)) == state.get_prob_state_id()) {
                break;
            }
        }
        stack.erase(stack.begin(), it);
        //std::cout << "]" << std::endl;
    }

    // TODO ?
    //state.flag(flag);

    //std::cout << "==== END >" << state.get_prob_state_id() << "< --> "
    //    << flag << " (" << expansions << "/ " << successors << ") ====" << std::endl;

    return flag;
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
SearchStatus
TarjanEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>::
step()
{
    bool has_changed = false;
    bool incomplete = false;
    do {
        this->report_progress(this->search_space, this->get_initial_state_id());

        has_changed = false;
        int current_index = 0;
        incomplete = dfs(this->get_initial_state(), has_changed, current_index);
        stack.clear();


        std::list<size_t>::iterator it = visited.begin();
        while (it != visited.end()) {
            this->search_space.get_prob_state(*it).clear();
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

