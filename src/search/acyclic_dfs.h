
#ifndef ACYCLIC_SEARCH_H
#define ACYCLIC_SEARCH_H

#include "prob_search_engine.h"

#include <list>
#include <queue>
#include <limits>
#include <ctime>

struct IsLabeledSolved {
    template<typename SearchSpace, typename StateSpace, typename State, typename TieBreaking>
    bool operator()(SearchSpace &, StateSpace &, TieBreaking &, State state,
                    float) const {
        return state.is_solved();
    }
};

struct VIConverged {
private:
    struct Comparator {
        bool operator()(const std::pair<int, size_t> &x1,
                        const std::pair<int, size_t> &x2) const {
            return x1.first > x2.first;
        }
    };
    Comparator comp;
    template<typename SearchSpace, typename StateSpace, typename State>
    void dfs(SearchSpace &search_space, StateSpace &state_space, State state,
             std::priority_queue < std::pair<int, size_t>,
             std::vector<std::pair<int, size_t> >,
             Comparator > &order
            ) {
        //if (state.get_policy_operator_index() < 0) {
        //    state.dump_status();
        //}
        assert(state.get_policy_operator_index() >= 0);

        state.set_largelink(0);
        unsigned largelink = 0;
        state.mark();
        typename StateSpace::successor_iterator it = state_space.successors(state,
                state.get_policy_operator_index());
        for (; !it.end(); it++) {
            State succ = search_space.get_prob_state(*it);
            if (!succ.is_goal() && !succ.is_dead_end() && !succ.is_solved()) {
                if (!succ.is_marked()) {
                    dfs(search_space, state_space, succ, order);
                }
                if (succ.get_largelink() >= largelink) {
                    largelink = succ.get_largelink() + 1;
                }
            }
        }
        state.set_largelink(largelink);
        order.push(std::make_pair(state.get_largelink(), state.get_prob_state_id()));
    }
public:
    VIConverged() {}
    template<typename SearchSpace, typename StateSpace, typename State, typename TieBreaking>
    bool operator()(SearchSpace &search_space, StateSpace &state_space,
                    TieBreaking &tiebreaking, State init, float epsilon) {
        std::priority_queue < std::pair<int, size_t>,
            std::vector<std::pair<int, size_t> >,
            Comparator > order;
        dfs(search_space, state_space, init, order);
        bool changed = false;
        int old_op;
        std::pair<float, int> newv;
        while (!order.empty()) {
            State state = search_space.get_prob_state(order.top().second);
            state.unmark();
            order.pop();
            old_op = state.get_policy_operator_index();
            search_space.qval_update(state, newv, tiebreaking, epsilon);
            if (old_op != newv.second) {
                changed = true;
            }
        }
        return !changed;
    }
};

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
class AcyclicDFSEngine : public
    ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
{
private:
    typedef Node<NodeInfo> State;
    typedef SSearchSpace<SStateSpace, NodeInfo, Node> SearchSpace;
    typedef SStateSpace<State, typename SearchSpace::StateIDLookup> StateSpace;

    const float epsilon;

    const bool labeling;
    const bool fw_updates;
    const bool bw_updates;
    const bool terminate_trial;
    const bool stop_inconsistent;

    const bool cache_flags;

    Callback<State> callback;
    TieBreaking tiebreaking;

    CheckSolved check_solved;
    VLookup getv;

    std::list<size_t> visited;

    bool expand(State state) {
        if (!this->generate_all_successors(state, callback, !fw_updates)) {
            state.mark_as_dead_end();
            this->value_initializer.dead_end(state);
            return false;
        } else {
            state.close();
            return true;
        }
    }
    bool dfs(State state, bool &has_changed, bool &incomplete);
    virtual SearchStatus step();
public:
    AcyclicDFSEngine(const Options &opts);
    virtual ~AcyclicDFSEngine() {}
    virtual void print_options();
};


#include "acyclic_dfs.cc"

#endif

