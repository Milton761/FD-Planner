
#ifndef TARJAN_SEARCH_H
#define TARJAN_SEARCH_H

#include "prob_search_engine.h"

#include "segmented_vector.h"

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

struct TVIConverged {
private:

    struct Comparator {
        bool operator()(const std::pair<int, std::vector<size_t>*> &x1,
                        const std::pair<int, std::vector<size_t>*> &x2) const {
            return x1.first > x2.first;
        }
    };
    std::list<size_t> stack;
    int current_index;
    Comparator comp;
    template<typename SearchSpace, typename StateSpace, typename State>
    void dfs(SearchSpace &search_space, StateSpace &state_space, State state,
             std::priority_queue < std::pair<int, std::vector<size_t>*>,
             std::vector<std::pair<int, std::vector<size_t>*> >,
             Comparator > &order
            ) {
        if (state.is_goal() || state.is_dead_end()) {
            return;
        }

        assert(state.get_policy_operator_index() >= 0);

        stack.push_front(state.get_prob_state_id());
        state.set_index(current_index);
        state.set_lowlink(current_index++);
        state.set_onstack(true);
        state.set_largelink(0);

        unsigned largelink = 0;
        typename StateSpace::successor_iterator it = state_space.successors(state,
                state.get_policy_operator_index());
        for (; !it.end(); it++) {
            State succ = search_space.get_prob_state(*it);
            if (!succ.is_goal() && !succ.is_dead_end() && !succ.is_solved()) {
                if (!succ.visited()) {
                    dfs(search_space, state_space, succ, order);
                    if (succ.visited())
                    state.update_lowlink(succ.get_lowlink());
                } else if (succ.is_onstack()) {
                    state.update_lowlink(succ.get_index());
                }
                if (succ.get_largelink() >= largelink) {
                    largelink = succ.get_largelink() + 1;
                }
            }
        }
        state.set_largelink(largelink);

        if (state.get_index() == state.get_lowlink()) {
            std::vector<size_t> *scc = new std::vector<size_t>();
            std::list<size_t>::iterator it = stack.begin();
            while (true) {
                State s = search_space.get_prob_state(*it);
                s.set_largelink(largelink);
                s.set_onstack(false);
                scc->push_back(*it);

                if (*(it++) == state.get_prob_state_id()) {
                    break;
                }
            }
            stack.erase(stack.begin(), it);
            order.push(std::make_pair(largelink, scc));
        }
    }
public:
    TVIConverged() {}
    template<typename SearchSpace, typename StateSpace, typename State, typename TieBreaking>
    bool operator()(SearchSpace &search_space, StateSpace &state_space,
                    TieBreaking &tiebreaking, State init, float epsilon) {
        std::priority_queue < std::pair<int, std::vector<size_t>*>,
            std::vector<std::pair<int, std::vector<size_t>*> >,
            Comparator > order;
        dfs(search_space, state_space, init, order);
        bool changed = false;
        int old_op;
        std::pair<float, int> newv;
        while (!order.empty()) {
            std::vector<size_t> *scc = order.top().second;
            order.pop();
            float error;
            do {
                error = 0;
                float tmp;
                for (uint i = 0; i < scc->size(); i++) {
                    State state = search_space.get_prob_state((*scc)[i]);
                    state.clear();
                    old_op = state.get_policy_operator_index();
                    tmp = search_space.qval_update(state, newv, tiebreaking, epsilon);
                    if (old_op != newv.second) {
                        changed = true;
                    }
                    if (tmp > error) {
                        error = tmp;
                    }
                }
            } while (error > epsilon);
            delete(scc);
        }
        return !changed;
    }
};

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
class TarjanEngine : public
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
    std::list<size_t> stack;

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
    bool dfs(State state, bool &has_changed, int &current_index);
    virtual SearchStatus step();
public:
    TarjanEngine(const Options &opts);
    virtual ~TarjanEngine() {}
    virtual void print_options();
};


#include "tarjan_dfs.cc"

#endif

