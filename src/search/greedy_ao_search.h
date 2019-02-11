
#ifndef AO_SEARCH_H
#define AO_SEARCH_H

#include "prob_search_engine.h"
#include "prob_search_space.h"
#include "prob_search_node_info.h"
#include "rng.h"

#include "timer.h"

#include "segmented_vector.h"

#include "utilities.h"

#include "ao_value_updater.h"
#include "ao_tip_list.h"
#include "value_initializer.h"
#include "value_reporter.h"

#include "action_tie_breaking.h"

#include <iostream>
#include <list>
#include <set>
#include <queue>
#include <deque>

class Options;
class OptionParser;

template<typename State>
struct OnNewStateStoreHAndPreferredOps {
private:
    Heuristic *pref_h;
public:
    OnNewStateStoreHAndPreferredOps(Heuristic *pref_h)
        : pref_h(pref_h) {}
    template<typename SearchSpace, typename StateSpace>
    void operator()(SearchSpace &, StateSpace &state_space, const State *,
            State &state, const GlobalOperator *, float h) {
        state.set_estimated_distance(h);
        if (pref_h != NULL) {
            bool x;
            state_space.compute_goal_estimation(*pref_h, state, x,
                    state.get_preferred_outcomes());
        }
    }
};

template<typename OpenList>
struct PreferredOpenListAdder {
private:
    OpenList *l;
    OpenList *pl;
public:
    PreferredOpenListAdder(OpenList *l, OpenList *pl) :
        l(l), pl(pl) {}
    template<typename State, typename SearchSpace, typename StateSpace>
    void operator()(SearchSpace &, StateSpace &, const State &parent, State &state,
                    const GlobalOperator *op, bool is_new) {
        if (!is_new || !state.is_opened()) return; // have already handled this successor
        state.set_depth(parent.get_depth() + 1);
        if (parent.get_preferred_outcomes().count(op) > 0) {
            pl->insert(state);
        } else {
            l->insert(state);
        }
    }
};

template<typename State, typename SearchSpace, typename StateSpace, typename OpenList>
class AOLOutcomeSelector
{
    OpenList *l;
    OpenList *pl;
    SearchSpace *search_space;
    StateSpace *state_space;
    int pop(OpenList *open) const {
        while (true) {
            if (open->empty()) {
                return -1;
            }
            size_t sid = open->next();
            State state = this->search_space->get_prob_state(sid);
            if (state.is_closed() || state.is_dead_end() || state.is_goal()) {
                continue;
            }
            return sid;
        }
    }
public:
    AOLOutcomeSelector(OpenList *l, OpenList *pl) : l(l), pl(pl) {}
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &) {
        int res = pop(pl);
        if (res < 0) {
            res = pop(l);
        }
        return res;
    }
};

template<typename ValueInitializer>
struct AOValueInitializer {
    ValueInitializer initializer;

    template<class State>
    inline void dead_end(State &state) const {
        initializer.dead_end(state);
        state.unmark();
    }

    template<class State>
    inline void goal(State &state) const {
        initializer.goal(state);
        state.unmark();
    }

    template<class State>
    inline void set_policy(State &state, int op) const {
        initializer.set_policy(state, op);
    }

    template<class State>
    inline void operator()(State &state) const {
        initializer(state);
        state.mark();
    }
};

template<typename State, typename SearchSpace, typename StateSpace>
struct AOOutcomeSelectorArbitrary {
    SearchSpace *search_space;
    StateSpace *state_space;
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &state) const {
        typename StateSpace::successor_iterator it =
            state_space->successors(state, state.get_policy_operator_index());
        while (!it.end()) {
            State succ = search_space->get_prob_state(*it);
            if (succ.is_opened() || succ.is_marked()) {
                return *it;
            }
            it++;
        }
        return -1;
    }
};

template<typename State, typename SearchSpace, typename StateSpace>
struct AOOutcomeSelectorMostLikely {
    SearchSpace *search_space;
    StateSpace *state_space;
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &state) const {
        typename StateSpace::successor_iterator it =
            state_space->successors(state, state.get_policy_operator_index());
        int res = -1;
        float best = 0;
        while (!it.end()) {
            State succ = search_space->get_prob_state(*it);
            if (succ.is_opened() || succ.is_marked()) {
                //return *it;
                if (it.prob() > best) {
                    best = it.prob();
                    res = *it;
                }
            }
            it++;
        }
        return res;
    }
};

template<typename State, typename SearchSpace, typename StateSpace>
struct AOOutcomeSelectorMinH {
    SearchSpace *search_space;
    StateSpace *state_space;
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &state) const {
        std::pair<float, float> best = std::make_pair(std::numeric_limits<float>::max(),
                                       1);
        size_t sid = 0;
        typename StateSpace::successor_iterator it =
            state_space->successors(state, state.get_policy_operator_index());
        while (!it.end()) {
            State succ = search_space->get_prob_state(*it);
            if (succ.is_opened() || succ.is_marked()) {
                std::pair<float, float> k = std::make_pair(it.prob() *
                                            succ.get_estimated_distance(),
                                            1.0 - it.prob());
                if (k < best) {
                    sid = *it;
                    best = k;
                }
            }
            it++;
        }
        return best.first == std::numeric_limits<float>::max() ? -1 : sid;
    }
};

template<typename State, typename SearchSpace, typename StateSpace>
struct AOOutcomeSelectorGap {
    SearchSpace *search_space;
    StateSpace *state_space;
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &state) const {
        std::pair<float, float> best = std::make_pair(-1, 0);
        float tmp;
        size_t sid = 0;
        typename StateSpace::successor_iterator it =
            state_space->successors(state, state.get_policy_operator_index());
        while (!it.end()) {
            State succ = search_space->get_prob_state(*it);
            if (succ.is_opened() || succ.is_marked()) {
                tmp = it.prob() * fabs(state.get_v_current() - state.get_v_optimistic());
                std::pair<float, float> k = std::make_pair(tmp, it.prob());
                if (k > best) {
                    sid = *it;
                    best = k;
                }
            }
            it++;
        }
        return best.first == -1 ? -1 : sid;
    }
};

template<typename State, typename SearchSpace, typename StateSpace>
struct AOOutcomeSelectorPreferred {
    SearchSpace *search_space;
    StateSpace *state_space;
    void initialize(SearchSpace &search_space, StateSpace &state_space) {
        this->search_space = &search_space;
        this->state_space = &state_space;
    }
    int operator()(State &state) const {
        std::pair<bool, float> best = std::make_pair(false, 0);
        size_t sid = 0;
        typename StateSpace::successor_iterator it =
            state_space->successors(state, state.get_policy_operator_index());
        size_t prob_operator = this->state_space->get_prob_operator_id(state,
                               state.get_policy_operator_index());
        size_t local_ref = 0;
        while (!it.end()) {
            State succ = search_space->get_prob_state(*it);
            if (succ.is_opened() || succ.is_marked()) {
                std::pair<bool, float> k = std::make_pair(
                                               state.get_preferred_outcomes().count(
                                                   &g_operators[g_prob_operators[prob_operator][local_ref].second]) > 0,
                                               it.prob());
                if (k > best) {
                    sid = *it;
                    best = k;
                }
            }
            local_ref++;
            it++;
        }
        return best.first == -1 ? -1 : sid;
    }
};


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, template <typename, typename, typename, typename> class ValueUpdater, template<typename, typename, typename> class OutcomeSelector, template<typename> class Callback, typename CallbackRe, typename TieBreaking, bool Deterministic>
class AOSearch : public
    ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
{
public:
    typedef NODE<NodeInfo> Node;
    typedef SSearchSpace<SStateSpace, NodeInfo, NODE> SearchSpace;
    typedef SStateSpace<Node, typename SearchSpace::StateIDLookup> StateSpace;
private:
    size_t num_iterations;
protected:
    //OnSuccessorIncrUnsolved callback;
    Timer _t;
    Callback<Node> &callback;
    CallbackRe &callback_re;
    ValueUpdater<Node, SearchSpace, StateSpace, TieBreaking> vupdater;
    OutcomeSelector<Node, SearchSpace, StateSpace> &oselector;

    void expand_node(Node node);
    virtual SearchStatus step();
public:
    AOSearch(const Options &opts);
    virtual ~AOSearch() {}
    virtual void print_statistics() const {
        cout << "Number of expansions: " << num_iterations << endl;
        if (Deterministic) {
            printf("Spent %.4f seconds on computing deterministic plans.\n",
                   _t());
        }
    }
    void draw_policy_graph(std::ostream &out);
    virtual void print_options() {
        std::cout << "Running AO*..." << std::endl;
        ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options();
    }
};


#include "greedy_ao_search.cc"

#endif

