
#ifndef LRTDP_H
#define LRTDP_H

#include "prob_search_engine.h"
#include "prob_search_space.h"
#include "prob_search_node_info.h"
#include "rng.h"

#include "timer.h"

#include "value_initializer.h"
#include "value_reporter.h"

#include "action_tie_breaking.h"

#include <deque>
#include <iostream>

class Options;
class OptionParser;

struct LRTDPStdSampler {
    std::pair<float, int> newv;
    RandomNumberGenerator gen;
    bool epsilon_consistent;
    const float epsilon;
    LRTDPStdSampler(int seed, bool epsilon_consistent, float epsilon) : gen(seed), epsilon_consistent(epsilon_consistent), epsilon(epsilon) {}
    template<class SearchSpace, class StateSpace, class State, typename TieBreaking>
    int operator()(SearchSpace &search_space, StateSpace &state_space,
                   State &state, TieBreaking &tb) {
        float diff = search_space.qval_update(state, newv, tb, epsilon);
        if (epsilon_consistent && diff <= epsilon) {
            return -1;
        }
        if (newv.second == -1) {
            return -1;
        }
        float r = gen();

        typename StateSpace::successor_iterator it =
            state_space.successors(state, newv.second);
        while (!it.end()) {
            if (it.prob() + g_epsilon >= r) {
                return *it;
            }
            r -= it.prob();
            it++;
        }

        return -1;
    }
};

template<typename V>
struct BiasedLRTDPSampler {
    V v;
    std::pair<float, int> newv;
    RandomNumberGenerator gen;
    bool epsilon_consistent;
    const float epsilon;
    BiasedLRTDPSampler(int seed, bool epsilon_consistent, float epsilon) : gen(seed), epsilon_consistent(epsilon_consistent), epsilon(epsilon) {}
    template<class SearchSpace, class StateSpace, class State, typename TieBreaking>
    int operator()(SearchSpace &search_space, StateSpace &state_space,
                   State &state, TieBreaking &tb) {
        float diff = search_space.qval_update(state, newv, tb, epsilon);
        if (newv.second == -1 || (epsilon_consistent && diff <= epsilon)) {
            return -1;
        }

        float r = gen();
        typename StateSpace::successor_iterator it =
            state_space.successors(state, newv.second);
        float cur = 0;
        std::deque<std::pair<float, size_t> > outcomes;
        while (!it.end()) {
            State succ = search_space.get_prob_state(*it);
            cur += v(succ, it.prob());
            outcomes.push_back(make_pair(cur, *it));
            it++;
        }
        r *= cur;
        for (std::deque<std::pair<float, size_t> >::iterator i = outcomes.begin();
             i != outcomes.end(); i++) {
            if (i->first >= r) {
                return i->second;
            }
        }

        return -1;
    }
};

template<typename Combine, typename Bias>
struct _CombinedBias {
    Combine combine;
    Bias bias;
    template<typename State>
    inline float operator()(const State &state, const float &prob) const {
        return combine(prob, bias(state));
    }
};

struct _GapBias {
    template<typename State>
    inline float operator()(const State &state) const {
        return fabs(state.get_v_current() - state.get_v_optimistic());
    }
};

struct _InverseHBias {
    template<typename State>
    inline float operator()(const State &state) const {
        return (1.0 / state.get_estimated_distance());
    }
};

struct _HBias {
    template<typename State>
    inline float operator()(const State &state) const {
        return (state.get_estimated_distance());
    }
};

template<typename V>
struct _ValueBias {
    V v;
    template<typename State>
    inline float operator()(const State &state) const {
        return v(state);
    }
};

struct _CombineMultiplication {
    inline float operator()(const float &prob, float val) const {
        return prob * val;
    }
};

struct _CombineMC {
    inline float operator()(const float &prob, float val) const {
        return (1.0 + prob) * val;
    }
};

struct BiasedGap : public _CombinedBias<_CombineMultiplication, _GapBias> {};
struct BiasedH : public _CombinedBias<_CombineMultiplication, _InverseHBias> {};
struct BiasedH_INV : public _CombinedBias<_CombineMultiplication, _HBias> {};
struct BiasedMCL : public
        _CombinedBias<_CombineMC, _ValueBias<OptimisticV> > {};
struct BiasedMCU : public
        _CombinedBias<_CombineMC, _ValueBias<BiOptimisticV> > {};

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
class LRTDP : public
    ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
{
private:
    typedef NODE<NodeInfo> State;
    typedef SSearchSpace<SStateSpace, NodeInfo, NODE> SearchSpace;
    typedef SStateSpace<State, typename SearchSpace::StateIDLookup> StateSpace;
    size_t num_trials;
    size_t num_open_states;
    Timer _t;
protected:
    //const bool lazy;
    TieBreaking tie_breaking;
    typedef NODE<NodeInfo> Node;
    Callback<Node> callback;
    Value val;
    Sampler &sample;

    float epsilon;

    bool expand_state(Node &state);
    float adaptive_qval(Node &state, std::pair<float, int> &newv);
    float adaptive_qval_update(Node &state, std::pair<float, int> &newv);
    bool check_and_solve(size_t id);

    virtual SearchStatus step();
public:
    LRTDP(const Options &opts);
    virtual ~LRTDP() {}
    virtual void print_statistics() const;

    virtual void print_options() {
        std::cout << "Running LRTDP..." << std::endl;
        ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options();
    }
};
#include "lrtdp.cc"

#endif
