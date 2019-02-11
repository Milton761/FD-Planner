
#ifndef TOPOLOGICAL_VALUE_ITERATION_H
#define TOPOLOGICAL_VALUE_ITERATION_H

#include "prob_search_engine.h"
#include "prob_search_space.h"
#include "prob_search_node_info.h"

#include "value_initializer.h"
#include "value_reporter.h"

#include "option_parser.h"

#include <iostream>
#include <deque>
#include <queue>

#define TVI_ITERATIVE_TARJAN 1

struct TVIStateInfo : public BasicStateInfo {
    constexpr static const size_t UNDEFINED = -1;
    size_t index;
    size_t lowlink;
    size_t largelink;
/*
#if TVI_ITERATIVE_TARJAN
    size_t parent;
    size_t open;
#endif
*/
    bool onstack;
    TVIStateInfo(int sid, int budget)
        : BasicStateInfo(sid, budget), index(UNDEFINED), lowlink(UNDEFINED),
          largelink(0), onstack(false) {}
};

template<typename NodeInfo>
class TVIState : public BasicState<NodeInfo>
{
public:
    TVIState(size_t sid, NodeInfo &info, unsigned u)
        : BasicState<NodeInfo>(sid, info, u) {}
    size_t get_index() const {
        return this->info.index;
    }
    void set_index(size_t index) {
        this->info.index = index;
    }
    bool is_index_undefined() const {
        return this->info.index == NodeInfo::UNDEFINED;
    }
    size_t get_lowlink() const {
        return this->info.lowlink;
    }
    void set_lowlink(size_t lowlink) {
        this->info.lowlink = lowlink;
    }
    bool is_onstack() const {
        return this->info.onstack;
    }
    void set_onstack(bool onstack = true) {
        this->info.onstack = onstack;
    }
    void set_largelink(size_t largelink) {
        this->info.largelink = largelink;
    }
    size_t get_largelink() const {
        return this->info.largelink;
    }
    /*
    size_t &parent() {
        return this->info.parent;
    }
    size_t &open() {
        return this->info.open;
    }
    */
};


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
class TopologicalValueIteration : public
    ProbSearchEngine
    <NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
{
    typedef NODE<NodeInfo> State;
    typedef SSearchSpace<SStateSpace, NodeInfo, NODE> SearchSpace;
    typedef SStateSpace<State, typename SearchSpace::StateIDLookup> StateSpace;
protected:

#if TVI_ITERATIVE_TARJAN
    struct OpenElem {
        size_t stateid;
        typename StateSpace::applicable_op_iterator aop;
        typename StateSpace::successor_iterator it;
        bool stopped;
        OpenElem(StateSpace &sp, State state)
            :
            stateid(state.get_prob_state_id()),
            aop(sp.applicable_operators(state)),
            it(sp.successors(state, aop)),
            stopped(false) {
            aop++;
        }
    };

    size_t proceed(OpenElem *elem, bool &changed);

    std::deque<OpenElem*> open;
#endif

    void expand(State state) {
        if (!this->generate_all_successors(state)) {
            this->value_initializer.dead_end(state);
            state.mark_as_dead_end();
        }
    }

    float epsilon;

    size_t index;
    std::deque<size_t> stack;

    typedef std::pair<size_t, std::vector<size_t>*> PrioEntry;
    struct Compare {
        bool operator()(const PrioEntry &e1, const PrioEntry &e2) const {
            return e1.first > e2.first;
        }
    };
    std::priority_queue < PrioEntry,
        std::vector<PrioEntry>,
        Compare > update_order;

    void solve(size_t id);

    virtual SearchStatus step();
public:
    TopologicalValueIteration(const Options &opts);
    virtual ~TopologicalValueIteration() { }
    virtual void print_statistics() const {}
    virtual void print_options() {
        std::cout << "Running Topological VI..." << std::endl;
        ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options();
    }
};

#include "topological_vi.cc"

#endif
