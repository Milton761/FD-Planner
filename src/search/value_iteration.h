
#ifndef VALUEITERATION_H
#define VALUEITERATION_H

#include "prob_search_engine.h"
#include "prob_search_space.h"
#include "prob_search_node_info.h"
#include "rng.h"

#include "state_space.h"

#include <iostream>

class Options;
class OptionParser;

template<typename NodeInfo, template<typename> class NODE, int Type, template<template<typename, typename> class, typename, template<typename> class, int> class SSearchSpace, template<typename, typename> class StateSpace>
class ValueIteration : public
    ProbSearchEngine<NodeInfo, NODE, Type, SSearchSpace, StateSpace>
{
protected:
    typedef NODE<NodeInfo> Node;

    float epsilon;
    size_t _initial_state;

    virtual void initialize();
    virtual SearchStatus step();
public:
    ValueIteration(const Options &opts);
    virtual ~ValueIteration() {}
    virtual void print_statistics() const {}
    void dump_states();
    void dump_table();
};

template<int TType>
struct VIFactory {
    typedef BasicStateInfo StateInfo;
    template<typename Info> struct State : public BasicState<Info> {
        State(size_t a, Info &b) : BasicState<Info>(a, b) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C, int D>
    struct SearchSpace : public ProbSearchSpace<A, B, C, D> {
        SearchSpace(A<C<B>, typename ProbSearchSpace<A, B, C, D>::StateIDLookup>
                    &state_space) : ProbSearchSpace<A, B, C, D>(state_space) {}
    };
    //template<template<typename, typename> class A, typename B, template<typename> class C, int D>
    //struct SearchSpace : public ProbSearchSpace<A, B, C, D> {
    //    SearchSpace(A<B<C>, SearchSpace<A, B, C, D> > &a) : ProbSearchSpace<A, B, C, D>(a) {}
    //};
    static constexpr int Type = TType;

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(const Options &opts) const {
        return new ValueIteration<StateInfo, State, Type, SearchSpace, StateSpace>
               (opts);
    }

    static void add_options_to_parser(OptionParser &parser);
};

#include "value_iteration.cc"

#endif
