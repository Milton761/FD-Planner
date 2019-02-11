
#ifndef ACYCLIC_VALUEITERATION_H
#define ACYCLIC_VALUEITERATION_H

#include "prob_search_engine.h"
#include "prob_search_space.h"
#include "prob_search_node_info.h"
#include "rng.h"

#include "value_initializer.h"
#include "value_reporter.h"

#include <iostream>

class Options;
class OptionParser;

/* Per state data, stored in search space.
   If your algorithm requires to store data that is not stored in BasicStateInfo,
   create a new class inheriting from BasicStateInfo. For example:
*/
struct AcyclicProbSearchNodeInfo : public BasicStateInfo {
    unsigned unsolved_successors;
    AcyclicProbSearchNodeInfo(int sid, int budget)
        : BasicStateInfo(sid, budget), unsolved_successors(0) {}
};

/* Wrapper to access per state data; is created when reading a state
   from the search space.
   Whenever you create a new StateInfo class, you have to define the
   corresponding ProbSearchNode which provides an interface to the additional
   data defined in the new StateInfo class. For example:
*/
template<typename NodeInfo>
class AcyclicProbSearchNode : public BasicState<NodeInfo>
{
public:
    AcyclicProbSearchNode(size_t sid, NodeInfo &info, unsigned u)
        : BasicState<NodeInfo>(sid, info, u) {}
    int get_order_id() const {
        return this->info.unsolved_successors;
    }
    void set_order_id(int id) {
        this->info.unsolved_successors = id;
    }
};

/*
    NodeInfo         -- per state data; stored in search space
    NODE             -- template is instantiated to NODE<NodeInfo>; is a wrapper class
                        to access a state's data; is created by search space
    SSearchSpace     -- template is instantiated to SSearchSpace<SStateSpace, NodeInfo, NODE>;
                        it stores for each state in the state space additional
                        data, as defined by NodeInfo; provides interfaces to access
                        this data
    SStateSpace      -- template is instantiated to SStateSpace<NODE<NodeInfo>, SearchSpace::StateIDLookup>;
                        defines the underlying state space on which the search
                        algorithm is ran; SearchSpace::StateIDLookup provides an
                        interfaces to get the state id as defined by the state space
                        from a NODE<NodeInfo> object; the state space provides an
                        interfaces to (a) check whether a state is a dead end, (b)
                        check whether a state is a goal state, and (c) defines
                        the successor state relation
    ValueInitializer -- called to initialize the value(s) stored for a state; is
                        called whenever a new, not yet seen state has been found;
                        moreover, it is called whenever a dead end state is found:
                        it then sets the value to V^*(s) for dead end state s (e.g.,
                        in case of MaxProb, sets the value of s to 0; for MinExpCost
                        sets the value to the give up cost)
    ValueReporter    -- used to output the current value of the initial state
    TieBreaking      -- determines the tiebreaking choice between the actions that are
                        greedy under the updated value function

*/
/* Implementation of Value Iteration that first generates the part of state space
   that contains all states reachable from the initial states. While generating
   the state space, it maintains a topological ordering of the states. After the
   generation is completed, it performs updates on all these states, following the
   topological ordering. By following the topligical ordering, we only have to do
   one QVal update per state. */
template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
class AcyclicValueIteration : public
    ProbSearchEngine <NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
{
protected:
    /* Recommended typedefs for easier usage of all these template classes.
       Just copy these */
    typedef NODE<NodeInfo> Node;
    typedef SSearchSpace<SStateSpace, NodeInfo, NODE> SearchSpace;
    typedef SStateSpace<Node, typename SearchSpace::StateIDLookup> StateSpace;

    typedef std::pair<int, size_t> UpdateElement;
    struct UpdateElementComparator {
        bool operator()(const UpdateElement &x, const UpdateElement &y) const {
            return x.first > y.first;
        }
    };
    typedef std::priority_queue<UpdateElement, std::vector<UpdateElement>, UpdateElementComparator> UpdateOrder;

    void traversal(UpdateOrder &order, NODE<NodeInfo> state);

    /* In step() the actual search is happening.
       When creating a new search algorithm, you must overwrite
       step(). This function is automatically called from ProbSearchEngine
       (parent class). */
    virtual SearchStatus step();
public:
    AcyclicValueIteration(const Options &opts);
    virtual ~AcyclicValueIteration() { }
    virtual void print_statistics() const {}
    void dump_states();
    void dump_table();
    virtual void print_options() {
        std::cout << "Running VI on acyclic state space..." << std::endl;
        ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options();
    }
};

#include "acyclic_value_iteration.cc"

#endif
