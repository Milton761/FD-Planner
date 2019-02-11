#ifndef FRET_H
#define FRET_H

#include "prob_search_engine.h"

#include "segmented_vector.h"

#include "option_parser.h"

#include "lrtdp.h"

#include <deque>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "timer.h"

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
struct FRET : public
        ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking> {
    typedef NODE<NodeInfo> Node;
    typedef SSearchSpace<SStateSpace, NodeInfo, NODE> SearchSpace;
    typedef SStateSpace<Node, typename SearchSpace::StateIDLookup> StateSpace;

    struct FRETData {
        constexpr static const size_t UNDEFINED = -1;
        size_t index;
        size_t lowlink;
        bool onstack;
        bool leaf;
#if _DEBUG_SCC
        std::vector<int> ops;
#endif
        FRETData() : index(UNDEFINED), lowlink(UNDEFINED), onstack(false), leaf(true) {}
    };

    struct OpenElem {
        size_t stateid;
        std::vector<int> ops;
        size_t op_index;
        typename StateSpace::successor_iterator it;
        bool interrupted;
        OpenElem(size_t state) : stateid(state), op_index(0), interrupted(false) {}
        OpenElem(size_t state, int op) : stateid(state), op_index(0),
            interrupted(false) {
            ops.push_back(op);
        }
        int next_op() {
            if (op_index >= ops.size()) {
                return -1;
            }
            return ops[op_index++];
        }
    };

    struct PolicyConstruct {
        OpenElem *operator()(Node node) {
            return new OpenElem(node.get_prob_state_id(), node.get_policy_operator_index());
        }
        bool requires_update() const {
            return false;
        }
    };

    struct ValueConstruct {
    private:
        SearchSpace &search_space;
        const float delta;
    public:
        ValueConstruct(SearchSpace &search_space,
                       float delta) : search_space(search_space), delta(delta) {}
        OpenElem *operator()(Node node) {
            OpenElem *res = new OpenElem(node.get_prob_state_id());
            search_space.qval_lookup(node, res->ops, delta);
            return res;
        }
        bool requires_update() const {
            /* Need to (re-) update here because
               1. the operators are extracted by recomputing the Q value for
                  each possible action so that we don't have to store all
                  candidates
               2. the value might have not been converged since it is only
                  guaranteed that the values in ONE greedy policy graph are
                  converged, while we are considering all possible greedy policy
                  graphs
               */
            return true;
        }
    };

    size_t num_iters;
    size_t size_gv;
    size_t size_gv_leaf;
protected:
    TieBreaking tie_breaking;
    ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
    *engine;
    const bool local_trap_elimination;
    const float delta;
    const float epsilon;

    PolicyConstruct pconstruct;
    ValueConstruct vconstruct;

    Timer _t0;
    Timer _t1;
    Timer _t2;
    Timer _t3;
    Timer _t4;
    Timer _t5;

    bool unit_sccs;
    size_t index;
    float leaf_diff;
    bool extended_search_space;
    std::deque<size_t> stack;
    std::deque<OpenElem*> queue;

    //FRETData &lookup(SegmentedVector<FRETData> &data, size_t s) {
    //    if (s)
    //}

    void apply_abstraction(StateSpace &state_space, const std::set<size_t> &scc,
                           size_t repr);
    void handle_scc(SegmentedVector<FRETData> &data, Node state);
    void update_successors(StateSpace &state_space, Node state,
                           std::unordered_map < unsigned,
                           std::unordered_set<std::vector<size_t> > > &new_edges);
    template<typename F>
    void eliminate_traps(SegmentedVector<FRETData> &data, size_t state, F &construct);
    size_t proceed(SegmentedVector<FRETData> &data, OpenElem &elem, bool &changed);

    virtual SearchStatus step();

    virtual void print_statistics();
public:
    FRET(const Options &opts);
    virtual void print_options();

    virtual void extract_policy(std::ostream &out);
};

#include "fret.cc"

#endif
