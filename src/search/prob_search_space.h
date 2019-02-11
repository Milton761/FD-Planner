#ifndef PROB_SEARCH_SPACE_H
#define PROB_SEARCH_SPACE_H

#include "optimization_criteria.h"
#include "segmented_vector.h"

#include <set>
#include <vector>
#include <unordered_map>
#include <string>

class GlobalOperator;
class GlobalState;

struct OptimisticV {
    template<class State>
    inline float operator()(const State &state) const {
        return state.get_v_current();
    }
};

struct BiOptimisticV {
    template<class State>
    inline float operator()(const State &state) const {
        return state.get_v_optimistic();
    }
};

template<template<typename> class State, typename Info>
struct SingleVState : public State<Info> {
    SingleVState(size_t prob_state_id_, Info &info_, unsigned currentstep)
        : State<Info>(prob_state_id_, info_, currentstep) {}
};

template<template<typename> class State, typename Info>
struct UpperAndLowerVState : public State<Info> {
    UpperAndLowerVState(size_t prob_state_id_, Info &info_, unsigned currentstep)
        : State<Info>(prob_state_id_, info_, currentstep) {}
    void set_v_optimistic(float v, int op) {
        this->info.v_optimistic = v;
        this->info.policy = op;
    }
    float get_v_optimistic() const {
        return this->info.v_optimistic;
    }
};

template<typename NodeInfo>
class BasicState
{
protected:
    size_t prob_state_id;
    NodeInfo &info;
    unsigned currentstep;
public:

    BasicState(size_t prob_state_id_, NodeInfo &info_, unsigned currentstep);
    virtual ~BasicState() {}

    size_t get_prob_state_id() const {
        return prob_state_id;
    }

    int get_global_state_id() const;
    //GlobalState get_global_state() const;

    void set_budget(int budget);
    int get_budget() const;

    void set_v_current(float v, int op);
    float get_v_current() const;
    void set_policy_operator_index(int op);
    int get_policy_operator_index() const;
    //int get_policy_operator() const;

    //void add_applicable_operator(int op);
    //std::vector<std::vector<size_t> > &get_all_successors();
    //const std::vector<std::vector<size_t> > &get_all_successors() const;

    bool add_parent(size_t parent);
    const std::set<size_t> &get_all_parents() const;
    std::set<size_t> &get_all_parents();

    //const std::vector<int> &get_applicable_operators() const;

    bool is_new() const;
    bool is_opened() const;
    bool is_closed() const;
    bool is_dead_end() const;

    void set_open();
    void close();

    void set_solved();
    bool is_solved() const;

    void mark_as_dead_end();

    void mark();
    void unmark();
    bool is_marked() const;

    void set_goal();
    bool is_goal() const;

    void dump_status(std::ostream &out = std::cout) const;
    std::string status() const;
};

template<typename StateInfo>
struct HState : public BasicState<StateInfo>
{
    HState(size_t id, StateInfo &info, unsigned i) : BasicState<StateInfo>(id, info, i) {}
    void set_estimated_distance(float h) {
        this->info.h = h;
    }
    float get_estimated_distance() const {
        return this->info.h;
    }
};

template<template<typename> class State, typename Info>
struct PreferredOpState : public State<Info> {
    PreferredOpState(size_t id, Info &info, unsigned u) : State<Info>(id, info, u) {}
    std::set<const GlobalOperator*> &get_preferred_outcomes() {
        return this->info.preferred_outcomes;
    }
    const std::set<const GlobalOperator*> &get_preferred_outcomes() const {
        return this->info.preferred_outcomes;
    }
};

template<template<typename> class State, typename Info>
struct DepthState : public State<Info> {
    DepthState(size_t id, Info &info, unsigned u) : State<Info>(id, info, u) {}
    int get_depth() const {
        return this->info.depth;
    }
    bool set_depth(unsigned d) {
        if (d > this->info.depth) {
            this->info.depth = d;
            return true;
        }
        return false;
    }
};

template<template<typename> class State, typename Info>
struct LargelinkState : public State<Info> {
    LargelinkState(size_t id, Info &info, unsigned u) : State<Info>(id, info, u) {}
    unsigned get_largelink() const {
        return this->info.largelink;
    }
    void set_largelink(int largelink) {
        this->info.largelink = largelink;
    }
    bool is_flagged() const {
        return this->info.flag == 1;
    }
    void flag(bool flag = true) {
        this->info.flag = flag ? 1 : 0;
    }
    void unflag() {
        this->info.flag = 0;
    }
};


template<template<typename> class State, typename Info>
struct TarjanState : public State<Info> {
    TarjanState(size_t id, Info &info, unsigned u) : State<Info>(id, info, u) {}
    unsigned get_largelink() const {
        return this->info.largelink;
    }
    void set_largelink(int largelink) {
        this->info.largelink = largelink;
    }
    bool is_flagged() const {
        return this->info.flag == 1;
    }
    void flag(bool flag = true) {
        this->info.flag = flag ? 1 : 0;
    }
    void unflag() {
        this->info.flag = 0;
    }
    void set_index(int index) {
        this->info.index = index;
    }
    unsigned get_index() const {
        return this->info.index;
    }
    bool visited() const {
        return this->info.index >= 0;
    }
    unsigned get_lowlink() const {
        return this->info.lowlink;
    }
    void set_lowlink(int x) {
        this->info.lowlink = x;
    }
    void update_lowlink(unsigned index) {
        if (index < get_lowlink()) {
            this->info.lowlink = index;
        }
    }
    bool is_onstack() const {
        return this->info.onstack;
    }
    void set_onstack(bool x = true) {
        this->info.onstack = x;
    }
    void clear() {
        this->info.onstack = 0;
        this->info.flag = 0;
        this->info.index = -1;
    }
};

template<typename Info>
struct DepthHState : public DepthState<HState, Info> {
    DepthHState(size_t id, Info &info, unsigned u) : DepthState<HState, Info>(id, info, u) {}
};

template<typename Info>
struct PreferredState : public PreferredOpState<BasicState, Info> {
    PreferredState(size_t z, Info &x, unsigned u) : PreferredOpState<BasicState, Info>(z, x, u) {}
};

template<typename Info>
struct PreferredHState : public PreferredOpState<HState, Info> {
    PreferredHState(size_t z, Info &x, unsigned u) : PreferredOpState<HState, Info>(z, x, u) {}
};

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
class ProbSearchSpace
{
public:
    struct StateIDLookup
    {
    private:
        ProbSearchSpace &sp;
    public:
        StateIDLookup(ProbSearchSpace &sp) : sp(sp) {}
        inline size_t lookup_id(int state, int budget) {
            return sp.lookup_id(state, budget);
        }
        inline int prob_to_global(size_t p_id) {
            return sp.get_prob_state(p_id).get_global_state_id();
        }
        size_t size() const {
            return sp.size();
        }
    };

    typedef StateSpace<Node<NodeData>, StateIDLookup> t_state_space;
    typedef Node<NodeData> t_state;
    typedef QVal<ProbSearchSpace, t_state> t_qval;
protected:
    SegmentedVector<std::unordered_map<int, size_t> > lookup;
    SegmentedVector<NodeData> data;
    size_t current_id;

    StateIDLookup _lookup;
    t_qval _qval;
    t_state_space &state_space;
    size_t num_updates;
    size_t num_updates_initial_state;

    unsigned solved_step;
public:
    ProbSearchSpace(StateSpace<Node<NodeData>, StateIDLookup> &state_space);
    virtual ~ProbSearchSpace();

    unsigned get_solved_step() const {
        return solved_step;
    }

    void increase_solved_step() {
        solved_step++;
    }

    inline Node<NodeData> get_prob_state(const GlobalState &state, int budget);
    inline Node<NodeData> get_prob_state(const unsigned &state_id, int budget);
    inline Node<NodeData> get_prob_state(size_t state_id);
    inline Node<NodeData> operator[](size_t state_id);
    size_t lookup_id(const unsigned &state, int budget);
    std::size_t size() const;

    //std::pair<float, int> (*qval) (const Node<NodeData> &state);

    inline float qvals(const Node<NodeData> &state, typename t_state_space::applicable_op_iterator op) {
        return _qval.compute_value(state, op);
    }
    int qval(const Node<NodeData> &state, std::pair<float, int> &newv);
    float qval_update(Node<NodeData> &state,
                               std::pair<float, int> &newv);
    void qval_lookup(const Node<NodeData> &state,
            std::vector<int> &ops, float delta = 0);

    template<typename TieBreaking>
    void qval(const Node<NodeData> &state, std::pair<float, int> &newv,
            TieBreaking &tie_breaking, float delta);
    template<typename TieBreaking>
    float qval_update(Node<NodeData> &state,
                               std::pair<float, int> &newv,
                               TieBreaking &tie_breaking, float delta);
    template<typename TieBreaking>
    float qval_lookup(Node<NodeData> &state,
                               std::pair<float, int> &newv,
                               TieBreaking &tie_breaking, float delta);
    /*
    template<typename TieBreaking>
    float qval_update_prune(Node<NodeData> &state,
                               std::pair<float, int> &newv,
                               TieBreaking &tie_breaking, float delta);
   */

    bool qval_is_worse(Node<NodeData> &state, float newval, float epsilon = 0) {
        return _qval.compare(state, newval, epsilon);
    }

    float get_value(Node<NodeData> &state)
    {
        return this->_qval.get_value(state);
    }

    bool is_goal(const Node<NodeData> &node) const;

    void dump(std::ostream &out = std::cout);
    void dump_policy_graph(size_t sid = 0, std::ostream &out = std::cout);
    void dump_policy_graph2(size_t sid = 0, std::ostream &out = std::cout);
    void dump_policy_graph3(size_t sid = 0, std::ostream &out = std::cout);
    void dump_policy_graph4(size_t sid = 0, std::ostream &out = std::cout);

    t_state_space &get_underlying_state_space()
    {
        return state_space;
    }

    const t_state_space &get_underlying_state_space() const
    {
        return state_space;
    }

    size_t get_updates_init() const {
        return num_updates_initial_state;
    }

    size_t get_updates() const {
        return num_updates;
    }

    size_t ASD;
    size_t DSA;
};

template<template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
struct ProbSearchSpaceMin : public ProbSearchSpace<QValMin, StateSpace, NodeData, Node>
{
    ProbSearchSpaceMin(typename ProbSearchSpace<QValMin, StateSpace, NodeData, Node>::t_state_space &state_space)
        : ProbSearchSpace<QValMin, StateSpace, NodeData, Node>(state_space) {}
};

template<template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
struct ProbSearchSpaceMax : public ProbSearchSpace<QValMax, StateSpace, NodeData, Node>
{
    ProbSearchSpaceMax(typename ProbSearchSpace<QValMax, StateSpace, NodeData, Node>::t_state_space &state_space)
        : ProbSearchSpace<QValMax, StateSpace, NodeData, Node>(state_space) {}
};

template<template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
struct ProbSearchSpaceDualMax : public ProbSearchSpace<DualQValMax, StateSpace, NodeData, Node>
{
    ProbSearchSpaceDualMax(typename ProbSearchSpace<DualQValMax, StateSpace, NodeData, Node>::t_state_space &state_space)
        : ProbSearchSpace<DualQValMax, StateSpace, NodeData, Node>(state_space) {}
};

template<template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
struct ProbSearchSpaceDualMin : public ProbSearchSpace<DualQValMin, StateSpace, NodeData, Node>
{
    ProbSearchSpaceDualMin(typename ProbSearchSpace<DualQValMin, StateSpace, NodeData, Node>::t_state_space &state_space)
        : ProbSearchSpace<DualQValMin, StateSpace, NodeData, Node>(state_space) {}
};



#include "prob_search_space.cc"

#endif
