
#ifndef PROB_SEARCH_ENGINE_H
#define PROB_SEARCH_ENGINE_H

#include "search_engine.h"
#include "optimization_criteria.h"
#include "prob_search_space.h"
#include "timer.h"

#include "option_parser.h"
#include "ff_heuristic.h"
#include "evaluation_context.h"


class Option;
class OptionParser;
class GlobalOperator;
class CountdownTimer;


template<typename State>
struct DoNothingOnNewSuccessor {
public:
    template<typename SearchSpace, typename StateSpace>
    inline void operator()(SearchSpace &, StateSpace &, const State *,
                           const State &, const GlobalOperator *, float) const {}
};

struct DoNothingOnSuccessor {
public:
    template<typename State, typename SearchSpace, typename StateSpace>
    inline void operator()(SearchSpace &, StateSpace &, const State &, State &,
                           const GlobalOperator *, bool) const {}
};

template<typename State>
struct StoreHOnNewSuccessor {
public:
    template<typename SearchSpace, typename StateSpace>
    inline void operator()(SearchSpace &, StateSpace &, const State *, State &succ,
                           const GlobalOperator *, float h) const {
        succ.set_estimated_distance(h);
    }
};

template<typename State, template<typename> class Do>
struct StorePreferredOperators {
private:
    Heuristic *preferred;
    Do<State> do_;
public:
    StorePreferredOperators() {
        Options opts;
        opts.set<int>("cost_type", 0);
        preferred = new FFHeuristic(opts);
    }
    ~StorePreferredOperators() {
        delete preferred;
    }
    template<typename SearchSpace, typename StateSpace>
    void operator()(SearchSpace &search_space, StateSpace &state_space,
                    const State *pa, State &state, const GlobalOperator *op, float h) {
        do_(search_space, state_space, pa, state, op, h);
        if (!state.is_opened()) {
            return;
        }
        bool x;
        state_space.compute_goal_estimation(*preferred, state, x,
                                            state.get_preferred_outcomes());
    }
};

template<typename State>
struct StorePref : public
        StorePreferredOperators<State, DoNothingOnNewSuccessor> { };

template<typename State>
struct StorePrefH : public
        StorePreferredOperators<State, StoreHOnNewSuccessor> { };



/*
    NodeInfo         -- per state data; stored in search space
    Node             -- template is instantiated to Node<NodeInfo>; is a wrapper class
                        to access a state's data; is created by search space
    SSearchSpace     -- template is instantiated to SSearchSpace<SStateSpace, NodeInfo, Node>;
                        it stores for each state in the state space additional
                        data, as defined by NodeInfo; provides interfaces to access
                        this data
    SStateSpace      -- template is instantiated to SStateSpace<Node<NodeInfo>, SearchSpace::StateIDLookup>;
                        defines the underlying state space on which the search
                        algorithm is ran; SearchSpace::StateIDLookup provides an
                        interfaces to get the state id as defined by the state space
                        from a Node<NodeInfo> object; the state space provides an
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
template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
class ProbSearchEngine : public SearchEngine
{
private:
    typedef Node<NodeInfo> State;
    typedef SSearchSpace<SStateSpace, NodeInfo, Node> SearchSpace;
    typedef SStateSpace<State, typename SearchSpace::StateIDLookup> StateSpace;
    bool initialized;
protected:
    const float m_epsilon;
    TieBreaking m_tiebreaking;

    struct OnNewSuccessorDoNothing {
    private:
        static OnNewSuccessorDoNothing *inst;
    public:
        template<typename SearchSpace, typename StateSpace>
        inline void operator()(SearchSpace &, StateSpace &, const Node<NodeInfo> *,
                               const Node<NodeInfo> &, const GlobalOperator *, float) const {}
        static OnNewSuccessorDoNothing &instance() {
            return *inst;
        }
    };
    struct OnSuccessorDoNothing {
    private:
        static OnSuccessorDoNothing *inst;
    public:
        template<typename _State, typename _SearchSpace, typename _StateSpace>
        inline void operator()(_SearchSpace &, _StateSpace &, const _State &, _State &,
                               const GlobalOperator *, bool)
        const {}
        static OnSuccessorDoNothing &instance() {
            return *inst;
        }
    };
public:
    struct ProgressReporter {
        ValueReporter &value_reporter;
        Timer &_t;
        ProgressReporter(ValueReporter &vrp, Timer &_t) : value_reporter(vrp), _t(_t) {}
        void operator()(SearchSpace &search_space, size_t sid,
                         bool force_print = false) {
            Node<NodeInfo> init = search_space.get_prob_state(sid);
            if (force_print || value_reporter.should_report(init)) {
                cout << "[";
                value_reporter(init);
                printf(", |S| = %d, U = %d, t = %.4fs]\n",
                       (int) search_space.size(),
                       (int) search_space.get_updates_init(),
                       _t());
            }
        }
    };
private:
    Timer _t;
    float _last_printed;
protected:
    ValueInitializer &value_initializer;
    ValueReporter &value_reporter;
    ProgressReporter progress_reporter;

    float initialize_value(Node<NodeInfo> &state);

    StateSpace &state_space;
    SearchSpace &search_space;
    // You can overwrite initialize() to initialize your own data structures.
    // However, when you overwrite it, you have to call
    // ProbSearchEngine::initialize().
    virtual void initialize();

    size_t _initial_state_id;

    ProbSearchEngine(
        ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
        *copy);
public:
    /* Inside your search engine, you can use the following methods to generate the
       successor states of a new (not yet expanded) state */
    /* These functions will return false, if the state does not have any successor
       state */
    /* If update is set to true, will immediately perform a QVal update after finishing
       the expansion */
    /* C1 and C2 are callbacks where
        C1 will be called for each successor. It is assumed that C1 is a function
        (or it implements operator()) while reading the arguments
            SearchSpace             -- the search space
            StateSpace              -- the state space
            State t                 -- successor state
            State s                 -- state that is expanded
            const GlobalOperator *  -- action outcome that led from s to t
            bool                    -- true if no other action outcome seen so far
                                       led from s to t; false otherwise
        C2 will be called for each successor state that has been newly created, i.e.,
        a state that has not been seen so far. Again, C2 must be a function or a
        struct that implements operator(), while reading the arguments
            SearchSpace             -- the search space
            StateSpace              -- the state space
            State t                 -- successor state
            State s                 -- state that is expanded
            const GlobalOperator *  -- action outcome that led from s to t
            float                   -- estimated determernistic distance to a goal state,
                                       or 0 if no deterministic heuristic is used
    */
    template<typename C1, typename C2>
    bool generate_all_successors(Node<NodeInfo> &node, C1 &c1, C2 &c2, bool update = false);
    /* If you do not need these callbacks, you can call one of the following three methods: */
    template<typename C>
    bool generate_all_successors(Node<NodeInfo> &node, C &c, bool update = false) {
        return generate_all_successors(node, OnSuccessorDoNothing::instance(), c, update);
    }
    template<typename C>
    bool generate_all_successors_new(Node<NodeInfo> &node, C &c, bool update = false) {
        return generate_all_successors(node, c, OnNewSuccessorDoNothing::instance(), update);
    }
    bool generate_all_successors(Node<NodeInfo> &node, bool update = false) {
        return generate_all_successors(node, OnSuccessorDoNothing::instance(),
                                       OnNewSuccessorDoNothing::instance(), update);
    }
    ValueInitializer &get_vinit() {
        return value_initializer;
    }
public:
    // This function is called from the planner. Internally search() calls step() to
    // do the actual search all new probabilistic search engines have to overwrite step().
    // DO NOT OVERWRITE search()
    virtual void search();
    void search_internal(CountdownTimer *t);

    // Overwrite this function, to perform the actual search. Return
    //      IN_PROGRESS -- if search is not completed, and step() should be called again
    //      SUCCESSFUL  -- if search is completed
    // Examples of step():
    //  VI: (1) Construct entire state space, (2) Run updates on the states
    //          of the state space. When the values of all states have
    //          converged, then V* has been computed for all states; and
    //          SUCCESSFUL is returned. Thus, here, step() is only called once.
    //  LRTDP: Each step() runs one trial. If the initial state is not labeled solved
    //         after the trial, IN_PROGRESS is returned. If the initial state is labeled
    //         solved, SUCESSFUL is returned.
    virtual SearchStatus step() = 0;

    ProbSearchEngine(const Options &opts);
    virtual ~ProbSearchEngine() {}
    virtual void print_statistics() const {}
    void report_progress(SearchSpace &search_space,
                         size_t initial_state, bool force_print = false);
    SearchSpace &get_search_space() {
        return search_space;
    }
    StateSpace &get_state_space() {
        return state_space;
    }

    size_t get_initial_state_id() const {
        return _initial_state_id;
    }
    Node<NodeInfo> get_initial_state() {
        return search_space.get_prob_state(_initial_state_id);
    }
    virtual void print_options();

    virtual void extract_policy(std::ostream &out = std::cout);
protected:
    bool m_sub_engine;
};

namespace prob_search_engine
{
static void add_options_to_parser(OptionParser &parser);
}

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
typename ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnNewSuccessorDoNothing
*ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnNewSuccessorDoNothing::inst
    = new ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnNewSuccessorDoNothing();

template<typename NodeInfo, template<typename> class Node, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
typename ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnSuccessorDoNothing
*ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnSuccessorDoNothing::inst
    = new ProbSearchEngine<NodeInfo, Node, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::OnSuccessorDoNothing();

#include "prob_search_engine.cc"

#endif

