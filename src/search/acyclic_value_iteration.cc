
#ifdef ACYCLIC_VALUEITERATION_H

#include "acyclic_value_iteration.h"

#include "option_parser.h"

#include "globals.h"
#include "successor_generator.h"
#include "global_operator.h"
#include "task_proxy.h"

#include "timer.h"

#include <deque>
#include <limits>
#include <fstream>

using namespace std;

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
AcyclicValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::AcyclicValueIteration(const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>(opts)
{
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void AcyclicValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::traversal(UpdateOrder &order, NODE<NodeInfo> state)
{
    state.set_order_id(0);
    // state.is_goal() and state.is_dead() are automatically set
    // when creating state
    // state.is_dead_end() is only set, if state is not a goal,
    // and the state space reports the state to be a dead end
    if (state.is_goal() || state.is_dead_end()) {
        return;
    } else if (!this->generate_all_successors(state)) {
        // if generate_all_successors fails, then state does
        // not have any successor states
        // since state is not a goal state, state must
        // be a dead end
        // NOTE: if you mark a new state as dead end, you have
        // have to call value_initializer.dead_end(state)!
        state.mark_as_dead_end();
        this->value_initializer.dead_end(state);
        return;
    }
    state.close();
    int orderid = 0;
    // use the following to iterate over all applicable
    // operators of a state
    typename StateSpace::applicable_op_iterator op
        = this->state_space.applicable_operators(state);
    for (; !op.end(); op++) {
        // given an applicable operator, the following
        // is an iterator over the successor states
        // resulting from the different outcomes of a
        // (probabilistic) operator
        typename StateSpace::successor_iterator it
           = this->state_space.successors(state, op);
        for (; !it.end(); it++) {
            // *it returns the search node id
            // to get the underlying state data, use the following
            // function of the search space:
            Node succ = this->search_space.get_prob_state(*it);
            if (!succ.is_closed()) {
                traversal(order, succ);
            }
            if (succ.get_order_id() >= orderid) {
                orderid = succ.get_order_id() + 1;
            }
        }
    }
    state.set_order_id(orderid);
    order.push(std::make_pair(orderid, state.get_prob_state_id()));
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
SearchStatus AcyclicValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::step()
{
    Timer construction_t;
    construction_t.resume();

    // Generate complete state space, and maintain an order on the states
    UpdateOrder order;
    this->traversal(order, this->get_initial_state());

    construction_t.stop();
    cout << "State space has been constructed in " << construction_t << ". |S| = "
         << this->search_space.size() << endl;

    // Follow the topological order on the states to run updates.
    pair<float, int> newv;
    size_t num_updates = 0;
    while (!order.empty()) {
        size_t sid = order.top().second;
        order.pop();
        // given a search node id, the following function
        // constructs a Node object providing access to the
        // underlying state data
        Node p_state = this->search_space.get_prob_state(sid);
        num_updates++;
        // to update the value of a state, call this function
        // alternatively, to use tiebreaking, use
        // search_space.qval_update(p_state, newv, this->m_tiebreaking, epsilon)
        this->search_space.qval_update(p_state, newv);
    }

    cout << "Number of Belman-Updates: " << num_updates << endl;

    return SOLVED;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void AcyclicValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::dump_states()
{
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void AcyclicValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::dump_table()
{
}

#endif
