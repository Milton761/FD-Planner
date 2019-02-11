
#ifndef AO_VALUE_UPDATER
#define AO_VALUE_UPDATER

#include <cstdlib>
#include <queue>
#include <set>
#include <unordered_map>
#include <deque>

#include <limits>

#include <iostream>

#include "globals.h"
#include "segmented_vector.h"

struct AllParentsSelection
{
    template<typename State, typename SearchSpace, typename StateSpace>
    bool operator()(State &, State &, SearchSpace &, StateSpace &) const {
        return true;
    }
};

struct NoUpdateHandler
{
    template<typename State, typename SearchSpace, typename StateSpace>
    bool operator()(State &, SearchSpace &, StateSpace &) const {
        return false;
    }
};

struct LabelSolvedUpdateHandler
{
    template<typename State, typename SearchSpace, typename StateSpace>
    bool operator()(State &state, SearchSpace &search_space, StateSpace &state_space) const {
        bool unsolved = false;
        bool was_unsolved = state.is_marked();
        if (state.get_policy_operator_index() >= 0) {
            typename StateSpace::successor_iterator succ =
                state_space.successors(state, state.get_policy_operator_index());
            while (!succ.end()) {
                if (search_space.get_prob_state(*succ).is_marked()) {
                    unsolved = true;
                    break;
                }
                succ++;
            }
        }
        if (unsolved) {
            state.mark();
        } else {
            state.unmark();
        }
        return unsolved != was_unsolved;
    }
};

template<typename State, typename SearchSpace, typename StateSpace, typename ParentSelection, typename UpdateHandler, typename TieBreaking>
struct OrderedValueUpdater
{
    typedef std::unordered_map<size_t, size_t> UpdateOrder;
    typedef UpdateOrder::iterator UpdatePosition;
private:
    TieBreaking tie_breaking;
    SearchSpace &search_space;
    StateSpace &state_space;
    ParentSelection recur;
    UpdateHandler handler;
    const float epsilon;
    size_t get_order(State state, UpdateOrder &order)
    {
        size_t id = 0;
        size_t tmp;
        const std::set<size_t> &pars = state.get_all_parents();
        for (std::set<size_t>::iterator it = pars.begin(); it != pars.end(); it++) {
            State par = search_space.get_prob_state(*it);
            UpdatePosition i = order.find(*it);
            tmp = 0;
            if (i == order.end()) {
                if (!recur(par, state, search_space, state_space)) {
                    continue;
                }
                tmp = get_order(par, order);
            } else {
                tmp = i->second;
            }
            if (tmp >= id) {
                id = tmp + 1;
            }
        }
        order[state.get_prob_state_id()] = id;
        return id;
    }
public:
    OrderedValueUpdater(SearchSpace &search_space, StateSpace &state_space, float epsilon)
        : search_space(search_space), state_space(state_space), epsilon(epsilon) {}
    void operator()(State &state, int upto)
    {
        size_t start_id = state.get_prob_state_id();
        UpdateOrder order_mapping;
        get_order(state, order_mapping);
        std::priority_queue<std::pair<size_t, size_t> > update_order;
        update_order.push(std::make_pair(order_mapping[start_id], start_id));
        order_mapping.erase(order_mapping.find(start_id));
        float diff;
        std::pair<float, int> newv;
        while (!update_order.empty()) {
            size_t sid = update_order.top().second;
            State state2 = search_space.get_prob_state(sid);
            update_order.pop();
            if (upto >= 0 && (unsigned) upto == sid) {
                upto = -1;
            }
            assert(state.is_closed() || state.is_dead_end());
            diff = g_epsilon + 1;
            if (!state2.is_dead_end() && !state2.is_goal()) {
                diff = search_space.qval_update(state2, newv, tie_breaking, epsilon);
            }
            if (handler(state2, search_space, state_space) || upto >= 0 || diff > g_epsilon) {
                const std::set<size_t> &pars = state2.get_all_parents();
                for (std::set<size_t>::iterator it = pars.begin(); it != pars.end(); it++) {
                    UpdatePosition pos = order_mapping.find(*it);
                    if (pos != order_mapping.end()) {
                        update_order.push(std::make_pair(pos->second, pos->first));
                        order_mapping.erase(pos);
                    }
                }
            }
        }
    }
};

#endif

