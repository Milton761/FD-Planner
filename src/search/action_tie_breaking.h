
#ifndef ACTION_TIE_BREAKING_H
#define ACTION_TIE_BREAKING_H

#include "globals.h"
#include "template_factory.h"
#include <vector>
#include <limits>
#include <string>
#include <iostream>

namespace action_tie_breaking {
    enum Type {
        ARBITRARY = 0,
        MINEXPH = 1,
        MINEXPGAP = 2,
        MAXEXGAP = 3,
        PREFERRED = 4,
    };
    void get_options(std::vector<std::string> &opts);
}

struct TieBreakingArbitrary {
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &, const std::vector<int> &candidates, SearchSpace &, StateSpace &) const {
        return candidates[0];
    }
};

struct TieBreakingMinExpH {
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &state, const std::vector<int> &candidates, SearchSpace &search_space, StateSpace &state_space) const {
        float best = std::numeric_limits<float>::max();
        float tmp;
        int op = 0;
        for (size_t i = 0; i < candidates.size(); i++) {
            tmp = 0;
            typename StateSpace::successor_iterator it = state_space.successors(state, candidates[i]);
            while (!it.end()) {
                State succ = search_space.get_prob_state(*it);
                tmp += it.prob() * succ.get_estimated_distance();
                it++;
            }
            if (tmp < best) {
                best = tmp;
                op = candidates[i];
            }
        }
        return op;
    }
};

struct TieBreakingMinExpGap {
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &state, const std::vector<int> &candidates, SearchSpace &search_space, StateSpace &state_space) const {
        float best = std::numeric_limits<float>::max();
        float tmp;
        int op = 0;
        for (size_t i = 0; i < candidates.size(); i++) {
            tmp = 0;
            typename StateSpace::successor_iterator it = state_space.successors(state, candidates[i]);
            while (!it.end()) {
                State succ = search_space.get_prob_state(*it);
                tmp += it.prob() * fabs(state.get_v_current() - state.get_v_optimistic());
                it++;
            }
            if (tmp < best) {
                best = tmp;
                op = candidates[i];
            }
        }
        return op;
    }
};

struct TieBreakingMaxExpGap {
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &state, const std::vector<int> &candidates, SearchSpace &search_space, StateSpace &state_space) const {
        float best = 0;
        float tmp;
        int op = 0;
        for (size_t i = 0; i < candidates.size(); i++) {
            tmp = 0;
            typename StateSpace::successor_iterator it = state_space.successors(state, candidates[i]);
            while (!it.end()) {
                State succ = search_space.get_prob_state(*it);
                tmp += it.prob() * fabs(state.get_v_current() - state.get_v_optimistic());
                it++;
            }
            if (tmp > best) {
                best = tmp;
                op = candidates[i];
            }
        }
        return op;
    }
};

struct TieBreakingMaxExpV {
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &state, const std::vector<int> &candidates, SearchSpace &search_space, StateSpace &state_space) const {
        float best = 0;
        float tmp;
        int op = 0;
        for (size_t i = 0; i < candidates.size(); i++) {
            tmp = 0;
            typename StateSpace::successor_iterator it = state_space.successors(state, candidates[i]);
            while (!it.end()) {
                State succ = search_space.get_prob_state(*it);
                tmp += it.prob() * state.get_v_current();
                it++;
            }
            if (tmp > best) {
                best = tmp;
                op = candidates[i];
            }
        }
        return op;
    }
};

template<typename SecondTieBreaking>
struct TieBreakingPreferred {
    SecondTieBreaking tie_breaking;
    template<typename State, typename SearchSpace, typename StateSpace>
    int operator()(State &state, const std::vector<int> &candidates, SearchSpace &search_space, StateSpace &state_space) const {
        std::vector<int> filtered;
        filtered.reserve(candidates.size());
        for (size_t i = 0; i < candidates.size(); i++) {
            bool is_pref = false;
            size_t lr = 0;
            typename StateSpace::successor_iterator it = state_space.successors(state, candidates[i]);
            while (!it.end()) {
                if (state.get_preferred_outcomes().count(&g_operators[g_prob_operators[candidates[i]][lr].second]) > 0) {
                    is_pref = true;
                    break;
                }
                lr++;
                it++;
            }
            if (is_pref) {
                filtered.push_back(candidates[i]);
            }
        }
        if (filtered.size() == 1) {
            return filtered[0];
        }
        else if (filtered.size() > 1) {
            return tie_breaking(state, filtered, search_space, state_space);
        } else {
            return tie_breaking(state, candidates, search_space, state_space);
        }
    }
};

struct TieBreakingPreferredArbitrary
: public TieBreakingPreferred<TieBreakingArbitrary> { };

#endif
