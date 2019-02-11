
#ifndef VALUE_INITIALIZER_H
#define VALUE_INITIALIZER_H

#include "optimization_criteria.h"
#include "globals.h"

template<int Type>
struct _InitValueWrapper {
    const float V_GOAL;
    const float V_GIVEUP;

    _InitValueWrapper() :
        V_GOAL(Type == OptimizationCriterion::MAXP ? 1 : 0),
        V_GIVEUP(Type == OptimizationCriterion::MAXP ? 0 : g_giveup)
    {
    }

    template<class State>
    inline void dead_end(State &state) const {
        state.set_v_current(V_GIVEUP, -1);
    }

    template<class State>
    inline void goal(State &state) const {
        state.set_v_current(V_GOAL, -1);
    }

    template<class State>
    inline void set_policy(State &state, int op) const {
        state.set_v_current(state.get_v_current(), op);
    }
};

template<int Type>
struct PessimisticValueInitializer : _InitValueWrapper<Type> {
    template<class State>
    inline void operator()(State &state) const {
        state.set_v_current(this->V_GIVEUP, -1);
    }
};

template<int Type>
struct OptimisticValueInitializer : _InitValueWrapper<Type> {
    template<class State>
    inline void operator()(State &state) const {
        state.set_v_current(this->V_GOAL, -1);
    }
};

template<int Type>
struct BiValueInitializer {
    const float V_GOAL;
    const float V_GIVEUP;

    BiValueInitializer() :
        V_GOAL(Type == OptimizationCriterion::MAXP ? 1 : 0),
        V_GIVEUP(Type == OptimizationCriterion::MAXP ? 0 : g_giveup)
    {
    }

    template<class State>
    inline void dead_end(State &state) const {
        state.set_v_current(V_GIVEUP, -1);
        state.set_v_optimistic(V_GIVEUP, -1);
    }

    template<class State>
    inline void goal(State &state) const {
        state.set_v_current(V_GOAL, -1);
        state.set_v_optimistic(V_GOAL, -1);
    }

    template<class State>
    inline void set_policy(State &state, int op) const {
        state.set_v_current(state.get_v_current(), op);
    }

    template<class State>
    inline void operator()(State &state) const {
        state.set_v_current(this->V_GIVEUP, -1);
        state.set_v_optimistic(this->V_GOAL, -1);
    }
};

#endif
