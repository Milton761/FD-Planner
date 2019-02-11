
#ifndef VALUE_REPORTER_H
#define VALUE_REPORTER_H

#include "optimization_criteria.h"

#include <cstdio>
#include <string>

constexpr const float EPSILON = 0.0005;

struct _SingleValueReporter {
    const float epsilon;
    const std::string format;
    float v;
    _SingleValueReporter(float epsilon, const std::string &format) : epsilon(epsilon), format(format), v(-1) {}
    template<class State>
    inline bool should_report(const State &state) const {
        return fabs(v - state.get_v_current()) > epsilon;
    }
    template<class State>
    inline void report(const State &state) {
        v = state.get_v_current();
        printf(format.c_str(), v);
    }
    template<class State>
    inline void report_final(const State &state) {
        v = state.get_v_current();
        printf("%.8f <= V*(i) <= %.8f", v, v);
    }
    template<class State>
    inline void operator()(const State &state) {
        report(state);
    }
};

template<int Type>
struct OptimisticValueReporter : public _SingleValueReporter {
    OptimisticValueReporter(float epsilon =  EPSILON) : _SingleValueReporter(epsilon, generate_format()) {}
    std::string generate_format() const {
        if (Type == OptimizationCriterion::MAXP) {
            return "0 <= V*(i) <= %.8f";
        } else {
            return "%.8f <= V*(i) <= inf";
        }
    }
};

template<int Type>
struct PessimisticValueReporter : public _SingleValueReporter {
    PessimisticValueReporter(float epsilon =  EPSILON) : _SingleValueReporter(epsilon, generate_format()) {}
    std::string generate_format() const {
        if (Type == OptimizationCriterion::MAXP) {
            return "%.8f <= V*(i) <= 1";
        } else {
            return "0 <= V*(i) <= %.8f";
        }
    }
};

template<int Type>
struct BiValueReporter {
    float vp = -1;
    float vo = -1;
    float epsilon = 0.001;
    BiValueReporter(float epsilon =  EPSILON)  : epsilon(epsilon) {}
    template<class State>
    inline bool should_report(const State &state) const {
        return (fabs(vp - state.get_v_current()) > epsilon ||
            fabs(vo - state.get_v_optimistic()) > epsilon);
    }
    template<class State>
    inline void report(const State &state) {
        vp = state.get_v_current();
        vo = state.get_v_optimistic();
        switch (Type) {
        case OptimizationCriterion::MAXP:
            printf("%.8f <= V*(i) <= %.8f", vp, vo);
            break;
        case OptimizationCriterion::EXPC:
            printf("%.8f <= V*(i) <= %.8f", vo, vp);
            break;
        }
    }
    template<class State>
    inline void report_final(const State &state) {
        report(state);
    }
    template<class State>
    inline void operator()(const State &state) {
        report(state);
    }
};

template<int Type>
struct LRTDPBiValueReporter {
    float vp = -1;
    float vo = -1;
    float epsilon = 0.001;
    template<class State>
    inline float get_pessimistic(const State &state) const {
        if (state.is_solved()) {
            return state.get_v_optimistic();
        }
        return state.get_v_current();
    }
    LRTDPBiValueReporter(float epsilon =  0.001)  : epsilon(epsilon) {}
    template<class State>
    inline bool should_report(const State &state) const {
        return (fabs(vp - get_pessimistic(state)) > epsilon ||
                fabs(vo - state.get_v_optimistic()) > epsilon);
    }
    template<class State>
    inline void report(const State &state) {
        vp = get_pessimistic(state);
        vo = state.get_v_optimistic();
        switch (Type) {
        case OptimizationCriterion::MAXP:
            printf("%.8f <= V*(i) <= %.8f", vp, vo);
            break;
        case OptimizationCriterion::EXPC:
            printf("%.8f <= V*(i) <= %.8f", vo, vp);
            break;
        }
    }
    template<class State>
    inline void report_final(const State &state) {
        report(state);
    }
    template<class State>
    inline void operator()(const State &state) {
        report(state);
    }
};

template<int Type>
struct FRETLRTDPBiValueReporter {
    float vp = -1;
    float vo = -1;
    float epsilon = 0.001;
    template<class State>
    inline float get_pessimistic(const State &state) const {
        if (state.is_solved() && state.is_marked()) {
            return state.get_v_optimistic();
        }
        return state.get_v_current();
    }
    FRETLRTDPBiValueReporter(float epsilon =  0.001)  : epsilon(epsilon) {}
    template<class State>
    inline bool should_report(const State &state) const {
        return (fabs(vp - get_pessimistic(state)) > epsilon ||
                fabs(vo - state.get_v_optimistic()) > epsilon);
    }
    template<class State>
    inline void report(const State &state) {
        vp = get_pessimistic(state);
        vo = state.get_v_optimistic();
        switch (Type) {
        case OptimizationCriterion::MAXP:
            printf("%.8f <= V*(i) <= %.8f", vp, vo);
            break;
        case OptimizationCriterion::EXPC:
            printf("%.8f <= V*(i) <= %.8f", vo, vp);
            break;
        }
    }
    template<class State>
    inline void report_final(const State &state) {
        report(state);
    }
    template<class State>
    inline void operator()(const State &state) {
        report(state);
    }
};

#endif

