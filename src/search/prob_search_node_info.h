
#ifndef PROB_SEARCH_NODE_INFO_H
#define PROB_SEARCH_NODE_INFO_H

#include <set>
#include <vector>

#include "global_operator.h"

template<typename Base>
struct UpperAndLowerVStateInfo : public Base {
    float v_optimistic;
    UpperAndLowerVStateInfo(int sid, int budget)
        : Base(sid, budget), v_optimistic(-1) {}
};

template<typename Base>
struct SingleVStateInfo : public Base {
    SingleVStateInfo(int sid, int budget)
        : Base(sid, budget) {}
};

struct BasicStateInfo {
    enum Status {
        NEW = 0,
        OPEN = 1,
        CLOSED = 2,
        DEAD_END = 3,
        GOAL = 4,
    };

    /* Global state information */
    int stateid;

    /* Remaining budget */
    int budget;

    /* Status */
    unsigned int status : 31;
    unsigned int marked : 1;
    unsigned solved;

    /* Value functions */
    float v_current;

    /* pointer to parent ProbSearchState */
    std::set<std::size_t> parents;

    int policy;

    BasicStateInfo(int sid, int budget)
        : stateid(sid), budget(budget), status(NEW), marked(0), solved(0),
          v_current(0), policy(-1) {
    }
};

struct HStateInfo : public BasicStateInfo {
    float h;
    HStateInfo(int sid, int budget) : BasicStateInfo(sid, budget), h(-1) {}
};

template<typename Info>
struct PreferredOpStateInfo : public Info {
    std::set<const GlobalOperator*> preferred_outcomes;
    PreferredOpStateInfo(int sid, int budget) : Info(sid, budget) {}
};

template<typename Info>
struct DepthStateInfo : public Info {
    unsigned depth;
    DepthStateInfo(int sid, int budget) : Info(sid, budget), depth(0) {}
};

struct PreferredStateInfo : public PreferredOpStateInfo<BasicStateInfo>
{

    PreferredStateInfo(int sid, int budget)
        : PreferredOpStateInfo<BasicStateInfo>(sid, budget) {}
};

struct PreferredHStateInfo : public PreferredOpStateInfo<HStateInfo>
{
    PreferredHStateInfo(int sid, int budget)
        : PreferredOpStateInfo<HStateInfo>(sid, budget) {}
};


template<typename Info>
struct LargelinkStateInfo : public Info {
    unsigned largelink : 31;
    unsigned flag : 1;
    LargelinkStateInfo(int sid, int budget) : Info(sid, budget), largelink(0), flag(0) {
    }
};


template<typename Info>
struct TarjanStateInfo : public Info {
    int index;
    unsigned lowlink : 31;
    unsigned onstack : 1;
    unsigned flag : 1;
    unsigned largelink : 31;
    TarjanStateInfo(int sid, int budget)
        : Info(sid, budget), index(-1), lowlink(0), onstack(0), flag(0), largelink(0) {
    }
};


//struct ProbSearchStateInfo : public BasicStateInfo
//{
//    /* pointer to successor ProbSearchStates */
//    std::vector<std::vector<std::size_t> > successors;
//    /* which operators were applicable and lead to which successor vector */
//    std::vector<unsigned> operators;
//    ProbSearchStateInfo(int sid, int budget) : BasicStateInfo(sid, budget) {}
//};

#endif

