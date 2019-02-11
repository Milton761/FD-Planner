#ifndef BLIND_SEARCH_HEURISTIC_H
#define BLIND_SEARCH_HEURISTIC_H

#include "heuristic.h"

class BlindSearchHeuristic : public Heuristic
{
    float min_operator_cost;
protected:
    virtual void initialize();
    virtual float compute_heuristic(const GlobalState &global_state);
public:
    BlindSearchHeuristic(const Options &options);
    ~BlindSearchHeuristic();
    virtual bool is_admissible() const {
        return true;
    }
};

#endif
