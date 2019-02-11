#ifndef MAS_VALUE_ITERATION_H
#define MAS_VALUE_ITERATION_H

#include "../../search_engine.h"
#include "../merge_strategy.h"

#include <vector>
#include <memory>

class Options;

class MASValueIteration : public SearchEngine
{
protected:
    bool hide_mas_output;
    std::shared_ptr<MergeStrategy> merge_strategy;
    std::vector<std::vector<std::pair<int, std::vector<int> > > > state_space;
    std::vector<float> hvals;
    std::vector<bool> goals;
    int initial_state;
    int dead_ends_id;
    virtual void initialize();
    virtual SearchStatus step();
public:
    MASValueIteration(const Options &opts);
};

#endif
