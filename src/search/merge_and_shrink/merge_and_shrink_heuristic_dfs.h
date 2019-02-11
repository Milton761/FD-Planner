#ifndef MERGE_AND_SHRINK_MERGE_AND_SHRINK_HEURISTIC_DFS_H
#define MERGE_AND_SHRINK_MERGE_AND_SHRINK_HEURISTIC_DFS_H

#include "../heuristic.h"

#include <memory>
#include <vector>
#include <set>

class Labels;
class MergeLinearExtended;
class ShrinkStrategy;
class Timer;
class TransitionSystem;
class FactoredTransitionSystem;

class MergeAndShrinkHeuristicDFS : public Heuristic
{
    std::vector<TransitionSystem *> transition_systems;
    int num_vars;

    std::shared_ptr<MergeLinearExtended> merge_strategy;
    std::shared_ptr<ShrinkStrategy> shrink_strategy;
    std::shared_ptr<Labels> labels;
    long starting_peak_memory;

    int max_branches;

    /*
      TODO: after splitting transition system into several parts, we may
      want to change all transition system pointers into unique_ptr.
    */
    bool build_transition_system_dfs(TransitionSystem *system,
            std::set<std::set<int> > &explored_varsets,
            std::vector<TransitionSystem*> &res);
    void build_transition_system(const Timer &timer);

    void report_peak_memory_delta(bool final = false) const;
    void dump_options() const;
    void warn_on_unusual_options() const;
protected:
    virtual void initialize() override;
    virtual float compute_heuristic(const GlobalState &global_state) override;
    void get_candidate_vars(
            std::vector<int> &vars, std::set<std::set<int> > &explored,
            TransitionSystem *ts = nullptr) const;
public:
    explicit MergeAndShrinkHeuristicDFS(const Options &opts);
    ~MergeAndShrinkHeuristicDFS() = default;
};

#endif
