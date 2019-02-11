#ifndef MERGE_AND_SHRINK_MERGE_LINEAR_EXTENDED_H
#define MERGE_AND_SHRINK_MERGE_LINEAR_EXTENDED_H

#include "merge_strategy.h"
#include "merge_criteria.h"

#include <vector>
#include <memory>

class Options;

class MergeLinearExtended : public MergeStrategy
{
    // Only needed until variable order finder is initialized.
    const std::vector<shared_ptr<MergeCriterion> > criteria;
    const MergeOrder order;

    std::vector<int> selected_vars;
    std::vector<int> remaining_vars;

    void select_next(int var_no);
    bool need_first_index;
protected:
    virtual void dump_strategy_specific_options() const override;
    int next(TransitionSystem *prev);
public:
    explicit MergeLinearExtended(const Options &opts);
    virtual ~MergeLinearExtended() override = default;
    void initialize(const std::shared_ptr<AbstractTask> task, bool increment);
    virtual void initialize(const std::shared_ptr<AbstractTask> task) override;

    virtual std::pair<int, int> get_next(const std::vector<TransitionSystem *>
                                         &all_transition_systems) override;
    virtual std::string name() const override;
    //const std::vector<shared_ptr<MergeCriterion> > &get_criteria() const;

    void get_next(std::vector<int> &candidate_vars, TransitionSystem *abstraction);
};

//class MergeLinearDFS : public MergeStrategy {
//    // Only needed until variable order finder is initialized.
//    const std::vector<MergeCriterion*> criteria;
//    const MergeOrder order;
//
//    std::vector<int> selected_vars;
//    std::vector<int> remaining_vars;
//
//    void select_next(int var_no);
//    bool need_first_index;
//protected:
//    virtual void dump_strategy_specific_options() const override;
//    int next(TransitionSystem *prev);
//public:
//    explicit MergeLinearExtended(const Options &opts);
//    virtual ~MergeLinearExtended() override = default;
//    virtual void initialize(const std::shared_ptr<AbstractTask> task) override;
//
//    virtual std::pair<int, int> get_next(const std::vector<TransitionSystem *> &all_transition_systems) override;
//    virtual std::string name() const override;
//};


#endif
