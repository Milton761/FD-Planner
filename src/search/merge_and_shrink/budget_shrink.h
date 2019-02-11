
#ifndef BUDGET_SHRINK_H
#define BUDGET_SHRINK_H

#include "shrink_strategy.h"

class BudgetShrink : public ShrinkStrategy
{
protected:
    virtual void compute_equivalence_relation(
        const TransitionSystem &ts,
        int target,
        StateEquivalenceRelation &equivalence_relation) const;
    virtual std::string name() const;
    virtual void dump_strategy_specific_options() const;
public:
    BudgetShrink(const Options &opts);
};

#endif
