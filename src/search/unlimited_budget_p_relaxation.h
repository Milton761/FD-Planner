
#ifndef UNLIMITED_BUDGET_P_RELAXATION_H
#define UNLIMITED_BUDGET_P_RELAXATION_H

#include "delegating_task.h"

#include <vector>

class UnlimitedBudgetPRelaxation : public DelegatingTask
{
    std::vector<float> costs;
public:
    UnlimitedBudgetPRelaxation();
    virtual float get_operator_cost(int index, bool is_axiom) const override;
};

#endif

