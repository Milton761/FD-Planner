
#ifndef SELF_LOOP_RELAXATION_H
#define SELF_LOOP_RELAXATION_H

#include "delegating_task.h"

class SelfLoopRelaxation : public DelegatingTask
{
public:
    SelfLoopRelaxation();
    virtual float get_operator_cost(int index, bool is_axiom) const;
};

#endif

