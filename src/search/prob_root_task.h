
#ifndef PROB_ABSTRACT_TASK_H
#define PROB_ABSTRACT_TASK_H

#include "root_task.h"

/* Just needed for successor generator generation */
class ProbRootTask : public RootTask
{
public:
    ProbRootTask() : RootTask() { }
    virtual float get_operator_cost(int index, bool is_axiom) const override;
    virtual const std::string &get_operator_name(int index,
            bool is_axiom) const override;
    virtual int get_num_operators() const override;
    virtual int get_num_operator_preconditions(int index,
            bool is_axiom) const override;
    virtual std::pair<int, int> get_operator_precondition(
        int op_index, int fact_index, bool is_axiom) const override;
    virtual int get_num_operator_effects(int op_index,
                                         bool is_axiom) const override;
    virtual int get_num_operator_effect_conditions(
        int op_index, int eff_index, bool is_axiom) const override;
    virtual std::pair<int, int> get_operator_effect_condition(
        int op_index, int eff_index, int cond_index, bool is_axiom) const override;
    virtual std::pair<int, int> get_operator_effect(
        int op_index, int eff_index, bool is_axiom) const override;
};

#endif

