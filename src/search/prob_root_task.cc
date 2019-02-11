
#include "prob_root_task.h"

#include "globals.h"
#include "global_operator.h"

using namespace std;

// NOTE: CURRENTLY, AXIOMS ARE NOT SUPPORTED

static inline GlobalOperator &get_first_action_outcome(int index)
{
    return g_operators[g_prob_operators[index][0].second];
}

float ProbRootTask::get_operator_cost(int index, bool /*is_axiom*/) const
{
    return get_first_action_outcome(index).get_cost();
}

const std::string &ProbRootTask::get_operator_name(int index,
        bool /*is_axiom*/) const
{
    return g_prob_operator_name[index];
}

int ProbRootTask::get_num_operators() const
{
    return g_prob_operators.size();
}

int ProbRootTask::get_num_operator_preconditions(int index,
        bool /*is_axiom*/) const
{
    return get_first_action_outcome(index).get_preconditions().size();
}

pair<int, int> ProbRootTask::get_operator_precondition(
    int op_index, int fact_index, bool /*is_axiom*/) const
{
    const GlobalOperator &op = get_first_action_outcome(op_index);
    const GlobalCondition &precondition = op.get_preconditions()[fact_index];
    return make_pair(precondition.var, precondition.val);
}

int ProbRootTask::get_num_operator_effects(int op_index, bool) const
{
    // SHOULD NOT BE CALLED!
    return get_first_action_outcome(op_index).get_effects().size();
}

int ProbRootTask::get_num_operator_effect_conditions(
    int op_index, int eff_index, bool) const
{
    // SHOULD NOT BE CALLED!
    return get_first_action_outcome(
               op_index).get_effects()[eff_index].conditions.size();
}

pair<int, int> ProbRootTask::get_operator_effect_condition(
    int op_index, int eff_index, int cond_index, bool) const
{
    // SHOULD NOT BE CALLED!
    const GlobalEffect &effect = get_first_action_outcome(
                                     op_index).get_effects()[eff_index];
    const GlobalCondition &condition = effect.conditions[cond_index];
    return make_pair(condition.var, condition.val);
}

pair<int, int> ProbRootTask::get_operator_effect(
    int op_index, int eff_index, bool) const
{
    // SHOULD NOT BE CALLED!
    const GlobalEffect &effect = get_first_action_outcome(
                                     op_index).get_effects()[eff_index];
    return make_pair(effect.var, effect.val);
}


