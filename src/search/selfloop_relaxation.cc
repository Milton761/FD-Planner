
#include "selfloop_relaxation.h"

#include "globals.h"
#include "plugin.h"
#include "option_parser.h"

SelfLoopRelaxation::SelfLoopRelaxation()
    : DelegatingTask(g_root_task())
{
}

float SelfLoopRelaxation::get_operator_cost(int index, bool is_axiom) const
{
    const std::pair<int, int> &op = g_outcome_to_action[index];
    const float &prob = g_prob_operators[op.first][op.second].first;
    return (1.0 / prob) * DelegatingTask::get_operator_cost(index, is_axiom);
}

static AbstractTask *_parse(OptionParser &parser)
{
    if (!parser.dry_run()) {
        return new SelfLoopRelaxation();
    }
    return NULL;
}

static Plugin<AbstractTask> _plugin("selfloop", _parse);

