
#include "unlimited_budget_p_relaxation.h"

#include <math.h>

#include "globals.h"
#include "plugin.h"
#include "option_parser.h"

#include "global_operator.h"

using namespace std;

UnlimitedBudgetPRelaxation::UnlimitedBudgetPRelaxation()
    : DelegatingTask(g_root_task())
{
    costs.resize(g_outcome_to_action.size());
    for (size_t i = 0; i < costs.size(); i++) {
        const std::pair<int, int> &op = g_outcome_to_action[i];
        costs[i] = log(1.0 / g_prob_operators[op.first][op.second].first);
    }
}

float UnlimitedBudgetPRelaxation::get_operator_cost(int index,
        bool /*is_axiom*/) const
{
    return costs[index];
}

static shared_ptr<AbstractTask> _parse(OptionParser &parser)
{
    if (!parser.dry_run()) {
        return shared_ptr<AbstractTask>(new UnlimitedBudgetPRelaxation());
    }
    return nullptr;
}

static PluginShared<AbstractTask> _plugin("ubprelax", _parse);

