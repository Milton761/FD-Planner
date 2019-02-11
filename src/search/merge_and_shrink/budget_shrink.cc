
#include "budget_shrink.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "transition_system.h"
#include "../globals.h"

using namespace std;

BudgetShrink::BudgetShrink(const Options &opts) : ShrinkStrategy(opts)
{
}

void BudgetShrink::compute_equivalence_relation(const TransitionSystem &ts,
        int , StateEquivalenceRelation &equiv) const
{
    equiv.reserve(ts.get_size());
    for (int state = 0; state < ts.get_size(); state++) {
        if (g_budget < 0 ||
                ts.get_init_distance(state) + ts.get_goal_distance(state) <= g_budget) {
            forward_list<AbstractStateRef> c;
            c.push_front(state);
            equiv.push_back(c);
        }
    }
}

std::string BudgetShrink::name() const
{
    return "BudgetShrink";
}

void BudgetShrink::dump_strategy_specific_options() const
{
}

static shared_ptr<ShrinkStrategy> _parse(OptionParser &parser) {
    ShrinkStrategy::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        return shared_ptr<ShrinkStrategy>(new BudgetShrink(opts));
    }
    return nullptr;
}

static PluginShared<ShrinkStrategy> _plugin("budget", _parse);

