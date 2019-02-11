#include "merge_linear_extended.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../utilities.h"

#include "merge_criteria.h"

#include <cassert>
#include <iostream>
#include <algorithm>

using namespace std;

MergeLinearExtended::MergeLinearExtended(const Options &opts)
    : MergeStrategy(),
      criteria(opts.get_list<shared_ptr<MergeCriterion> >("criteria")),
      order(MergeOrder(opts.get_enum("order"))),
      need_first_index(true)
{
}

void MergeLinearExtended::initialize(const shared_ptr<AbstractTask> task)
{
    initialize(task, true);
}

void MergeLinearExtended::initialize(const shared_ptr<AbstractTask> task, bool increment)
{
    MergeStrategy::initialize(task);

    remaining_vars.resize(task->get_num_variables());
    int num = static_cast<int>(remaining_vars.size());
    switch (order) {
    case MO_REVERSE_LEVEL:
        for (int i = 0; i < num; ++i) {
            remaining_vars[i] = i;
        }
        break;
    default:
        for (int i = 0; i < num; ++i) {
            remaining_vars[i] = num - 1 - i;
        }
        break;
    }
    if (order == MO_RANDOM) {
        random_shuffle(remaining_vars.begin(), remaining_vars.end());
    }

    for (size_t i = 0; i < criteria.size(); i++) {
        if (!increment) {
            criteria[i]->disable_incremental();
        }
        criteria[i]->init();
    }
}

void MergeLinearExtended::select_next(int var_no)
{
    vector<int>::iterator pos = find(remaining_vars.begin(), remaining_vars.end(),
                                     var_no);
    remaining_vars.erase(pos);
    selected_vars.push_back(var_no);
    for (size_t i = 0; i < criteria.size(); i++) {
        criteria[i]->select_next(var_no);
    }
}

int MergeLinearExtended::next(TransitionSystem *prev)
{
    vector<int> candidate_vars(remaining_vars);
    for (size_t i = 0; candidate_vars.size() > 1 && i < criteria.size(); ++i) {
        criteria[i]->filter(candidate_vars, prev);
    }
    int var = candidate_vars[0];
    select_next(var);
    return var;
}

pair<int, int> MergeLinearExtended::get_next(const vector<TransitionSystem *>
        &all_transition_systems)
{
    assert(initialized());
    assert(!done());

    int first;
    if (need_first_index) {
        need_first_index = false;
        first = next(NULL);
    } else {
        first = all_transition_systems.size() - 1;
    }
    int second = next(all_transition_systems[first]);
    --remaining_merges;
    return make_pair(first, second);
}

void MergeLinearExtended::get_next(std::vector<int> &candidate_vars,
                                   TransitionSystem *abstraction)
{
    //Apply the criteria in order, until its finished or there is only one remaining variable
    if (candidate_vars.empty()) {
        return;
    }

    for (size_t i = 0; candidate_vars.size() > 1 &&
         i < criteria.size(); ++i) {
        criteria[i]->filter(candidate_vars, abstraction);
    }

    if (order == MO_RANDOM) {
        random_shuffle(candidate_vars.begin(),
                       candidate_vars.end());
    } else if (order == MO_LEVEL) {
        std::sort(candidate_vars.begin(), candidate_vars.end(), std::greater<int>());
    } else if (order == MO_REVERSE_LEVEL) {
        std::sort(candidate_vars.begin(), candidate_vars.end());
    }
}

void MergeLinearExtended::dump_strategy_specific_options() const
{
}

string MergeLinearExtended::name() const
{
    return "linear_extended";
}

static shared_ptr<MergeStrategy>_parse(OptionParser &parser)
{
    parser.add_list_option<shared_ptr<MergeCriterion> >("criteria", "");
    vector<string> merge_order;
    merge_order.push_back("level");
    merge_order.push_back("reverse_level");
    merge_order.push_back("random");
    parser.add_enum_option("order", merge_order, "");

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    } else {
        return make_shared<MergeLinearExtended>(opts);
    }
}

static PluginShared<MergeStrategy> _plugin("merge_linear_ext", _parse);
