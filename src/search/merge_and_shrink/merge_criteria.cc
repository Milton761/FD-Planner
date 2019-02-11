
#include "merge_criteria.h"

#include "transition_system.h"

#include "../legacy_causal_graph.h"
#include "../globals.h"

#include "../option_parser.h"
#include "../option_parser_util.h"
#include "../plugin.h"


#include <cassert>
#include <cstdlib>
#include <vector>
#include <map>

using namespace std;

void MergeCriterionCG::init()
{
    is_causal_predecessor.resize(g_variable_domain.size(), false);
}

void MergeCriterionCG::select_next(int var_no)
{
    if (allow_incremental) {
        const vector<int> &new_vars = g_legacy_causal_graph->get_predecessors(var_no);
        for (size_t i = 0; i < new_vars.size(); ++i) {
            is_causal_predecessor[new_vars[i]] = true;
        }
    }
}

void MergeCriterionCG::filter(std::vector <int> &vars,
                              TransitionSystem *abstraction)
{
    if (!abstraction) {
        return;
    }
    if (!allow_incremental) {
        is_causal_predecessor.resize(g_variable_domain.size(), false);
        const vector<int> &varset = abstraction->get_varset();
        for (size_t v = 0; v < varset.size(); v++) {
            const vector<int> &new_vars = g_legacy_causal_graph->get_predecessors(
                                              varset[v]);
            for (size_t i = 0; i < new_vars.size(); ++i) {
                is_causal_predecessor[new_vars[i]] = true;
            }
        }
    }
    MergeCriterion::filter(vars, is_causal_predecessor);
}


void MergeCriterionGoal::init()
{
    is_goal_variable.resize(g_variable_domain.size(), false);
    for (size_t i = 0; i < g_goal.size(); ++i) {
        is_goal_variable[g_goal[i].first] = true;
    }
}

void MergeCriterionGoal::select_next(int /*var_no*/) {}

void MergeCriterionGoal::filter(std::vector <int> &vars,
                                TransitionSystem * /*abstraction*/)
{
    MergeCriterion::filter(vars, is_goal_variable);
}

MergeCriterionMinSCC::MergeCriterionMinSCC(const Options &opts) :
    MergeCriterion(opts),
    reverse(opts.get<bool> ("reverse")),
    tie_by_level(opts.get<bool>("level"))
{
}

void MergeCriterionMinSCC::init()
{
    is_causal_predecessor.resize(g_variable_domain.size(), false);
    if (reverse) {
        scc.compute_scc(g_legacy_causal_graph->get_inverse_arcs());
    } else {
        scc.compute_scc(g_legacy_causal_graph->get_arcs());
    }
}

void MergeCriterionMinSCC::select_next(int var_no)
{
    if (allow_incremental) {
        const vector<int> &new_vars = g_legacy_causal_graph->get_predecessors(var_no);
        for (size_t i = 0; i < new_vars.size(); ++i) {
            is_causal_predecessor[new_vars[i]] = true;
        }
    }
}

void MergeCriterionMinSCC::forbid_scc_descendants(int scc_index,
        const vector<set<int> > &scc_graph,
        vector<bool> &forbidden_sccs) const
{
    const set<int> &descendants = scc_graph[scc_index];
    for (set<int>::iterator it = descendants.begin(); it != descendants.end();
         ++it) {
        if (!forbidden_sccs[*it]) {
            forbidden_sccs [*it] = true;
            forbid_scc_descendants(*it, scc_graph, forbidden_sccs);
        }
    }
}

void MergeCriterionMinSCC::filter(std::vector <int> &vars,
                                  TransitionSystem *abstraction)
{
    if (!abstraction) {
        return;
    }
    if (!allow_incremental) {
        is_causal_predecessor.resize(g_variable_domain.size(), false);
        const vector<int> &varset = abstraction->get_varset();
        for (size_t v = 0; v < varset.size(); v++) {
            const vector<int> &new_vars = g_legacy_causal_graph->get_predecessors(
                                              varset[v]);
            for (size_t i = 0; i < new_vars.size(); ++i) {
                is_causal_predecessor[new_vars[i]] = true;
            }
        }
    }

    if (!MergeCriterion::filter(vars, is_causal_predecessor)) {
        return; //No CG relevant vars => we do not prefer any variable over another
    }

    const vector<set<int> > &scc_graph = scc.get_scc_graph();
    const vector<int> &vars_scc = scc.get_vars_scc();
    vector<bool> forbidden_sccs(scc_graph.size(), false);
    //In each SCC,select only the variable whose "level" is "closer to the root"
    //We consider a variable closer to the root if it has lower id
    //If reverse is activated, we consider the opposite order
    map<int, int> vars_by_scc;
    //1) forbid all sccs pointed by scc_var
    for (size_t i = 0; i < vars.size(); i++) {
        int var = vars[i];
        int scc_var = vars_scc [var];
        if (!forbidden_sccs[scc_var]) {
            forbid_scc_descendants(scc_var, scc_graph, forbidden_sccs);
            if (!vars_by_scc.count(scc_var) || (!reverse && vars_by_scc[scc_var] > var)
                || (reverse && vars_by_scc[scc_var] < var)) {
                vars_by_scc [scc_var] = var;
            }
        }
    }

    //2) Filter all variables whose scc has been forbidden.
    vector<int> new_vars;
    if (tie_by_level) { //For valid sccs, include the selected variable
        for (map<int, int>::iterator it = vars_by_scc.begin();
             it != vars_by_scc.end(); ++it) {
            if (!forbidden_sccs[it->first]) {
                new_vars.push_back(it->second);
            }
        }
    } else {
        //For valid sccs, include all variables
        for (size_t i = 0; i < vars.size(); i++) {
            int var = vars[i];
            int scc_var = vars_scc [vars[i]];
            if (!forbidden_sccs[scc_var]) {
                new_vars.push_back(var);
            }
        }
    }
    vars.swap(new_vars);
    /* cout << "Candidate variable after scc: ";
    for(int i = 0; i < vars.size(); i++)
      cout << vars[i] << " ";
      cout << endl;*/

}

void MergeCriterionEmpty::init() {}
void MergeCriterionEmpty::select_next(int /*var_no*/) {}
void MergeCriterionEmpty::filter(std::vector <int> &vars,
                                 TransitionSystem *abstraction)
{
    if (abstraction) { //Check if abstraction exists, because the first variable is selected without abstraction
        const vector<double> &score = abstraction->get_count_transitions_var_empty();
        // for(int i = 0; i < vars.size(); i++)
        //   cout << "ScoreEmpty " << vars[i] << ": " << score[vars[i]] << " " << endl;
        MergeCriterion::filter_best(vars, score, false);
    }
}

void MergeCriterionEmptyGoal::init() {}
void MergeCriterionEmptyGoal::select_next(int /*var_no*/) {}
void MergeCriterionEmptyGoal::filter(std::vector <int> &vars,
                                     TransitionSystem *abstraction)
{
    if (abstraction) { //Check if abstraction exists, because the first variable is selected without abstraction
        const vector<double> &score =
            abstraction->get_count_transitions_var_empty_goal();
        // for(int i = 0; i < vars.size(); i++)
        //   cout << "ScoreEmptyGoal " << vars[i] << ": " << score[vars[i]] << " " << endl;
        MergeCriterion::filter_best(vars, score, false);
    }
}


void MergeCriterionNum::init() {}
void MergeCriterionNum::select_next(int /*var_no*/) {}
void MergeCriterionNum::filter(std::vector <int> &vars,
                               TransitionSystem *abstraction)
{
    if (abstraction) { //Check if abstraction exists, because the first variable is selected without abstraction
        const vector<double> &score = abstraction->get_count_transitions_var();
        // for(int i = 0; i < vars.size(); i++)
        //   cout << "ScoreNum " << vars[i] << ": " << score[vars[i]] << " " << endl;
        MergeCriterion::filter_best(vars, score, false);
    }
}

void MergeCriterionRelevant::init()
{
    MergeCriterionCG::init();
    for (size_t i = 0; i < g_goal.size(); ++i) {
        is_causal_predecessor[g_goal[i].first] = true;
    }
}

template <class T>
static shared_ptr<MergeCriterion> _parse(OptionParser &parser)
{
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    } else {
        return shared_ptr<MergeCriterion>(new T(opts));
    }
}

//TODO: This should be removed, because it is equivalent to scc(level=false)
static shared_ptr<MergeCriterion> _parse_scc_no_level(OptionParser &parser)
{
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    } else {
        opts.set<bool>("reverse", false);
        opts.set<bool>("level", false);
        return shared_ptr<MergeCriterion>(new MergeCriterionMinSCC(opts));
    }
}

static shared_ptr<MergeCriterion> _parse_scc(OptionParser &parser)
{
    parser.add_option<bool>("reverse", "",  "false");
    parser.add_option<bool>("level", "", "false");
    Options opts = parser.parse();

    if (parser.dry_run()) {
        return nullptr;
    } else {
        return shared_ptr<MergeCriterion>(new MergeCriterionMinSCC(opts));
    }
}

//MultipleLinearMergeStrategy::MultipleLinearMergeStrategy(
//    const std::vector <MergeCriterion *> &merge_criteria,
//    const MergeOrder &merge_order, bool /*is_first*/) :
//    criteria(merge_criteria), order(merge_order)
//{
//
//    for (int i = 0; i < merge_criteria.size(); ++i) {
//        merge_criteria[i]->disable_incremental();
//        merge_criteria[i]->init();
//    }
//}
//
//MultipleLinearMergeStrategy::~MultipleLinearMergeStrategy()
//{
//}
//
//void MultipleLinearMergeStrategy::next_vars(vector<int> &candidate_vars,
//        TransitionSystem *abstraction) const
//{
//    //Apply the criteria in order, until its finished or there is only one remaining variable
//    for (int i = 0; candidate_vars.size() > 1 &&
//         i < criteria.size(); ++i) {
//        criteria[i]->filter(candidate_vars, abstraction);
//    }
//
//    //Order the remaining variables based on the different criteria
//    //std::sort(candidate_vars.begin(), candidate_vars.end(),
//    //	    this->cmp);
//
//    cout << "Candidates: ";
//    for (int i = 0; i < candidate_vars.size(); i++) {
//        cout << candidate_vars[i] << " ";
//    }
//    cout << endl;
//
//    if (order == RANDOM) {
//        random_shuffle(candidate_vars.begin(),
//                       candidate_vars.end());
//    } else if (order == LEVEL) {
//        std::sort(candidate_vars.begin(), candidate_vars.end(), std::greater<int>());
//    } else if (order == REVERSE_LEVEL) {
//        std::sort(candidate_vars.begin(), candidate_vars.end());
//    }
//}
//
//bool MultipleLinearMergeStrategy::cmp(const int v1, const int v2)
//{
//    return v1 < v2;
//}


static PluginShared<MergeCriterion> _plugin_cg("cg",
        _parse<MergeCriterionCG>);
static PluginShared<MergeCriterion> _plugin_goal("goal",
        _parse<MergeCriterionGoal>);
static PluginShared<MergeCriterion> _plugin_scc("scc",   _parse_scc);
static PluginShared<MergeCriterion> _plugin_scc_no_level("scc_no_level",
        _parse_scc_no_level);
static PluginShared<MergeCriterion> _plugin_empty("empty",
        _parse<MergeCriterionEmpty>);
static PluginShared<MergeCriterion> _plugin_empty_goal("empty_goal",
        _parse<MergeCriterionEmptyGoal>);
static PluginShared<MergeCriterion> _plugin_num("num",
        _parse<MergeCriterionNum>);
static PluginShared<MergeCriterion> _plugin_rel("relevant",
        _parse<MergeCriterionRelevant>);

