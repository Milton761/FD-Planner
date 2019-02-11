
#include "value_iteration.h"
#include "../merge_and_shrink_heuristic.h"
#include "../shrink_bisimulation.h"
#include "../budget_shrink.h"
#include "../labels.h"

#include "../../option_parser.h"
#include "../../plugin.h"
#include "../../evaluation_context.h"

#include "../transition_system.h"
#include "mas_search_space.h"

#include "../../globals.h"
#include "../../timer.h"

#include <unordered_map>
#include <stdio.h>
#include <ctime>

#include <fstream>
#include "../../task_proxy.h"
#include "../../utilities.h"

#include <forward_list>

using namespace std;

#define VERBOSE_DEBUGGING 0

#if VERBOSE_DEBUGGING
static void permutations(std::vector<int> &x,
                         std::vector<std::vector<int> > &res)
{
    size_t var = x.size();
    if (var == g_variable_domain.size()) {
        res.push_back(x);
    } else {
        for (int val = 0; val < g_variable_domain[var]; val++) {
            x.push_back(val);
            permutations(x, res);
            x.pop_back();
        }
    }
}

string state_representation(const std::vector<int> &s)
{
    string res = "[";
    for (size_t i = 0; i < s.size(); i++) {
        if (i > 0) {
            res += ", ";
        }
        res += g_fact_names[i][s[i]];
    }
    res += ']';
    return res;
}
#endif

MASValueIteration::MASValueIteration(const Options &opts)
    : SearchEngine(opts), merge_strategy(
        opts.get<shared_ptr<MergeStrategy> >("merge_strategy"))
{
    hide_mas_output = false;
}

void MASValueIteration::initialize()
{
    SearchEngine::initialize();

    cout << "Starting M&S construction..." << endl;
    streambuf *old = NULL;
    if (hide_mas_output) {
        old = cout.rdbuf(0);
    }
    clock_t begin = clock();

    vector<shared_ptr<ShrinkStrategy> > shrink_strategies;

    Options opts_shrink;
    opts_shrink.set<bool>("greedy", false);
    opts_shrink.set<int>("at_limit", 0);
    opts_shrink.set<int>("max_states", numeric_limits<int>::max());
    opts_shrink.set<int>("max_states_before_merge", numeric_limits<int>::max());
    opts_shrink.set<int>("threshold", 1);
    opts_shrink.set<int>("cost_type", 0);
    shrink_strategies.push_back(shared_ptr<ShrinkStrategy>
                                        (new BudgetShrink(opts_shrink)));
    shrink_strategies.push_back(shared_ptr<ShrinkStrategy>
                                        (new ShrinkBisimulation(opts_shrink)));

    Options opts_reduct;
    opts_reduct.set<bool>("before_shrinking", false);
    opts_reduct.set<bool>("before_merging", false);
    opts_reduct.set<int>("method", 0);
    opts_reduct.set<int>("system_order", 0);
    opts_reduct.set<int>("cost_type", 0);
    shared_ptr<Labels> label_reduction = shared_ptr<Labels>(new Labels(
            opts_reduct));

    Options opts_ms;
    opts_ms.set<shared_ptr<MergeStrategy> >("merge_strategy", merge_strategy);
    opts_ms.set<vector<shared_ptr<ShrinkStrategy> > >("shrink_strategy", shrink_strategies);
    opts_ms.set<shared_ptr<Labels> >("label_reduction", label_reduction);
    opts_ms.set<int>("cost_type", 0);
    opts_ms.set<bool>("shrink_result", true);
    MergeAndShrinkHeuristic &ms = *(new MergeAndShrinkHeuristic(opts_ms));

    ms.dump_options();

    Timer t;
    TransitionSystem &transition_system = *ms.build_transition_system(t);

    // abstract away from states that cannot be reached under the given budget
    if (g_budget >= 0) {
        bool pruned_something = false;
        vector<forward_list<int> > abstraction;
        abstraction.reserve(transition_system.get_size());
        for (int state = 0; state < transition_system.get_size(); state++) {
            if (!budget_is_sufficient(g_budget, transition_system.get_init_distance(state) + transition_system.get_goal_distance(state)))
            {
                pruned_something = true;
            } else {
                abstraction.push_back(forward_list<int>());
                abstraction.back().push_front(state);
            }
        }
        if (pruned_something) {
            transition_system.apply_abstraction(abstraction);
        }
        abstraction.clear();
    }

    if (hide_mas_output) {
        cout.rdbuf(old);
    }

    double elapsed = ((double)clock() - begin) / CLOCKS_PER_SEC;
    cout << "Merge-and-Shrink construction done in " << elapsed << "s" << endl;
    cout << "Copying data structures ..." << endl;

    begin = clock();

#if VERBOSE_DEBUGGING
    vector<vector<unsigned> > debug;
    vector<string> state_descriptions;
    debug.resize(transition_system.get_size());
    state_descriptions.resize(transition_system.get_size(), "");
    vector<int> _tmp1;
    vector<vector<int> > _tmp2;
    cout << "Computing all states"  << endl;
    permutations(_tmp1, _tmp2);
    cout << "Total number of states: " << _tmp2.size() << endl;
    cout << "Computing state descriptions for all states ..." << endl;
    for (size_t i = 0; i < _tmp2.size(); i++) {
        cout << "creating state#" << i << endl;
        State s(*(g_root_task()), vector<int>(_tmp2[i]));
        cout << "which corresponds to " << flush;
        int abstate = transition_system.find_state(s);
        cout << abstate << endl;
        if (abstate < 0) {
            continue;
        }
        if (state_descriptions[abstate].length() > 0) {
            state_descriptions[abstate] += "; ";
        }
        state_descriptions[abstate] += state_representation(_tmp2[i]);
    }
    cout << "Writing state space to .dot file" << endl;
    ofstream fout;
    fout.open("statespace.dot");
    fout << "digraph {" << endl;
    for (size_t i = 0; i < state_descriptions.size(); i++) {
        fout << "n" << i << " [label=\"{" << state_descriptions[i] << "} ("
             << i << ")\"";
        if (transition_system.is_goal_state(i)) {
            fout << ", peripheries=2";
        }
        fout << "]" << endl;
    }
    for (TSConstIterator it = transition_system.begin();
         it != transition_system.end(); ++it) {
        const vector<Transition> &transitions = it.get_transitions();
        int outcome_id = *transition_system.get_underlying_labels(it.get_id()).begin();
        for (size_t j = 0; j < transitions.size(); ++j) {
            const Transition &transition = transitions[j];
            fout << "n" << transition.src << " -> n" << transition.target
                 << " [label=\"" << g_operators[outcome_id].get_name() << "\"]" << endl;
        }
    }
    fout << "}" << endl;
    fout.close();
    //exit(1);
#endif

    print_peak_memory(false);

    //vector<unordered_map<int, int> > state_ops;
    //state_ops.resize(transition_system.get_size());
    state_space.resize(transition_system.get_size() +
                       1); // need an additionals state for dead ends
    dead_ends_id = transition_system.get_size();//state_space.size() - 1;
    for (TSConstIterator it = transition_system.begin();
         it != transition_system.end(); ++it) {
#if VERBOSE_DEBUGGING
        if (transition_system.get_underlying_labels(it.get_id()).size() > 1) {
            cout << "ERROR! Label reduction should be deactivated..." << endl;
            cout << "Label group " << it.get_id() << " represents " <<
                 transition_system.get_underlying_labels(it.get_id()).size()
                 << " many labels" << endl;
            exit(1);
        }
#endif
        const pair<int, int> &outcome =
            g_outcome_to_action[*transition_system.get_underlying_labels(it.get_id()).begin()];
        const vector<Transition> &transitions = it.get_transitions();
        for (size_t j = 0; j < transitions.size(); ++j) {
            const Transition &transition = transitions[j];
            vector<pair<int, vector<int> > > &successors = state_space[transition.src];
            unsigned at;

            at = 0;
            while (at < successors.size() && successors[at].first != outcome.first) {
                at++;
            }
            if (at == successors.size()) {
                successors.push_back(make_pair(outcome.first, vector<int>()));
                successors.back().second.resize(g_prob_operators[outcome.first].size(),
                                                dead_ends_id);
            }
//            unordered_map<int, int> &ops = state_ops[transition.src];
//            unordered_map<int, int>::iterator insert = ops.find(outcome.first);
//            if (insert == ops.end()) {
//                at = successors.size();
//                ops[outcome.first] = at;
//                successors.push_back(make_pair(outcome.first, vector<int>()));
//                successors.back().second.resize(g_prob_operators[outcome.first].size(), dead_ends_id);
//
//#if VERBOSE_DEBUGGING
//                debug[transition.src].push_back(0);
//#endif
//            } else {
//                at = insert->second;
//            }

#if VERBOSE_DEBUGGING
            if (transition.target == dead_ends_id) {
                cout << "there is a state with the same id as dead_ends" << endl;
                exit(1);
            }
            if (successors[at].first != outcome.first) {
                cout << "wrong operator lookup id..." << endl;
                exit(1);
            }
            int outcome_id = *transition_system.get_underlying_labels(it.get_id()).begin();
            if (g_operators[outcome_id].get_cost() != it.get_cost()) {
                cout << "cost of det_operator[outcome_id] does not match the cost of the label group"
                     << endl;
                exit(1);
            }
            debug[transition.src][at]++;
#endif

            successors[at].second[outcome.second] = transition.target ==
                                                    TransitionSystem::PRUNED_STATE ? dead_ends_id : transition.target;
        }
    }

    print_peak_memory(false);

    transition_system.release_memory();
    hvals.insert(hvals.end(), transition_system.get_goal_distances().begin(),
                 transition_system.get_goal_distances().end());
    goals.insert(goals.end(), transition_system.get_goal_states().begin(),
                 transition_system.get_goal_states().end());
    initial_state = transition_system.get_init_state();
    delete(&transition_system);
#if VERBOSE_DEBUGGING
    for (int state = 0; state < (int)debug.size(); state++) {
        for (int num = 0; num < (int)debug[state].size(); num++) {
            int op = state_ops[state][num];
            if (g_prob_operators[op].size() != debug[state][num]) {
                cout << "Invalid number of transitions ..." << endl;
                cout << "Should have got " << g_prob_operators[op].size()
                     << " outcomes, but got " << debug[state][num] << endl;
                cout << "Operator: " << g_prob_operator_name[op] << endl;
                exit(1);
            }
        }
    }

    ofstream fout2;
    fout2.open("statespace2.dot");
    fout2 << "digraph {" << endl;
    for (size_t i = 0; i < state_descriptions.size(); i++) {
        fout2 << "n" << i << " [label=\"{" << state_descriptions[i] << "} ("
              << i << ")\"]" << endl;
    }
    fout2 << " n" << dead_ends_id << " [label=\"dead\"]" << endl;
    for (size_t i = 0; i  < state_space.size(); i++) {
        const vector<pair<int, vector<int> > > &transitions = state_space[i];
        for (size_t j = 0; j < transitions.size(); ++j) {
            for (size_t k = 0; k < transitions[j].second.size(); k++) {
                fout2 << "n" << i << " -> n" << transitions[j].second[k]
                      << " [label=\"" <<
                      g_operators[g_prob_operators[transitions[j].first][k].second].get_name() <<
                      "\"]" << endl;
            }
        }
    }
    fout2 << "}" << endl;
    fout2.close();
#endif

    elapsed = ((double)clock() - begin) / CLOCKS_PER_SEC;
    cout << "Copied abstract transition system in "
         << elapsed << "s" << endl;

    delete(&ms);
}


#if VERBOSE_DEBUGGING
static void print_search_space(const string &filename,
                               MASSearchSpace &search_space)
{
    ofstream fout;
    fout.open(filename);
    fout << "digraph {" << endl;
    for (unsigned i = 0; i < search_space.size(); i++) {
        fout << "n" << i << " [label=\"(" << search_space[i].info.sid
             << ", " << search_space[i].info.budget
             << ", " << search_space[i].info.unsolved_successors
             << "), " << search_space[i].info.value
             << ", {" << search_space[i].info.status << "}"
             << "\"]" << endl;
    }

    for (unsigned i = 0; i < search_space.size(); i++) {
        const vector<vector<unsigned> > &succ = search_space[i].info.successors;
        for (unsigned j = 0; j < succ.size(); j++) {
            for (unsigned k = 0; k < succ[j].size(); k++) {
                fout << "n" << i << " -> n" << succ[j][k]
                     << " [label=\"" << g_prob_operator_name[search_space[i].info.operators[j]]
                     << "\"]" << endl;;
            }
        }
        for (unordered_set<unsigned>::const_iterator it =
                 search_space[i].info.parents.begin();
             it != search_space[i].info.parents.end(); it++) {
            fout << "n" << i << " -> n" << (*it) << " [style=\"dashed\"]" << endl;
        }
    }
    fout << "}" << endl;
    fout.close();
}
#endif

SearchStatus MASValueIteration::step()
{
    cout << "Constructing annotated state space ..." << endl;
    clock_t begin = clock();
    MASSearchSpace &search_space = *(new MASSearchSpace(state_space.size()));
    vector<unsigned> open_construction;
    vector<unsigned> open_vi;
    if (initial_state != dead_ends_id) {
        open_construction.push_back(search_space.get(initial_state, g_budget).get_id());
        search_space[0].info.set_seen();
    }
    MASSearchNode dead_ends = search_space.get(dead_ends_id, -1);
    dead_ends.info.set_dead_end();
    dead_ends.info.value = 0;
    open_vi.push_back(dead_ends.get_id());
    while (!open_construction.empty()) {
        MASSearchNode state = search_space[open_construction.back()];
        MASSearchNodeInfo &info = state.get_data();
        open_construction.pop_back();
        const vector<pair<int, vector<int> > > &succs = state_space[info.sid];
        for (size_t i = 0; i < succs.size(); i++) {
            const int &op = succs[i].first;
            const vector<int> &outcomes = succs[i].second;
            float cost = get_adjusted_cost(g_prob_operator_cost[op]);
            if (!budget_is_sufficient(info.budget, cost)) {
                continue;
            }
            //info.operators.push_back(op);
            //info.successors.push_back(vector<unsigned>());
            //vector<unsigned> &toc = info.successors.back();
            //toc.resize(outcomes.size());
            for (size_t i = 0; i < outcomes.size(); i++) {
                MASSearchNode succ = search_space.get(outcomes[i],
                                                      outcomes[i] == dead_ends_id ? -1 : compute_budget(info.budget, cost));
                pair<unordered_set<unsigned>::iterator, bool> insert =
                    succ.info.parents.insert(state.get_id());
                if (insert.second) {
                    state.info.unsolved_successors++;
                }
                //toc[i] = succ.get_id();
                if (succ.get_data().is_new()) {
                    succ.info.set_seen();
                    succ.info.value = 0;
                    if (goals[succ.info.sid]) {
                        succ.info.set_goal();
                        succ.info.value = 1;
                    } else if (hvals[succ.info.sid] > succ.info.budget) {
                        succ.info.set_dead_end();
                    }

                    if (succ.info.is_default()) {
                        open_construction.push_back(succ.get_id());
                    } else {
                        open_vi.push_back(succ.get_id());
                    }
                }
            }
        }
        if (state.info.unsolved_successors == 0) {
            open_vi.push_back(state.get_id());
        }
    }

    double elapsed = ((double) clock() - begin) / CLOCKS_PER_SEC;
    printf("Constructed annotated state space with %d states (%d) in %.4fs\n",
           (int)search_space.size(), (int)goals.size(), elapsed);

#if VERBOSE_DEBUGGING
    print_search_space("searchspace0.dot", search_space);
    //for (size_t i = 0; i < search_space.size(); i++) {
    //    cout << "UNSOLVED[" << search_space[i].info.sid << ", "
    //        << search_space[i].info.budget << "] = "
    //        << search_space[i].info.unsolved_successors << endl;
    //}
#endif

    while (!open_vi.empty()) {
        MASSearchNode state = search_space[open_vi.back()];
        open_vi.pop_back();

        if (state.info.is_default()) {
            int choice = -1;
            float max = 0;
            float tmp;
            for (unsigned i = 0; i < state_space[state.info.sid].size(); i++) {
                const int &op = state_space[state.info.sid][i].first;
                int cost = get_adjusted_cost(g_prob_operator_cost[op]);
                if (!budget_is_sufficient(state.info.budget, cost)) {
                    continue;
                }
                int new_budget = compute_budget(state.info.budget, cost);
                const vector<pair<float, int> > &outcomes = g_prob_operators[op];
                const vector<int> &successors = state_space[state.info.sid][i].second;
                tmp = 0;
                for (unsigned j = 0; j < successors.size(); j++) {
                    tmp += outcomes[j].first * search_space.get_value(successors[j],
                            successors[j] == dead_ends_id ? -1 : new_budget);
                }
                if (tmp > max) {
                    max = tmp;
                    choice = i;
                }
            }
            //for (unsigned i = 0; i < state.info.successors.size(); i++) {
            //    tmp = 0;
            //    const vector<pair<float, int> > &outcomes = g_prob_operators[state.info.operators[i]];
            //    for (unsigned j = 0; j < state.info.successors[i].size(); j++) {
            //        tmp += outcomes[j].first * search_space[state.info.successors[i][j]].info.value;
            //    }
            //    if (tmp > max) {
            //        max = tmp;
            //        choice = i;
            //    }
            //}
            state.info.value = max;
            state.info.policy = choice;
        }

        for (unordered_set<unsigned>::iterator it = state.info.parents.begin();
             it != state.info.parents.end(); it++) {
            if (--search_space[*it].info.unsolved_successors == 0) {
                open_vi.push_back(*it);
            }
        }
    }

#if VERBOSE_DEBUGGING
    print_search_space("searchspace1.dot", search_space);
#endif

    elapsed = ((double) clock() - begin) / CLOCKS_PER_SEC;
    printf("Value iteration converged after %.4fs.\n", elapsed);
    printf("Prob search space contains %d states (%d)\n", (int)search_space.size(), (int)state_space.size());
    printf("V[i] = %.9f\n", search_space[0].info.value);

    delete(&search_space);

    return SOLVED;
}

static SearchEngine *_parse(OptionParser &parser)
{
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<shared_ptr<MergeStrategy> >("merge_strategy");
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        return new MASValueIteration(opts);
    }
    return NULL;
}

static Plugin<SearchEngine> _plugin("masvi", _parse);

