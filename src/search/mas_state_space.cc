
#ifdef MAS_STATE_SPACE_H

#include "option_parser.h"

#include "merge_and_shrink/shrink_strategy.h"
#include "merge_and_shrink/shrink_bisimulation.h"
#include "merge_and_shrink/budget_shrink.h"
#include "merge_and_shrink/labels.h"
#include "merge_and_shrink/merge_and_shrink_heuristic.h"
#include "merge_and_shrink/transition_system.h"

#include "timer.h"

#include "globals.h"
#include "root_task.h"

#include <cassert>
#include <ctime>
#include <vector>
#include <forward_list>

#define VERBOSE_DEBUGGING 1
#if VERBOSE_DEBUGGING >= 2
#include <fstream>
#endif

template<typename State, typename StateSpace>
MASStateSpace<State, StateSpace>::MASStateSpace(const Options &opts)
    :  is_budget_limited(g_budget >= 0),
        is_initialized(false),
       merge_strategy(opts.get<std::shared_ptr<MergeStrategy> >("merge_strategy")),
       shrink_strategies(
           opts.get_list<std::shared_ptr<ShrinkStrategy> >("shrink_strategy")),
       task_proxy(TaskProxy(*g_root_task()))
{
    hide_mas_output = false;
}

template<typename State, typename StateSpace>
void MASStateSpace<State, StateSpace>::initialize()
{
    if (is_initialized) {
        return;
    }
    is_initialized = true;
    std::cout << "Starting M&S construction..." << std::endl;
    std::streambuf *old = NULL;
    if (hide_mas_output) {
        old = std::cout.rdbuf(0);
    }
    clock_t begin = clock();

    Options opts_reduct;
    opts_reduct.set<bool>("before_shrinking", false);
    opts_reduct.set<bool>("before_merging", false);
    opts_reduct.set<int>("method", 0);
    opts_reduct.set<int>("system_order", 0);
    opts_reduct.set<int>("cost_type", 0);
    std::shared_ptr<Labels> label_reduction = std::shared_ptr<Labels>(new Labels(
                opts_reduct));

    Options opts_ms;
    opts_ms.set<std::shared_ptr<MergeStrategy> >("merge_strategy", merge_strategy);
    opts_ms.set<std::vector<std::shared_ptr<ShrinkStrategy> > >("shrink_strategy",
            shrink_strategies);
    opts_ms.set<std::shared_ptr<Labels> >("label_reduction", label_reduction);
    opts_ms.set<int>("cost_type", 0);
    opts_ms.set<bool>("shrink_result", true);
    MergeAndShrinkHeuristic &ms = *(new MergeAndShrinkHeuristic(opts_ms));

    Timer t;
    TransitionSystem &transition_system = *ms.build_transition_system(t);

#if 0
    // abstract away from states that cannot be reached under the given budget
    // already done by budget shrink strategy?
    if (g_budget >= 0) {
        bool pruned_something = false;
        std::vector<std::forward_list<int> > abstraction;
        abstraction.reserve(transition_system.get_size());
        for (int state = 0; state < transition_system.get_size(); state++) {
            if (!budget_is_sufficient(g_budget,
                                      transition_system.get_init_distance(state) +
                                      transition_system.get_goal_distance(state))) {
                pruned_something = true;
            } else {
                abstraction.push_back(std::forward_list<int>());
                abstraction.back().push_front(state);
            }
        }
        if (pruned_something) {
            transition_system.apply_abstraction(abstraction);
        }
        abstraction.clear();
    }
#endif

    if (hide_mas_output) {
        std::cout.rdbuf(old);
    }

    double elapsed = ((double)clock() - begin) / CLOCKS_PER_SEC;
    std::cout << "Merge-and-Shrink construction done in " << elapsed << "s" << endl;
    std::cout << "Copying data structures ..." << endl;

    begin = clock();

#if VERBOSE_DEBUGGING >= 2
    //std::vector<std::vector<unsigned> > debug;
    //debug.resize(transition_system.get_size());
    std::vector<string> state_descriptions;
    state_descriptions.resize(transition_system.get_size(), "");
    for (int i = 0; i < transition_system.get_size(); i++) {
        state_descriptions[i] = "state#" + std::to_string(i);
    }
    //std::vector<int> _tmp1;
    //std::vector<std::vector<int> > _tmp2;
    //std::cout << "Computing all states"  << endl;
    //permutations(_tmp1, _tmp2);
    //std::cout << "Total number of states: " << _tmp2.size() << endl;
    //std::cout << "Computing state descriptions for all states ..." << endl;
    //for (size_t i = 0; i < _tmp2.size(); i++) {
    //    std::cout << "creating state#" << i << endl;
    //    State s(*(g_root_task()), std::vector<int>(_tmp2[i]));
    //    std::cout << "which corresponds to " << flush;
    //    int abstate = transition_system.find_state(s);
    //    std::cout << abstate << endl;
    //    if (abstate < 0) {
    //        continue;
    //    }
    //    if (state_descriptions[abstate].length() > 0) {
    //        state_descriptions[abstate] += "; ";
    //    }
    //    state_descriptions[abstate] += state_representation(_tmp2[i]);
    //}
    std::cout << "Writing state space to .dot file" << endl;
    std::ofstream fout;
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
        const std::vector<Transition> &transitions = it.get_transitions();

        for (std::list<int>::const_iterator it2 =
                 transition_system.get_underlying_labels(it.get_id()).begin();
             it2 != transition_system.get_underlying_labels(it.get_id()).end(); it2++) {
            int outcome_id = *it2;
            for (size_t j = 0; j < transitions.size(); ++j) {
                const Transition &transition = transitions[j];
                fout << "n" << transition.src << " -> n" << transition.target
                     << " [label=\"" << g_operators[outcome_id].get_name() << "\"]" << endl;
            }
        }
    }
    fout << "}" << endl;
    fout.close();
    //exit(1);
#endif

    print_peak_memory(false);

    //std::vector<unordered_map<int, int> > state_ops;
    //state_ops.resize(transition_system.get_size());
    state_space.resize(transition_system.get_size() +
                       1); // need an additionals state for dead ends
    dead_ends_id = transition_system.get_size();//state_space.size() - 1;
    for (TSConstIterator it = transition_system.begin();
         it != transition_system.end(); ++it) {
#if VERBOSE_DEBUGGING
        if (transition_system.get_underlying_labels(it.get_id()).size() > 1) {
            std::cout << "ERROR! Label reduction should be deactivated..." << endl;
            std::cout << "Label group " << it.get_id() << " represents " <<
                      transition_system.get_underlying_labels(it.get_id()).size()
                      << " labels" << endl;
            //exit(1);
        }
#endif
        assert(transition_system.get_underlying_labels(it.get_id()).size() == 1);
        const std::pair<int, int> &outcome =
            g_outcome_to_action[*transition_system.get_underlying_labels(it.get_id()).begin()];
        const std::vector<Transition> &transitions = it.get_transitions();
        for (size_t j = 0; j < transitions.size(); ++j) {
            const Transition &transition = transitions[j];
            std::vector<std::pair<int, std::vector<int> > > &successors =
                state_space[transition.src];
            unsigned at;

            at = 0;
            while (at < successors.size() && successors[at].first != outcome.first) {
                at++;
            }
            if (at == successors.size()) {
                successors.push_back(std::make_pair(outcome.first, std::vector<int>()));
                successors.back().second.resize(g_prob_operators[outcome.first].size(),
                                                dead_ends_id);
            }
//            unordered_map<int, int> &ops = state_ops[transition.src];
//            unordered_map<int, int>::iterator insert = ops.find(outcome.first);
//            if (insert == ops.end()) {
//                at = successors.size();
//                ops[outcome.first] = at;
//                successors.push_back(std::make_pair(outcome.first, std::vector<int>()));
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
                std::cout << "there is a state with the same id as dead_ends" << endl;
                exit(1);
            }
            if (successors[at].first != outcome.first) {
                std::cout << "wrong operator lookup id..." << endl;
                exit(1);
            }
            int outcome_id = *transition_system.get_underlying_labels(it.get_id()).begin();
            if (g_operators[outcome_id].get_cost() != it.get_cost()) {
                std::cout <<
                          "cost of det_operator[outcome_id] does not match the cost of the label group"
                          << endl;
                exit(1);
            }
            //debug[transition.src][at]++;
#endif

            successors[at].second[outcome.second] = transition.target ==
                                                    TransitionSystem::PRUNED_STATE ? dead_ends_id : transition.target;
        }
    }

    print_peak_memory(false);

    transition_system.release_memory();
    hvals.insert(hvals.end(), transition_system.get_goal_distances().begin(),
                 transition_system.get_goal_distances().end());
    hvals.push_back(std::numeric_limits<int>::max());
    goals.insert(goals.end(), transition_system.get_goal_states().begin(),
                 transition_system.get_goal_states().end());
    goals.push_back(false);
    initial_state = transition_system.get_init_state();
    if (initial_state < 0) {
        initial_state = dead_ends_id;
    }
    hrepr = transition_system.get_heuristic_representation();
    delete(&transition_system);
#if VERBOSE_DEBUGGING >= 2
    //for (int state = 0; state < (int)debug.size(); state++) {
    //    for (int num = 0; num < (int)debug[state].size(); num++) {
    //        int op = state_ops[state][num];
    //        if (g_prob_operators[op].size() != debug[state][num]) {
    //            std::cout << "Invalid number of transitions ..." << endl;
    //            std::cout << "Should have got " << g_prob_operators[op].size()
    //                 << " outcomes, but got " << debug[state][num] << endl;
    //            std::cout << "Operator: " << g_prob_operator_name[op] << endl;
    //            exit(1);
    //        }
    //    }
    //}

    std::ofstream fout2;
    fout2.open("statespace2.dot");
    fout2 << "digraph {" << endl;
    for (size_t i = 0; i < state_descriptions.size(); i++) {
        fout2 << "n" << i << " [label=\"{" << state_descriptions[i] << "} ("
              << i << ")\"]" << endl;
    }
    fout2 << " n" << dead_ends_id << " [label=\"dead\"]" << endl;
    for (size_t i = 0; i  < state_space.size(); i++) {
        const std::vector<std::pair<int, std::vector<int> > > &transitions =
            state_space[i];
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
    std::cout << "Copied abstract transition system in "
              << elapsed << "s" << endl;

    delete(&ms);


}

template<typename State, typename StateSpace>
unsigned MASStateSpace<State, StateSpace>::get_state_id(const GlobalState &state)
{
    return hrepr->get_abstract_state(task_proxy.convert_global_state(state));
}



namespace state_space_mas
{
static void copy_options(const Options &from, Options &to)
{
    to.set<std::shared_ptr<MergeStrategy> >("merge_strategy",
                                            from.get<std::shared_ptr<MergeStrategy> >("merge_strategy"));
    if (from.contains("shrink_strategy")) {
        to.set<std::vector<std::shared_ptr<ShrinkStrategy> > >("shrink_strategy",
                from.get_list<std::shared_ptr<ShrinkStrategy> >("shrink_strategy"));
    } else {
        std::vector<std::shared_ptr<ShrinkStrategy> > shrink_strategies;

        Options opts_shrink;
        opts_shrink.set<bool>("greedy", false);
        opts_shrink.set<int>("at_limit", 0);
        opts_shrink.set<int>("max_states", numeric_limits<int>::max());
        opts_shrink.set<int>("max_states_before_merge", numeric_limits<int>::max());
        opts_shrink.set<int>("threshold", 1);
        opts_shrink.set<int>("cost_type", 0);
        if (g_budget >= 0) {
            shrink_strategies.push_back(std::shared_ptr<ShrinkStrategy>
                                        (new BudgetShrink(opts_shrink)));
        }
        shrink_strategies.push_back(std::shared_ptr<ShrinkStrategy>
                                    (new ShrinkBisimulation(opts_shrink)));

        to.set<std::vector<std::shared_ptr<ShrinkStrategy> > >("shrink_strategy",
                shrink_strategies);
    }
}
static void add_options_to_parser(OptionParser &parser)
{
    parser.add_option<std::shared_ptr<MergeStrategy> >("merge_strategy", "",
            "merge_linear_ext(criteria=[scc(),goal()], order=level)");
    parser.add_list_option<std::shared_ptr<ShrinkStrategy> >("shrink_strategy", "",
            OptionParser::NONE);
}
}


#endif
