#include "merge_and_shrink_heuristic_dfs.h"

#include "factored_transition_system.h"
#include "fts_factory.h"
#include "labels.h"
#include "merge_linear_extended.h"
#include "shrink_strategy.h"
#include "transition_system.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../timer.h"
#include "../utilities.h"

#include <cassert>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace std;

MergeAndShrinkHeuristicDFS::MergeAndShrinkHeuristicDFS(const Options &opts)
    : Heuristic(opts),
      shrink_strategy(opts.get<shared_ptr<ShrinkStrategy> >("shrink_strategy")),
      labels(opts.get<shared_ptr<Labels> >("label_reduction")),
      starting_peak_memory(-1),
      max_branches(opts.get<int>("max_branching_merge"))
{
    merge_strategy = dynamic_pointer_cast<MergeLinearExtended>(
      opts.get<shared_ptr<MergeStrategy> >("merge_strategy"));
    if (merge_strategy == nullptr) {
        exit_with(EXIT_CRITICAL_ERROR);
    }
    /*
      TODO: Can we later get rid of the initialize calls, after rethinking
      how to handle communication between different components? See issue559.
    */
    merge_strategy->initialize(task, false);
    labels->initialize(task_proxy);

    num_vars = task->get_num_variables();
}

void MergeAndShrinkHeuristicDFS::report_peak_memory_delta(bool final) const
{
    if (final) {
        cout << "Final";
    } else {
        cout << "Current";
    }
    cout << " peak memory increase of merge-and-shrink computation: "
         << get_peak_memory_in_kb() - starting_peak_memory << " KB" << endl;
}

void MergeAndShrinkHeuristicDFS::dump_options() const
{
    merge_strategy->dump_options();
    shrink_strategy->dump_options();
    labels->dump_options();
}

void MergeAndShrinkHeuristicDFS::warn_on_unusual_options() const
{
    string dashes(79, '=');
    if (!labels->reduce_before_merging() && !labels->reduce_before_shrinking()) {
        cerr << dashes << endl
             << "WARNING! You did not enable label reduction. This may "
             "drastically reduce the performance of merge-and-shrink!"
             << endl << dashes << endl;
    } else if (labels->reduce_before_merging()
               && labels->reduce_before_shrinking()) {
        cerr << dashes << endl
             << "WARNING! You set label reduction to be applied twice in "
             "each merge-and-shrink iteration, both before shrinking and\n"
             "merging. This double computation effort does not pay off "
             "for most configurations!"
             << endl << dashes << endl;
    } else {
        if (labels->reduce_before_shrinking() &&
            (shrink_strategy->get_name() == "f-preserving"
             || shrink_strategy->get_name() == "random")) {
            cerr << dashes << endl
                 << "WARNING! Bucket-based shrink strategies such as "
                 "f-preserving random perform best if used with label\n"
                 "reduction before merging, not before shrinking!"
                 << endl << dashes << endl;
        }
        if (labels->reduce_before_merging() &&
            shrink_strategy->get_name() == "bisimulation") {
            cerr << dashes << endl
                 << "WARNING! Shrinking based on bisimulation performs best "
                 "if used with label reduction before shrinking, not\n"
                 "before merging!"
                 << endl << dashes << endl;
        }
    }
}

void MergeAndShrinkHeuristicDFS::get_candidate_vars(
        std::vector<int> &vars,
        std::set<std::set<int> > &explored,
        TransitionSystem *ts) const
{
    set<int> vset;
    if (ts) {
        const std::vector<int> &tsvars = ts->get_varset();
        vset.insert(tsvars.begin(), tsvars.end());
        explored.insert(vset);
    }

    for (int i = 0; i < num_vars; i++) {
        if (vset.count(i)) {
            continue;
        }
        if (!transition_systems[i]->is_goal_relevant()) {
            continue;
        }
        vset.insert(i);
        if (explored.count(vset)) {
            vset.erase(i);
            continue;
        }
        vset.erase(i);
        vars.push_back(i);
    }
}

void MergeAndShrinkHeuristicDFS::build_transition_system(const Timer &)
{
    FactoredTransitionSystem fts = create_factored_transition_system(
                                       task_proxy, labels);
    bool erase = false;
    for (TransitionSystem * transition_system : fts.get_vector()) {
        if (!erase) {
            transition_systems.push_back(transition_system);
            if (!transition_system->is_solvable()) {
                erase = true;
            }
        } else {
            transition_system->release_memory();
            delete(transition_system);
        }
    }
    if (erase) {
        return;
    }
    cout << endl;

    vector<TransitionSystem*> res;
    set<set<int> > explored_varsets;
    vector<int> selected_vars;
    get_candidate_vars(selected_vars, explored_varsets);
    merge_strategy->get_next(selected_vars, nullptr);
    if (selected_vars.size() > 0) {
        transition_systems.push_back(nullptr);
    }
    for (size_t i = 0; i < selected_vars.size(); i++) {
        int var_no = selected_vars[i];
        TransitionSystem *system = transition_systems[var_no];
        // TODO prepare_after_merge ??
        if (build_transition_system_dfs(system, explored_varsets, res)) {
            break;
        }
    }
    for (size_t i = 0; i < transition_systems.size() - 1; i++) {
        transition_systems[i]->release_memory();
        transition_systems[i] = nullptr;
    }
    transition_systems.swap(res);
}


bool MergeAndShrinkHeuristicDFS::build_transition_system_dfs(TransitionSystem *system,
        set<set<int> > &explored_varsets, std::vector<TransitionSystem*> &res)
{
    vector<int> selected_vars;
    get_candidate_vars(selected_vars, explored_varsets, system);
    merge_strategy->get_next(selected_vars, system);
    bool unsolvable = false;
    if (selected_vars.empty()) {
        system->release_memory();
        res.push_back(system);
    } else {
        pair<int, int> merge_indices;
        merge_indices.first = transition_systems.size() - 1;
        for (size_t i = 0; !unsolvable && i < selected_vars.size(); i++) {
            int var_no = selected_vars[i];
            TransitionSystem *atomic = transition_systems[var_no];

            //system->statistics(timer);
            //atomic->statistics(timer);

            transition_systems.back() = system;
            if (labels->reduce_before_shrinking()) {
                labels->reduce(merge_indices, transition_systems);
            }

            // Shrinking
            // TODO: shrink any of these?
            pair<bool, bool> shrunk = shrink_strategy->shrink(*system, *atomic);
            if (shrunk.first) {
                //transition_system1->statistics(timer);
            }
            if (shrunk.second) {
                //transition_system2->statistics(timer);
            }

            // TODO: calling label reduction?
            if (labels->reduce_before_merging()) {
                labels->reduce(merge_indices, transition_systems);
            }

            // TODO: prepare_before_merge ???

            // Merging
            TransitionSystem *new_transition_system = new TransitionSystem(
                task_proxy, labels, system, atomic);
            // TODO: prepare_after_merge ?
            //new_transition_system->statistics(timer);
            //fts.get_vector().push_back(new_transition_system);

            /*
              NOTE: both the shrinking strategy classes and the construction of
              the composite require input transition systems to be solvable.
            */
            if (!new_transition_system->is_solvable()) {
                new_transition_system->release_memory();
                res.push_back(new_transition_system);
                return true;
            }

            unsolvable = build_transition_system_dfs(new_transition_system, explored_varsets,
                        res);
            new_transition_system->release_memory();
            if (res.size() == 0 || res.back() != new_transition_system) {
                delete(new_transition_system);
            }
        }
    }
    return unsolvable;
}

void MergeAndShrinkHeuristicDFS::initialize()
{
    Timer timer;
    cout << "Initializing merge-and-shrink heuristic..." << endl;
    starting_peak_memory = get_peak_memory_in_kb();
    verify_no_axioms(task_proxy);
    dump_options();
    warn_on_unusual_options();
    cout << endl;

    build_transition_system(timer);
    report_peak_memory_delta(true);
    cout << "Done initializing merge-and-shrink heuristic [" << timer << "]"
         << endl;
    cout << endl;
}

float MergeAndShrinkHeuristicDFS::compute_heuristic(const GlobalState
        &global_state)
{
    State state = convert_global_state(global_state);
    float cost = 0;
    float tmp;
    for (size_t i = 0; i < transition_systems.size(); i++) {
        tmp = transition_systems[i]->get_cost(state);
        if (tmp == -1) {
            return DEAD_END;
        }
        if (tmp > cost) {
            cost = tmp;
        }
    }
    return cost;
}

static Heuristic *_parse(OptionParser &parser)
{
    parser.document_synopsis(
        "Merge-and-shrink heuristic",
        "This heuristic implements the algorithm described in the following "
        "paper:\n\n"
        " * Silvan Sievers, Martin Wehrle, and Malte Helmert.<<BR>>\n"
        " [Generalized Label Reduction for Merge-and-Shrink Heuristics "
        "http://ai.cs.unibas.ch/papers/sievers-et-al-aaai2014.pdf].<<BR>>\n "
        "In //Proceedings of the 28th AAAI Conference on Artificial "
        "Intelligence (AAAI 2014)//, pp. 2358-2366. AAAI Press 2014.\n"
        "For a more exhaustive description of merge-and-shrink, see the journal "
        "paper\n\n"
        " * Malte Helmert, Patrik Haslum, Joerg Hoffmann, and Raz Nissim.<<BR>>\n"
        " [Merge-and-Shrink Abstraction: A Method for Generating Lower Bounds "
        "in Factored State Spaces "
        "http://ai.cs.unibas.ch/papers/helmert-et-al-jacm2014.pdf].<<BR>>\n "
        "//Journal of the ACM 61 (3)//, pp. 16:1-63. 2014\n"
        "Please note that the journal paper describes the \"old\" theory of "
        "label reduction, which has been superseded by the above conference "
        "paper and is no longer implemented in Fast Downward.");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects",
                                     "supported (but see note)");
    parser.document_language_support("axioms", "not supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");
    parser.document_note(
        "Note",
        "Conditional effects are supported directly. Note, however, that "
        "for tasks that are not factored (in the sense of the JACM 2014 "
        "merge-and-shrink paper), the atomic transition systems on which "
        "merge-and-shrink heuristics are based are nondeterministic, "
        "which can lead to poor heuristics even when only perfect shrinking "
        "is performed.");
    parser.document_note(
        "Note",
        "A currently recommended good configuration uses bisimulation "
        "based shrinking (selecting max states from 50000 to 200000 is "
        "reasonable), DFP merging, and the appropriate label "
        "reduction setting:\n"
        "merge_and_shrink(shrink_strategy=shrink_bisimulation(max_states=100000,"
        "threshold=1,greedy=false),merge_strategy=merge_dfp(),"
        "label_reduction=label_reduction(before_shrinking=true, before_merging=false))");

    // Merge strategy option.
    parser.add_option<shared_ptr<MergeStrategy> >(
        "merge_strategy",
        "See detailed documentation for merge strategies. "
        "We currently recommend merge_dfp.");

    // Shrink strategy option.
    parser.add_option<shared_ptr<ShrinkStrategy> >(
        "shrink_strategy",
        "See detailed documentation for shrink strategies. "
        "We currently recommend shrink_bisimulation.");

    // Label reduction option.
    parser.add_option<shared_ptr<Labels> >(
        "label_reduction",
        "See detailed documentation for labels. There is currently only "
        "one 'option' to use label_reduction. Also note the interaction "
        "with shrink strategies.");

    parser.add_option<int>("max_branching_merge", "",
            std::to_string(numeric_limits<int>::max()));

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();

    if (parser.dry_run()) {
        return nullptr;
    } else {
        return new MergeAndShrinkHeuristicDFS(opts);
    }
}

static Plugin<Heuristic> _plugin("mns_dfs", _parse);
