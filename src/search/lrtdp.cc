
#ifdef LRTDP_H

#include "lrtdp.h"

#include "option_parser.h"
#include "plugin.h"

#include "prob_search_space.h"
#include "globals.h"
#include "successor_generator.h"
#include "global_operator.h"
#include "task_proxy.h"

#include "timer.h"

#include <list>
#include <deque>

#include <fstream>
#include <sstream>

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::LRTDP(
    const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>(opts),
    //lazy(opts.get<bool>("lazy")),
    sample(*opts.get<Sampler *>("sampler"))
{
    epsilon = opts.get<float>("epsilon");
    num_trials = 0;
    num_open_states = 0;
    _t.stop();
    _t.reset();
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
inline bool
LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::expand_state(NODE<NodeInfo> &state)
{
    state.close();
    if (!this->generate_all_successors(state, callback)) {
        state.mark_as_dead_end();
        this->value_initializer.dead_end(state);
        return false;
    }
    return true;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
inline float LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::adaptive_qval(
    Node &state,
    std::pair<float, int> &newv)
{
    float oldv = val(state);
    newv.second = -1;
    if (state.is_opened()) {
        if (true) { // lazy
            expand_state(state);
        } else {
            if (!expand_state(state)) {
                state.mark_as_dead_end();
                this->value_initializer.dead_end(state);
            } else {
                this->search_space.qval(state, newv, tie_breaking, epsilon);
            }
            return epsilon + g_epsilon;
        }
    }
    this->search_space.qval(state, newv, tie_breaking, epsilon);
    if (newv.second < 0) {
        //std::cout << num_trials << ": "
        //    << "found dead end " << state.get_prob_state_id() << std::endl;
        state.mark_as_dead_end();
        this->value_initializer.dead_end(state);
    }
    //printf("%.12f vs %.12f => %.12f\n",
    //        oldv,
    //        newv.first,
    //        fabs(oldv - newv.first));
    return fabs(oldv - newv.first);
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
inline float LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::adaptive_qval_update(
    Node &state,
    std::pair<float, int> &newv)
{
    float vold = val(state);
    newv.second = -1;
    if (state.is_opened()) {
        expand_state(state);
    }
    this->search_space.qval_update(state, newv, tie_breaking, epsilon);
    if (newv.second < 0) {
        state.mark_as_dead_end();
        this->value_initializer.dead_end(state);
    }
    return fabs(vold - newv.first);
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
bool LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::check_and_solve(
    size_t id)
{
    Node s = this->search_space.get_prob_state(id);
    bool rv = true;
    deque<size_t> open;
    deque<size_t> closed;
    if (!s.is_solved()) {
        open.push_back(id);
        s.mark();
    }
    pair<float, int> newv;
    float diff;
    while (!open.empty()) {
        Node state = this->search_space.get_prob_state(open.back());
        closed.push_back(open.back());
        open.pop_back();

        if (state.is_goal()) {
            continue;
        }

        if (state.is_dead_end()) {
            continue;
        }

        //diff = adaptive_qval(state, newv);
        diff = adaptive_qval_update(state, newv); // HAVE TO DO THIS WHEN TERMINATING TRIAL AT EPSILON CONSISTENT STATES!!!
        if (diff > epsilon) {
            //if (id == 0) {
            //    printf("%u: diff=%.9f > %.9f\n", num_trials, diff, epsilon);
            //}
            rv = false;
            continue;
        }

        if (newv.second < 0) {
            continue;
        }

        typename StateSpace::successor_iterator it = this->state_space.successors(state,
                newv.second);
        while (!it.end()) {
            Node succ = this->search_space.get_prob_state(*it);
            if (!succ.is_solved() && !succ.is_marked()) {
                open.push_back(succ.get_prob_state_id());
                succ.mark();
            }
            it++;
        }
    }

    if (rv) {
        while (!closed.empty()) {
            Node state = this->search_space.get_prob_state(closed.back());
            state.set_solved();
            state.unmark();
            closed.pop_back();
        }
    } else {
        //if (id == 0) {
        //    std::cout << num_trials << ": initial state cannot be labeled solved ..." << std::endl;
        //}
        while (!closed.empty()) {
            Node state = this->search_space.get_prob_state(closed.back());
            if (!state.is_goal() && !state.is_dead_end()) {
                adaptive_qval_update(state, newv);
            }
            state.unmark();
            closed.pop_back();
        }
    }

    return rv;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
SearchStatus
LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::step()
{

    //ostringstream ss;
    //ss << "policy" << num_trials << ".dot";
    //ofstream out;
    //out.open(ss.str().c_str());
    //this->search_space.dump_policy_graph2(out);
    //out.close();

    //cout << "STEP-Start" << endl;
    if (this->get_initial_state().is_solved()) {
        //ofstream out;
        //out.open("pol.dot");
        //this->search_space.dump_policy_graph2(out);
        //out.close();
        //std::cout << "initial state is labeled solved" << std::endl;
        return SearchStatus::SOLVED;
    }

    num_trials++;

    //cout << "Tial#" << num_trials << ": |S| = " << this->search_space.size() << endl;
    //    << "Bellman: " << num_updates << endl;
    //cout << "Sampling-Start" << endl;

    int id = this->get_initial_state_id();
    list<size_t> visited;
    pair<float, int> newv;
    int length = 0;
    while (true) {
        Node state = this->search_space.get_prob_state(id);

        length++;
        //cout << length << " ~~> " << id << endl;

        if (state.is_solved()) {
            //cout << "\tis solved" << endl;
            break;
        }
        visited.push_back(id);

        if (state.is_goal()) {//}|| prob_test_goal(state)) {
            //cout << "\tis goal" << endl;
            break;
        }

        if (state.is_opened()) {
            num_open_states++;
#if 1
            expand_state(state);
#else
            if (Deterministic) {
                vector<int> plan;
                _t.resume();
                this->state_space.get_path_to_goal(state, plan);
                _t.stop();
                if (plan.size() > 0) {
                    visited.pop_back();
                    size_t i = 0;
                    while (true) {
                        Node state2 = this->search_space.get_prob_state(id);
                        if (state2.is_opened()) {
                            expand_state(state2);
                        }
                        if (state2.is_solved()) {
                            break;
                        }
                        visited.push_back(id);
                        if (i == plan.size()) {
                            break;
                        }
                        id = this->state_space.successor(state2, plan[i]);
                        i++;
                    }
                    break;
                } else {
                    expand_state(state);
                }
            } else {
                //cout << "expand" << endl;
                expand_state(state);
                //cout << state.is_opened() << state.is_closed() << state.is_dead_end() << endl;
            }
#endif
        }

        if (state.is_dead_end()) {
            //cout << "\tis dead end" << endl;
            break;
        }

        id = sample(this->search_space, this->state_space, state, tie_breaking);
        if (id < 0) {
            break;
        }
    }
    //cout << "terminated at " << id << endl;

    while (!visited.empty()) {
        //cout << "CheckAndSolve--Start" << endl;
        //cout << "visited.back() = " << visited.back() << endl;
        if (!check_and_solve(visited.back())) {
            //cout << "CheckAndSolve--End1" << endl;
            break;
        }
        //cout << "CheckAndSolve--End2" << endl;
        visited.pop_back();
    }

    visited.clear();

    this->report_progress(this->search_space, 0);

    //printf("#%d: %.6f\n", num_trials, this->get_initial_state().get_v_current());


    //ostringstream name;
    //name << "sp" << num_trials << ".dot";
    //ofstream out;
    //out.open(name.str().c_str());
    //this->search_space.dump(out);
    //out.close();

    return SearchStatus::IN_PROGRESS;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class Value, class Sampler, template<typename> class Callback, typename TieBreaking, bool Deterministic>
void
LRTDP<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, Value, Sampler, Callback, TieBreaking, Deterministic>::print_statistics() const
{
    cout << "Number of trials: " << num_trials << endl;
    cout << "Number of sampled open states: " << num_open_states << endl;
    printf("Spent %.4f seconds on computing deterministic plans.\n",
            _t());
}


#endif

