
#ifdef FRET_H

#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "utilities.h"
#include "option_parser.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <cassert>

#ifndef NDEBUG
#define FRET_DEBUGGING_LEVEL 0
//#define _DEBUG_SCC 1
#define _DEBUG_POLICY_GRAPH 0
#endif


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::FRET(
    const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>
    (dynamic_cast<ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>*>
     (opts.get<SearchEngine *>("engine"))),
    engine(dynamic_cast<ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>*>
           (opts.get<SearchEngine *>("engine"))),
    local_trap_elimination(opts.get<bool>("local")),
    delta(opts.get<float>("delta")),
    epsilon(opts.get<float>("epsilon")),
    vconstruct(this->search_space, delta)
{
    //_t0.stop();
    //_t0.reset();
    //_t1.stop();
    //_t1.reset();
    //_t2.stop();
    //_t2.reset();
    //_t3.stop();
    //_t3.reset();
    //_t4.stop();
    //_t4.reset();
    //_t5.stop();
    //_t5.reset();
}


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::apply_abstraction(
    StateSpace &state_space, const std::set<size_t> &scc, size_t repr)
{
    //std::unordered_map<size_t, size_t> &abstr = state_space.get_abstraction();
    //for (std::set<size_t>::iterator it = scc.begin(); it != scc.end(); it++) {
    //    if (*it != repr) {
    //        assert(abstr.find(*it) == abstr.end());
    //        abstr[*it] = repr;
    //    }
    //}

    for (std::set<size_t>::iterator it = scc.begin(); it != scc.end(); it++) {
        if (*it != repr) {
            state_space.replace_state(*it, repr);
        }
    }
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::update_successors(
    StateSpace &state_space, Node state, std::unordered_map < unsigned,
    std::unordered_set<std::vector<size_t> > > &new_edges)
{
    //cout << "(update_successors " << state.get_prob_state_id() << " -> " << new_edges.size() << ")" << endl;
    for (std::unordered_map<unsigned, std::unordered_set<std::vector<size_t> > >::iterator
         op = new_edges.begin(); op != new_edges.end(); op++) {
        for (std::unordered_set<std::vector<size_t> >::iterator
             succs = op->second.begin(); succs != op->second.end(); succs++) {
            //cout << "add_successors" << flush;
            state_space.add_successors(state, op->first, *succs);
            //cout << "...done" << endl;
        }
    }
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::handle_scc(
    SegmentedVector<FRETData> &data, Node p_state)
{
    //_t3.resume();

    size_t state_id = p_state.get_prob_state_id();

    if (data[state_id].leaf) {
        std::set<size_t> scc;
        std::deque<size_t>::iterator it = stack.begin();
        cout << "NEW LEAF_SCC = { ";
        while (true) {
            size_t _sid = *it;
            data[_sid].onstack = false;
            assert(data[_sid].leaf);
            //data[*it].leaf = true;
            scc.insert(_sid);
            cout << *it << " ";
            size_gv++;
            size_gv_leaf++;
            //it = stack.erase(it);
            it++;
            if (_sid == state_id) {
                break;
            }
        }
        stack.erase(stack.begin(), it);
        cout << "}" << " -> " << state_id << endl;
        //cout << "|STACK| = " << stack.size() << ", |SCC| = " << scc.size() << flush;
        //cout << ", |STACK'| = " << stack.size() << endl;

        //cout << "[SCC#" << state_id << "] is_leaf" << endl;

        //_t0.resume();
        std::unordered_map<unsigned, std::unordered_set<std::vector<size_t> > >
        new_edges;
        std::set<size_t> united_parents;
        std::unordered_set<size_t> combined_successors;
        bool selfloop = false;
        bool leftout = false;
        float min_old_v = std::numeric_limits<float>::max();
        float max_old_v = std::numeric_limits<float>::min();
        for (set<size_t>::iterator it = scc.begin(); it != scc.end(); it++) {
            Node node = this->search_space.get_prob_state(*it);
            float oldv = this->search_space.get_value(node);
            if (oldv < min_old_v) {
                min_old_v = oldv;
            }
            if (oldv > max_old_v) {
                max_old_v = oldv;
            }
            united_parents.insert(node.get_all_parents().begin(),
                                  node.get_all_parents().end());
            node.get_all_parents().clear();
            typename StateSpace::applicable_op_iterator op =
                this->state_space.applicable_operators(node);
            while (!op.end()) {
                std::vector<size_t> successors;
                typename StateSpace::successor_iterator succ =
                    this->state_space.successors(node, op);
                bool nonscc = false;
                bool tmp = false;
                while (!succ.end()) {
                    if (scc.count(*succ) > 0) {
                        successors.push_back(state_id);
                        tmp = true;
                    } else {
                        combined_successors.insert(*succ);
                        successors.push_back(*succ);
                        nonscc = true;
                    }
                    succ++;
                }
                if (nonscc) {
                    new_edges[*op].insert(successors);
                    selfloop = selfloop || tmp;
                }
                leftout = leftout || !nonscc;
                op++;
            }
            //cout << "clear_successors" << flush;
            this->state_space.clear_successors(node);
            //cout << "....done" << endl;
        }
        std::set<size_t> combined_parents;
        std::set_difference(united_parents.begin(), united_parents.end(),
                            scc.begin(), scc.end(),
                            std::inserter(combined_parents,
                                          combined_parents.end()));
        united_parents.clear();
        for (std::set<size_t>::iterator pa = combined_parents.begin();
             pa != combined_parents.end(); pa++) {
            Node node = this->search_space.get_prob_state(*pa);
            typename StateSpace::applicable_op_iterator
            op = this->state_space.applicable_operators(node);
            while (!op.end()) {
                typename StateSpace::successor_iterator
                succ = this->state_space.successors(node, op);
                while (!succ.end()) {
                    if (scc.count(*succ) > 0) {
                        succ.update(state_id);
                    }
                    succ++;
                }
                op++;
            }
        }
        if (selfloop) {
            combined_parents.insert(state_id);
        }
        p_state.get_all_parents().swap(combined_parents);
        for (std::unordered_set<size_t>::iterator succ = combined_successors.begin();
             succ != combined_successors.end(); succ++) {
            Node node = this->search_space.get_prob_state(*succ);
            std::set<size_t> old_parents;
            node.get_all_parents().swap(old_parents);
            std::set_difference(old_parents.begin(), old_parents.end(),
                                scc.begin(), scc.end(),
                                std::inserter(node.get_all_parents(),
                                              node.get_all_parents().end()));
            node.get_all_parents().insert(state_id);
        }
        combined_successors.clear();
        update_successors(this->state_space, p_state, new_edges);
        new_edges.clear();

        std::pair<float, int> newv;
        float tmp = this->search_space.qval_update(p_state, newv, tie_breaking,
                    epsilon);
        if (newv.second < 0) {
            this->value_initializer.dead_end(p_state);
            p_state.mark_as_dead_end();
        } else {
            while (tmp > epsilon) {
                tmp = this->search_space.qval_update(p_state, newv, tie_breaking, epsilon);
            }
        }
        float old = fabs(min_old_v - this->search_space.get_value(p_state));
        if (old > leaf_diff) {
            leaf_diff = old;
        }
        old = fabs(max_old_v - this->search_space.get_value(p_state));
        if (old > leaf_diff) {
            leaf_diff = old;
        }

#if 0
        for (size_t _sid = 0; _sid < this->search_space.size(); _sid++) {
            if (this->state_space.get_abstraction().find(_sid) !=
                this->state_space.get_abstraction().end()) {
                continue;
            }
            Node s = this->search_space.get_prob_state(_sid);
            typename StateSpace::applicable_op_iterator aop =
                this->state_space.applicable_operators(s);
            for (; !aop.end(); aop++) {
                typename StateSpace::successor_iterator it =
                    this->state_space.successors(s, aop);
                for (; !it.end(); it++) {
                    std::set<size_t>::iterator elem = scc.find(*it);
                    assert(elem == scc.end() || (*elem) == state_id);
                }
            }

            for (std::set<size_t>::iterator it = s.get_all_parents().begin();
                 it != s.get_all_parents().end(); it++) {
                assert(scc.count(*it) == 0 || *it == state_id);
            }
        }
#endif

        //cout << "(new_policy_op " << state_id << " -> " << newv.second << ")" << endl;

        apply_abstraction(this->state_space, scc, state_id);
        unit_sccs = unit_sccs && (scc.size() == 1 && !leftout);

        //_t0.stop();
    } else {
        std::deque<size_t>::iterator it = stack.begin();
        cout << "NEW (non-leaf) SCC = { ";
        while (true) {
            size_t _state_id = *it;
            data[_state_id].onstack = false;
            data[_state_id].leaf = false;
            size_gv++;
            //it = stack.erase(it);
            cout << _state_id << " ";
            it++;
            if (_state_id == state_id) {
                break;
            }
        }
        stack.erase(stack.begin(), it);
        cout << "}" << endl;
        //cout << "{}" << " -> " << state_id << endl;
        //cout << "|STACK| = " << stack.size() << ", |SCC| = " << scc.size() << flush;
        //cout << ", |STACK'| = " << stack.size() << endl;
    }

    //_t3.stop();
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
size_t FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::proceed(
    SegmentedVector<FRETData> &data, OpenElem &elem,
    bool &changed)
{
    std::cout << "[FRET] proceeding tarjan at "
        << elem.stateid << "... (queue.size = "
        << queue.size() << ")" << std::endl;

    std::cout << "[FRET] have to get new operator? "
        << elem.it.end() << std::endl;

    changed = false;
    Node state = this->search_space.get_prob_state(elem.stateid);
    FRETData &s_data = data[elem.stateid];
    size_t res = -1;
    while (!changed) {
        std::cout << "continue ... (!changed)" << std::endl;
        typename StateSpace::successor_iterator &it = elem.it;
        std::cout << "it.end = " << std::flush;
        std::cout << it.end() << std::endl;
        for (; !changed && !it.end(); it++) {
            std::cout << "successor: " << std::flush;
            std::cout << *it << std::endl;
            FRETData &succ_data = data[*it];
            if (elem.interrupted) {
                std::cout << "has been interrupted before, so check for lowlink" << std::endl;
                elem.interrupted = false;
                if (succ_data.lowlink < s_data.lowlink) {
                    s_data.lowlink = succ_data.lowlink;
                }
            } else if (succ_data.index == FRETData::UNDEFINED) {
                changed = true;
                res = *it;
                break;
            } else if (succ_data.onstack) {
                if (succ_data.index < s_data.lowlink) {
                    s_data.lowlink = succ_data.index;
                }
            }

            if (!succ_data.onstack || !succ_data.leaf) {
                s_data.leaf = false;
            }
            std::cout << "successor succefully handled" << std::endl;
        }

        if (!changed) {
            std::cout << "nothing changed ... need to get next policy operator "
                << std::endl;
            int next_op = elem.next_op();
            std::cout << "next op = " << next_op << std::endl;
            if (next_op < 0) {
                break;
            }
            elem.it = this->state_space.successors(state, next_op);
            std::cout << "got next operator ..." << std::endl;
        }
    }

    std::cout << "[FRET] done exploring " << elem.stateid
        << " (changed = " << changed << ")" << std::endl;

    if (!changed) {
        delete(&elem);

        if (s_data.lowlink == s_data.index) {
            handle_scc(data, state);
        }
    } else {
        elem.interrupted = true;
        queue.push_back(&elem);
    }

    std::cout << "[FRET] return from proceed(" << elem.stateid << ")"
        << " (queue.size = " << queue.size() << ")" << std::endl;

    return res;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
template<typename F>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::eliminate_traps(
    SegmentedVector<FRETData> &data, size_t state_id, F &construct)
{
    std::pair<float, int> newv;
    float oldv;
    while (true) {
        bool changed = true;
        while (changed) {
            changed = false;
            Node state = this->search_space.get_prob_state(state_id);
            data[state_id].index = index;
            data[state_id].lowlink = index++;

            if (state.is_opened()) {
                std::cout << "[FRET] found open state ("
                    << this->search_space.size() << ")" << std::endl;
                assert(construct.requires_update());
                extended_search_space = true;
                bool dead = !this->generate_all_successors(state);
                state.close();
                if (dead) {
                    oldv = this->search_space.get_value(state);
                    this->value_initializer.dead_end(state);
                    state.mark_as_dead_end();
                    oldv = fabs(oldv - this->search_space.get_value(state));
                    if (oldv > leaf_diff) {
                        leaf_diff = oldv;
                    }
                }
                data.resize(this->search_space.size());
                std::cout << "[FRET] Successfully expanded open state ("
                    << this->search_space.size() << ")" << std::endl;
            }

            //OpenElem *elem = new OpenElem(state_id, state.get_policy_operator_index());
            if (!state.is_dead_end() && !state.is_goal() && construct.requires_update()) {
                std::cout << "[FRET] qval update (leaf_diff = "
                    << leaf_diff << ")" << std::endl;
                oldv = this->search_space.get_value(state);
                while (this->search_space.qval_update(state, newv, tie_breaking, epsilon) > epsilon) ;
                oldv = fabs(oldv - this->search_space.get_value(state));
                if (oldv > leaf_diff) {
                    leaf_diff = oldv;
                }
                std::cout << "[FRET] ... updating done (leaf_diff = "
                    << leaf_diff << ")" << std::endl;
            }

            if (state.get_policy_operator_index() < 0) {
                assert(state.is_dead_end() || state.is_goal());
                break;
            }

            //assert(construct.requires_update() || state.is_solved());

            data[state_id].onstack = true;
            stack.push_front(state_id);

            OpenElem *elem = construct(state);

            state_id = proceed(data, *elem, changed);
        }

        while (!changed && !queue.empty()) {
            OpenElem *elem = queue.back();
            queue.pop_back();
            state_id = proceed(data, *elem, changed);
        }

        if (!changed && queue.empty()) {
            break;
        }
    }
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void
FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_options() {
    std::cout << "Running FRET on ";
    if (local_trap_elimination) {
        std::cout << "a greedy policy graph";
    } else {
        std::cout << "all policy graphs";
    }
    std::cout << std::endl
        << "delta = " << delta << std::endl
        << "epsilon = " << epsilon << std::endl;
    std::cout << "Underlying Search Algorithm: " << std::endl;
    this->engine->print_options();
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
SearchStatus
FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::step()
{
    num_iters = 0;
#if _DEBUG_POLICY_GRAPH
    size_t I = 0;
#endif
    unit_sccs = false;
    leaf_diff = epsilon + g_epsilon;
    extended_search_space = false;
    std::ostringstream s1, s2, s3, s4;
    while (!unit_sccs || leaf_diff > epsilon || extended_search_space) {
        num_iters++;
#if FRET_DEBUGGING_LEVEL > 0
        cout << "[FRET] iteration #" << num_iters << endl;
        cout << "[FRET] finding next fix-point" << endl;
#endif
        //_t1.resume();
        this->engine->search_internal(NULL);
        //_t1.stop();
#if FRET_DEBUGGING_LEVEL > 0
        this->engine->report_progress(this->search_space, this->get_initial_state_id(), true);
        cout << "[FRET] found new fix-point" << endl;
#endif

#if _DEBUG_POLICY_GRAPH
        //std::ostringstream sss1;
        //sss1 << "state_space" << (2 * I) << ".dot";
        //ofstream out1s;
        //out1s.open(sss1.str().c_str());
        //search_space.dump(out1s);
        //out1s.close();
        std::ostringstream ss1;
        ss1 << "policy" << (2 * I) << ".dot";
        ofstream out1;
        out1.open(ss1.str().c_str());
        //this->search_space.dump_policy_graph3(this->get_initial_state_id(), out1);
        this->search_space.dump_policy_graph2(this->get_initial_state_id(), out1);
        //this->search_space.dump(this->get_initial_state_id(), out1);
        out1.close();
#endif

        //for (size_t i = 0; i < this->search_space.size(); i++) {
        //    this->search_space[i].set_solved(false);
        //    this->search_space[i].unmark();
        //}

        //if (I % 2 == 0) {
        //    s1.str("");
        //    this->search_space.dump_policy_graph3(s1);
        //} else {
        //    s3.str("");
        //    this->search_space.dump_policy_graph3(s3);
        //}

        //std::ostringstream ss3;
        //ss3 << "complete" << (I++) << ".dot";
        //ofstream out3;
        //out3.open(ss3.str().c_str());
        //this->search_space.dump(out3);
        //out3.close();

        //cout << "[FRET] eliminating traps ..." << endl;

        size_gv = 0;
        size_gv_leaf = 0;
        //_t2.resume();
        unit_sccs = true;
        leaf_diff = 0;
        extended_search_space = false;
        index = 0;
        SegmentedVector<FRETData> data;
        data.resize(this->search_space.size());

#if FRET_DEBUGGING_LEVEL == 0
        std::streambuf *old = cout.rdbuf(0);
        std::streambuf *olderr = cerr.rdbuf(0);
#endif

        if (local_trap_elimination) {
            //eliminate_traps_policy(data, queue,
            //                       engine->get_initial_state_id());
            eliminate_traps(data, engine->get_initial_state_id(), pconstruct);
        } else {
            //eliminate_traps_value(data, search_space, state_space,
            //                      engine->get_initial_state_id());
            eliminate_traps(data, engine->get_initial_state_id(), vconstruct);
        }

#if FRET_DEBUGGING_LEVEL == 0
        cout.rdbuf(old);
        cerr.rdbuf(olderr);
#endif


        //printf("[FRET] diff = %.5f, unit = %d\n", leaf_diff, unit_sccs);

        //if (I % 2 == 0) {
        //    s3.str("");
        //    this->search_space.dump_policy_graph3(s3);
        //} else {
        //    s4.str("");
        //    this->search_space.dump_policy_graph3(s4);
        //}

#if _DEBUG_POLICY_GRAPH
        std::ostringstream ss2;
        ss2 << "policy" << (2 * I + 1) << ".dot";
        ofstream out2;
        out2.open(ss2.str().c_str());
        this->search_space.dump_policy_graph3(this->get_initial_state_id(), out2);
        //this->search_space.dump(this->get_initial_state_id(), out1);
        out2.close();
        //std::ostringstream sss2;
        //sss2 << "state_space" << (2 * I + 1) << ".dot";
        //ofstream out2s;
        //out2s.open(sss2.str().c_str());
        //this->search_space.dump(out2s);
        //out2s.close();

        //std::ostringstream ss4;
        //ss4 << "complete" << (I++) << ".dot";
        //ofstream out4;
        //out4.open(ss4.str().c_str());
        //this->search_space.dump(out4);
        //out4.close();

        //_t2.stop();

        //printf("FRET#%zu: |G_V| = %zu, |Leafs(G_V)| = %zu\n", num_iters, size_gv, size_gv_leaf);

        I++;
#endif

#if FRET_DEBUGGING_LEVEL
        for (size_t i = 0; i < this->search_space.size(); i++) {
            assert(!this->search_space[i].is_marked());
        }
#endif

        this->search_space.increase_solved_step();
    }

    //cout << "Num FRET iterations: " << num_iters << endl;

    //ofstream out;
    //out.open("policy0.dot");
    //out << s1.str();
    //out.close();
    //out.open("policy1.dot");
    //out << s2.str();
    //out.close();
    //out.open("policy2.dot");
    //out << s3.str();
    //out.close();
    //out.open("policy3.dot");
    //out << s4.str();
    //out.close();

    engine->get_initial_state().set_solved();
    engine->get_initial_state().mark();
    return SOLVED;
}


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::extract_policy(std::ostream &)
{
    std::cerr << "FRET policy extraction has not yet been implemented!" << std::endl;
}


template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, typename TieBreaking>
void FRET<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::print_statistics()
{
    printf("Number of FRET-iterations: %zu\n", num_iters);
    //printf("[t0 = %.4f, t1 = %.4f, t2 = %.4f, t3 = %.4f, t4 = %.4f, t5 = %.4f]\n",
    //       _t0(), _t1(), _t2(), _t3(), _t4(), _t5());
}

#endif
