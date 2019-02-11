#ifdef PROB_SEARCH_SPACE_H

#include "prob_search_space.h"
#include "globals.h"
#include "state_registry.h"

#include <memory>
#include <deque>
#include <sstream>

#include <set>
#include <list>

using namespace std;

template<typename NodeInfo>
BasicState<NodeInfo>::BasicState(size_t prob_state_id_, NodeInfo &info_, unsigned currentstep)
    : prob_state_id(prob_state_id_), info(info_), currentstep(currentstep)
{
}

template<typename NodeInfo>
int BasicState<NodeInfo>::get_global_state_id() const
{
    return info.stateid;
}

//template<typename NodeInfo>
//GlobalState BasicState<NodeInfo>::get_global_state() const
//{
//    return g_state_registry->lookup_state(info.stateid);
//}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_budget(int budget)
{
    info.budget = budget;
}

template<typename NodeInfo>
int BasicState<NodeInfo>::get_budget() const
{
    return info.budget;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_v_current(float v, int op)
{
    info.v_current = v;
    info.policy = op;
}

template<typename NodeInfo>
float BasicState<NodeInfo>::get_v_current() const
{
    return info.v_current;
}

//template<typename NodeInfo>
//int BasicState<NodeInfo>::get_policy_operator() const
//{
//    //return info.policy < 0 ? -1 : get_applicable_operators()[info.policy];
//    return info.policy;
//}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_policy_operator_index(int op)
{
    info.policy = op;
}

template<typename NodeInfo>
int BasicState<NodeInfo>::get_policy_operator_index() const
{
    return info.policy;
}

//template<typename NodeInfo>
//void BasicState<NodeInfo>::add_applicable_operator(int op)
//{
//    info.operators.push_back(op);
//}
//
//template<typename NodeInfo>
//std::vector<std::vector<size_t> >
//&BasicState<NodeInfo>::get_all_successors()
//{
//    return info.successors;
//}
//
//template<typename NodeInfo>
//const std::vector<std::vector<size_t> >
//&BasicState<NodeInfo>::get_all_successors() const
//{
//    return info.successors;
//}

template<typename NodeInfo>
bool BasicState<NodeInfo>::add_parent(size_t parent)
{
    return info.parents.insert(parent).second;
}

template<typename NodeInfo>
std::set<size_t> &BasicState<NodeInfo>::get_all_parents()
{
    return info.parents;
}

template<typename NodeInfo>
const std::set<size_t> &BasicState<NodeInfo>::get_all_parents() const
{
    return info.parents;
}

//template<typename NodeInfo>
//const std::vector<int> &BasicState<NodeInfo>::get_applicable_operators()
//const
//{
//    return info.operators;
//}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_new() const
{
    return info.status == NodeInfo::NEW;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_opened() const
{
    return info.status == NodeInfo::OPEN;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_closed() const
{
    return info.status == NodeInfo::CLOSED;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_dead_end() const
{
    return info.status == NodeInfo::DEAD_END;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::mark_as_dead_end()
{
    info.status = NodeInfo::DEAD_END;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_open()
{
    info.status = NodeInfo::OPEN;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::close()
{
    info.status = NodeInfo::CLOSED;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_solved()
{
    info.solved = currentstep;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_solved() const
{
    return info.solved >= currentstep;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::mark()
{
    info.marked = 1;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::unmark()
{
    info.marked = 0;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_marked() const
{
    return info.marked == 1;
}

template<typename NodeInfo>
void BasicState<NodeInfo>::set_goal()
{
    info.status = NodeInfo::GOAL;
}

template<typename NodeInfo>
bool BasicState<NodeInfo>::is_goal() const
{
    return info.status == NodeInfo::GOAL;
}

template<typename NodeInfo>
std::string BasicState<NodeInfo>::status() const
{
    std::ostringstream oss;
    oss << is_new() << is_opened() << is_closed() <<  is_dead_end() << is_goal();
    return oss.str();
}

template<typename NodeInfo>
void BasicState<NodeInfo>::dump_status(std::ostream &out) const
{
    out << is_new() << is_opened() << is_closed() <<  is_dead_end() << is_goal() << std::endl;
}










/* PROB SEARCH SPACE */

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::ProbSearchSpace(
    StateSpace<Node<NodeData>, StateIDLookup> &state_space)
    : current_id(0),
      _lookup(*this),
      _qval(*this),
      state_space(state_space),
      num_updates(0),
      num_updates_initial_state(0),
      solved_step(1)
{
    state_space.connect(&_lookup);
    ASD = 0;
    DSA = 0;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::~ProbSearchSpace()
{
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline Node<NodeData>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::get_prob_state(
    const GlobalState &state, int budget)
{
    unsigned state_id = state_space.get_state_id(state);
    return get_prob_state(lookup_id(state_id, budget));
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline Node<NodeData>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::get_prob_state(
    const unsigned &state, int budget)
{
    return get_prob_state(lookup_id(state, budget));
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline size_t ProbSearchSpace<QVal, StateSpace, NodeData, Node>::lookup_id(
    const unsigned &state, int budget)
{
    //std::cout << "looking up state " << state << " with budget " << budget << "..." << std::endl;
    if (state >= lookup.size()) {
        //std::cout << "haven't seen this state before ..." << std::endl;
        lookup.resize(state + 1);
    }
    //std::cout << "checking budget ..." << std::endl;
    unordered_map<int, size_t> &_lookup = lookup[state];
    unordered_map<int, size_t>::iterator it = _lookup.find(budget);
    size_t sid;
    if (it == _lookup.end()) {
        //std::cout << "budget not found" << std::endl;
        _lookup[budget] = current_id;
        sid = current_id++;
        // TODO: currently budget gets stored twice
        // replace map by set ?
        data.push_back(NodeData(state, budget));
    } else {
        sid = it->second;
    }
    //std::cout << " ----> " << sid << std::endl;
    return sid;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline Node<NodeData>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::get_prob_state(
    size_t state_id)
{
    return Node<NodeData>(state_id, data[state_id], solved_step);
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline Node<NodeData>
ProbSearchSpace<QVal, StateSpace, NodeData, Node>::operator[](
    size_t state_id)
{
    return get_prob_state(state_id);
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline size_t ProbSearchSpace<QVal, StateSpace, NodeData, Node>::size() const
{
    return current_id;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline int ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval(
    const Node<NodeData>
    &state, std::pair<float, int> &newv)
{
    return (_qval)(state, newv);
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline float ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval_update(
    Node<NodeData>
    &state, std::pair<float, int> &newv)
{
    num_updates++;
    if (state.get_prob_state_id() == 0) {
        num_updates_initial_state++;
    }
    return _qval.update(state, newv);
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval_lookup(
    const Node<NodeData> &state, std::vector<int> &ops, float delta)
{
    _qval.val(state, ops, delta);
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
template<typename TieBreaking>
inline void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval(
    const Node<NodeData> &state,
    std::pair<float, int> &newv,
    TieBreaking &tie_breaking,
    float delta)
{
    std::vector<int> candidates;
    _qval(state, newv, candidates, delta);
    if (candidates.size() > 1) {
        newv.second = tie_breaking(state, candidates, *this, this->get_underlying_state_space());
    }
    candidates.clear();
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
template<typename TieBreaking>
inline float ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval_update(
    Node<NodeData> &state,
    std::pair<float, int> &newv,
    TieBreaking &tie_breaking,
    float delta)
{
    std::vector<int> candidates;
    num_updates++;
    if (state.get_prob_state_id() == 0) {
        num_updates_initial_state++;
    }
    float res = _qval.update(state, newv, candidates, delta);
    if (candidates.size() > 1) {
        DSA++;
        int op = tie_breaking(state, candidates, *this,
                                   this->get_underlying_state_space());
        if (state.get_policy_operator_index() != op) {
            ASD++;
        }
        state.set_policy_operator_index(op);
        newv.second = op;
    }
    candidates.clear();
    return res;
}

/*
template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
template<typename TieBreaking>
inline float ProbSearchSpace<QVal, StateSpace, NodeData, Node>::qval_update_prune(
    Node<NodeData> &state,
    std::pair<float, int> &newv,
    TieBreaking &tie_breaking,
    float delta)
{
    std::vector<int> candidates;
    num_updates++;
    if (state.get_prob_state_id() == 0) {
        num_updates_initial_state++;
    }
    float res = _qval.update_prune(state, newv, candidates, delta);
    if (candidates.size() > 1) {
        DSA++;
        int op = tie_breaking(state, candidates, *this,
                                   this->get_underlying_state_space());
        if (state.get_policy_operator_index() != op) {
            ASD++;
        }
        state.set_policy_operator_index(op);
        newv.second = op;
    }
    candidates.clear();
    return res;
}
*/

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
inline bool ProbSearchSpace<QVal, StateSpace, NodeData, Node>::is_goal(
    const Node<NodeData> &state) const
{
    return (state.get_budget() == -1 || state.get_budget() >= 0) &&
           get_underlying_state_space().is_goal_state(state.get_global_state_id());
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump(
    std::ostream &out)
{
    std::stringstream graph;

    for (size_t it = 0; it < size(); it++) {
        Node<NodeData> state = get_prob_state(it);
        typename StateSpace<Node<NodeData>, StateIDLookup>::applicable_op_iterator op =
            get_underlying_state_space().applicable_operators(state);
        while (!op.end()) {
            typename StateSpace<Node<NodeData>, StateIDLookup>::successor_iterator succ =
                get_underlying_state_space().successors(state, op);
            while (!succ.end()) {
                graph << "s" << (it) << " -> " << "s" << (*succ)
                      << " [label=\"" << (*op) <<  ": " << succ.prob() << "\"]"
                      <<  std::endl;
                succ++;
            }
            op++;
        }
    }
    out << "digraph {" << endl;
    for (size_t it = 0; it < size(); it++) {
        Node<NodeData> state = get_prob_state(it);
        out << "s" << (it) << " [label=\"" << (it) << ": "
            << state.get_v_current() << "\"";
        if (state.is_dead_end()) {
            out << ", peripheries=2";
        } else if (state.is_goal()) {
            out << ", shape=rectangle, peripheries=2";
        } else if (state.is_opened()) {
            out << ", shape=hexagon";
        } else if (state.is_solved()) {
            out << ", shape=hexagon,peripheries=2";
        }
        out << "]" << std::endl;
    }
    out << graph.str() << "}" << std::endl;
}
template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump_policy_graph(
    size_t sid, std::ostream &out)
{
    std::stringstream graph;
    std::deque<size_t> states;
    std::deque<size_t>::iterator it;
    states.push_back(sid);

    it = states.begin();
    while (it != states.end()) {
        Node<NodeData> state = get_prob_state(*it);
        if (state.is_goal() || state.is_dead_end()) {
            it++;
            continue;
        }
        int op = state.get_policy_operator_index();
        if (op < 0) {
            it++;
            continue;
        }
        typename StateSpace<Node<NodeData>, StateIDLookup>::successor_iterator succ =
            get_underlying_state_space().successors(state, op);
        while (!succ.end()) {
            graph << "s" << (*it) << " -> " << "s" << (*succ) << std::endl;
            if (std::find(states.begin(), states.end(), *succ) == states.end()) {
                states.push_back(*succ);
            }
            succ++;
        }
        it++;
    }

    out << "digraph {" << endl;
    it = states.begin();
    while (it != states.end()) {
        Node<NodeData> state = get_prob_state(*it);
        out << "s" << (*it) << " [label=\"" << (*it) << ": "
            << state.get_v_current() << "\"";
        if (state.is_goal()) {
            out << ", shape=rectangle";
        }
        if (state.is_goal() || state.is_dead_end()) {
            out << ", peripheries=2";
        }
        out << "]" << std::endl;
        it++;
    }
    out << graph.str() << "}" << std::endl;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump_policy_graph2(
    size_t sid, std::ostream &out)
{
    out << "digraph {" << endl;
    std::set<size_t> drawn;
    std::list<size_t> open;
    open.push_back(sid);
    drawn.insert(open.back());
    t_state istat = get_prob_state(sid);
    while (!open.empty()) {
        t_state node = get_prob_state(open.back());
        out << "  s" << open.back() << " [label=\"(" << open.back() << ") v = "
            << node.get_v_current()
            << "; p = "  << node.get_policy_operator_index();
        out << "\"";
        if (node.is_dead_end()) {
            out << ", peripheries=2";
        } else if (node.is_goal()) {
            out << ", shape=rectangle, peripheries=2";
        } else if (node.is_opened()) {
            out << ", shape=hexagon";
        } else if (node.is_solved()) {
            out << ", shape=hexagon,peripheries=2";
        }
        out << "]" << endl;

        open.pop_back();
        if (node.is_goal() || node.is_dead_end()
            || node.get_policy_operator_index() < 0) {
            continue;
        }
        typename t_state_space::successor_iterator it =
            this->state_space.successors(node, node.get_policy_operator_index());
        while (!it.end()) {
            if (drawn.find(*it) == drawn.end()) {
                drawn.insert(*it);
                open.push_back(*it);
            }
            it++;
        }
    }

    for (set<size_t>::iterator s = drawn.begin(); s != drawn.end(); s++) {
        t_state node = get_prob_state(*s);
        if (node.is_goal() || node.is_dead_end()
            || node.get_policy_operator_index() < 0) {
            continue;
        }
        typename t_state_space::successor_iterator it =
            this->state_space.successors(node, node.get_policy_operator_index());
        while (!it.end()) {
            out << "  s" << (*s) << " -> s"
                << (*it) << "[label=\""  << it.prob()
                << "\"]" << endl;
            it++;
        }
    }

    out << "}" << endl;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump_policy_graph3(
    size_t sid, std::ostream &out)
{
    out << "digraph {" << endl;
    std::set<size_t> drawn;
    std::list<size_t> open;
    open.push_back(sid);
    drawn.insert(open.back());
    t_state istat = get_prob_state(sid);
    while (!open.empty()) {
        t_state node = get_prob_state(open.back());
        out << "  s" << open.back() << " [label=\"(" << open.back() << ") v = "
            << node.get_v_current()
            << "; p = "  << node.get_policy_operator_index();
        out << "\"";
        if (node.is_dead_end()) {
            out << ", peripheries=2";
        } else if (node.is_goal()) {
            out << ", shape=rectangle, peripheries=2";
        } else if (node.is_opened()) {
            out << ", shape=hexagon";
        } else if (node.is_solved()) {
            out << ", shape=hexagon,peripheries=2";
        }
        out << "]" << endl;

        open.pop_back();
        vector<int> ops;
        qval_lookup(node, ops, 0.0005);
        if (node.is_goal() || node.is_dead_end() || ops.size() == 0) {
            continue;
        }
        for (size_t i = 0; i < ops.size(); i++) {
            if (ops[i] < 0) continue;
            typename t_state_space::successor_iterator it =
                this->state_space.successors(node, ops[i]);
            while (!it.end()) {
                if (drawn.find(*it) == drawn.end()) {
                    drawn.insert(*it);
                    open.push_back(*it);
                }
                it++;
            }
        }
    }

    for (set<size_t>::iterator s = drawn.begin(); s != drawn.end(); s++) {
        t_state node = get_prob_state(*s);

        vector<int> ops;
        qval_lookup(node, ops, 0.0005);
        if (node.is_goal() || node.is_dead_end() || ops.size() == 0) {
            continue;
        }
        for (size_t i = 0; i < ops.size(); i++) {
            if (ops[i] < 0) continue;
            typename t_state_space::successor_iterator it =
                this->state_space.successors(node, ops[i]);
            while (!it.end()) {
                out << "  s" << (*s) << " -> s"
                    << (*it) << "[label=\"" << (ops[i]) << ": " << it.prob()
                    << "\"]" << endl;
                it++;
            }
        }
    }

    out << "}" << endl;
}

template<template<typename, typename> class QVal, template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump_policy_graph4(
    size_t sid, std::ostream &out)
{
    out << "digraph {" << endl;
    std::set<size_t> drawn;
    std::list<size_t> open;
    open.push_back(sid);
    drawn.insert(open.back());
    t_state istat = get_prob_state(sid);
    while (!open.empty()) {
        t_state node = get_prob_state(open.back());
        out << "  s" << open.back() << " [label=\"(" << open.back() << ") v = "
            << node.get_v_current()
            << "; p = "  << node.get_policy_operator_index();
        out << "\"";
        if (node.is_dead_end()) {
            out << ", peripheries=2";
        } else if (node.is_goal()) {
            out << ", shape=rectangle, peripheries=2";
        } else if (node.is_opened()) {
            out << ", shape=hexagon";
        } else if (node.is_solved()) {
            out << ", shape=hexagon,peripheries=2";
        }
        out << "]" << endl;

        open.pop_back();
        std::pair<float, int> newv;
        qval(node, newv);
        if (node.is_goal() || node.is_dead_end() || newv.second < 0) {
            continue;
        }
        typename t_state_space::successor_iterator it =
            this->state_space.successors(node, newv.second);
        while (!it.end()) {
            if (drawn.find(*it) == drawn.end()) {
                drawn.insert(*it);
                open.push_back(*it);
            }
            it++;
        }
    }

    for (set<size_t>::iterator s = drawn.begin(); s != drawn.end(); s++) {
        t_state node = get_prob_state(*s);
        std::pair<float, int> newv;
        qval(node, newv);
        if (node.is_goal() || node.is_dead_end() || newv.second < 0) {
            continue;
        }
        typename t_state_space::successor_iterator it =
            this->state_space.successors(node, newv.second);
        while (!it.end()) {
            out << "  s" << (*s) << " -> s"
                << (*it) << "[label=\""  << it.prob()
                << "\"]" << endl;
            it++;
        }
    }

    out << "}" << endl;
}

//template<template<typename, typename> class StateSpace, typename NodeData, template<typename> class Node>
//void ProbSearchSpace<QVal, StateSpace, NodeData, Node>::dump(std::ostream &out)
//{
//    for (size_t i = 0; i < g_state_registry->size(); i++) {
//        out << "[state#" << i << "]: ";
//        StateID *id = StateID::create(i);
//        GlobalState state = g_state_registry->lookup_state(*id);
//        for (int var = 0; var < g_variable_domain.size(); var++) {
//            if (var > 0) {
//                out << ", ";
//            }
//            out << g_fact_names[var][state[var]];
//        }
//        out << endl;
//        delete(id);
//    }
//    out << "#############################################################" << endl;
//    out << "digraph {" << endl;
//    for (size_t i = 0; i < data.size(); i++) {
//        Node<NodeData> node = this->get_prob_state(i);
//        out << "  s" << i << " [label=\"(" << i << ", " << node.get_v_current()
//            << ") #" << node.get_global_state_id()
//            << ": " << node.get_budget() << "\"";
//        if (this->is_goal(node)) {
//            out << ", peripheries=2";
//        }
//        out << "]" << endl;
//    }
//
//    for (size_t i = 0; i < data.size(); i++) {
//        Node<NodeData> node = this->get_prob_state(i);
//        const vector<int> &ops = node.get_applicable_operators();
//        for (size_t j = 0; j < ops.size(); j++) {
//            const vector<pair<float, int> > &outcomes = g_prob_operators[ops[j]];
//            const vector<size_t> &successors = node.get_all_successors()[j];
//            for (size_t k = 0; k < successors.size(); k++) {
//                out << "  s" << i << " -> s" << successors[k]
//                    << " [label=\"" << ops[j] << ": " << outcomes[k].first
//                    << "\"]" << endl;
//            }
//        }
//    }
//
//    out << "}" << endl;
//}


#endif
