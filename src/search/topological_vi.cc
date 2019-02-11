
#ifdef TOPOLOGICAL_VALUE_ITERATION_H

#include <limits>
#include <set>

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
TopologicalValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::TopologicalValueIteration(
    const Options &opts)
    : ProbSearchEngine<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>(opts),
    epsilon(opts.get<float>("epsilon"))
{
}

#if TVI_ITERATIVE_TARJAN

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
size_t TopologicalValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::proceed(OpenElem *elem, bool &changed)
{
    //std::cout << "proceed{" << std::flush;
    //std::cout << elem->stateid << "}" << std::endl;
    changed = false;

    State state_i = this->search_space.get_prob_state(elem->stateid);

    size_t id = -1;
    size_t ll = state_i.get_largelink();

    //std::cout << "A " << std::endl;
    while (!changed) {
        //std::cout << "a" << std::endl;
        for (; !changed && !elem->it.end(); elem->it++) {
            State succ = this->search_space.get_prob_state(*(elem->it));
            if (elem->stopped) {
                assert(!succ.is_index_undefined());
                elem->stopped = false;
                if (succ.get_lowlink() < state_i.get_lowlink()) {
                    state_i.set_lowlink(succ.get_lowlink());
                }
            }
            else if (succ.is_index_undefined()) {
                id = succ.get_prob_state_id();
                changed = true;
                break;
            } else if (succ.is_onstack()) {
                if (succ.get_index() < state_i.get_lowlink()) {
                    state_i.set_lowlink(succ.get_index());
                }
            }
            if (succ.get_largelink() >= ll) {
                ll = succ.get_largelink() + 1;
            }
        }
        //std::cout << "b" << std::endl;
        if (!changed) {
            //std::cout << ";;;;;;;;;" << std::endl;
            if (elem->aop.end()) {
                //std::cout << "0" << std::endl;
                break;
            } else {
                //std::cout << "1" << std::endl;
                elem->it = this->state_space.successors(state_i, elem->aop);
                //std::cout << "2" << std::endl;
                elem->aop++;
                //std::cout << "3" << std::endl;
            }
        }
        //std::cout << "c" << std::endl;
    }

    state_i.set_largelink(ll);

    if (!changed) {
        delete(elem);

        //std::cout << "Nothin changed" << std::endl;
        if (state_i.get_index() == state_i.get_lowlink()) {
            //std::cout << "found SCC of size " << std::flush;
            std::vector<size_t> *scc = new std::vector<size_t>();
            std::deque<size_t>::iterator it = stack.begin();
            //std::cout << " ... building SCC ... -> " << std::flush;
            while (true) {
                assert(it != stack.end());
                size_t sid = *it;
                it = stack.erase(it);
                State scc_state = this->search_space.get_prob_state(sid);
                scc_state.set_onstack(false);
                scc_state.set_largelink(ll);
                scc->push_back(sid);
                if (sid == state_i.get_prob_state_id()) {
                    break;
                }
            }
            //std::cout << scc->size() << std::endl;
            update_order.push(std::make_pair(state_i.get_largelink(), scc));
        }
    } else {
        elem->stopped = true;
        open.push_back(elem);
    }

    return id;
}

template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void TopologicalValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::solve(
    size_t id)
{
    while (true) {
        bool changed = true;
        while (changed) {
            //std::cout << "id: " << id << std::endl;
            changed = false;

            State state_i = this->search_space.get_prob_state(id);

            assert(state_i.is_index_undefined());

            state_i.set_index(index);
            state_i.set_lowlink(index++);
            state_i.set_largelink(0);

            if (state_i.is_opened()) {
                //std::cout << "generating successors " << std::endl;
                expand(state_i);
                //std::cout << "-> " << this->search_space.size() << std::endl;
            }

            if (state_i.is_dead_end() || state_i.is_goal()){
                break;
            }

            //std::cout << "gen-elem" << std::flush;
            OpenElem *elem = new OpenElem(this->state_space,
                    this->search_space.get_prob_state(id));
            //std::cout << " ...done" << std::endl;

            state_i.set_onstack(true);
            stack.push_front(id);

            id = proceed(elem, changed);
            //std::cout << changed << " ~~~> " << id << std::endl;
        }

        //std::cout << "> " << open.size() << std::endl;

        while (!changed && !open.empty()) {
            //std::cout << "-" << open.size() << std::endl;
            OpenElem *elem = open.back();
            open.pop_back();
            //std::cout << "PROCEED with " << elem->stateid << std::endl;
            id = proceed(elem, changed);
            //std::cout <<">" <<changed << " ~~~> " << id << std::endl;
        }

        if (!changed && open.empty()) {
            break;
        }
    }
}

#else
template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
void TopologicalValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::solve(
    size_t id)
{
    State state_i = this->search_space.get_prob_state(id);
    state_i.set_index(index);
    state_i.set_lowlink(index++);
    state_i.set_largelink(0);

    if (state_i.is_opened()) {
        //this->generate_all_successors(state_i);
        expand(state_i);
    }

    if (state_i.is_dead_end() || state_i.is_goal()){
        return;
    }

    state_i.set_onstack(true);
    stack.push_front(id);

    size_t ll = 0;

    typename StateSpace::applicable_op_iterator op =
        this->state_space.applicable_operators(state_i);
    while (!op.end()) {
        typename StateSpace::successor_iterator it =
            this->state_space.successors(state_i, op);
        while (!it.end()) {
            State succ = this->search_space.get_prob_state(*it);
            if (succ.is_index_undefined()) {
                solve(*it);
                if (succ.get_lowlink() < state_i.get_lowlink()) {
                    state_i.set_lowlink(succ.get_lowlink());
                }
            } else if (succ.is_onstack()) {
                if (succ.get_index() < state_i.get_lowlink()) {
                    state_i.set_lowlink(succ.get_index());
                }
            }
            if (succ.get_largelink() >= ll) {
                ll = succ.get_largelink() + 1;
            }
            it++;
        }
        op++;
    }

    state_i.set_largelink(ll);

    if (state_i.get_index() == state_i.get_lowlink()) {
        std::vector<size_t> *scc = new std::vector<size_t>();
        /*float diff = std::numeric_limits<float>::max();
        float tmp;
        std::pair<float, int> newv;
        while (diff > epsilon) {
            diff = 0;
            std::deque<size_t>::reverse_iterator it = stack.rbegin();
            while (true) {
                State scc_state = this->search_space.get_prob_state(*it);
                tmp = this->search_space.qval_update(scc_state, newv);
                if (tmp > diff) {
                    diff = tmp;
                }
                if (*it == id) {
                    break;
                }
                it++;
            }
        }*/

        std::deque<size_t>::iterator it = stack.begin();
        while (true) {
            size_t sid = *it;
            it = stack.erase(it);
            State scc_state = this->search_space.get_prob_state(sid);
            scc_state.set_onstack(false);
            scc_state.set_largelink(ll);
            scc->push_back(sid);
            if (sid == id) {
                break;
            }
        }

        update_order.push(std::make_pair(state_i.get_largelink(), scc));
    }
}
#endif
template<typename NodeInfo, template<typename> class NODE, template<template<typename, typename> class, typename, template<typename> class> class SSearchSpace, template<typename, typename> class SStateSpace, class ValueInitializer, class ValueReporter, class TieBreaking>
SearchStatus
TopologicalValueIteration<NodeInfo, NODE, SSearchSpace, SStateSpace, ValueInitializer, ValueReporter, TieBreaking>::step()
{
    index = 0;
    solve(this->get_initial_state_id());
    while (!update_order.empty()) {
        std::vector<size_t> *scc = update_order.top().second;
        update_order.pop();

        float diff = std::numeric_limits<float>::max();
        float tmp;
        std::pair<float, int> newv;
        while (diff > epsilon) {
            diff = 0;
            for (size_t i = 0; i < scc->size(); i++) {
                State scc_state = this->search_space.get_prob_state((*scc)[i]);
                tmp = this->search_space.qval_update(scc_state, newv);
                if (tmp > diff) {
                    diff = tmp;
                }
            }
        }

        delete(scc);
    }
    return SOLVED;
}


#endif
