
#ifndef OPTIMIZATION_CRITERIA_H
#define OPTIMIZATION_CRITERIA_H

#include "globals.h"

#include <vector>
#include <limits>
#include <math.h>

#include <iostream>


struct CurrentVal {
    template<typename State>
    inline float operator()(const State &state) const {
        return state.get_v_current();
    }

    template<typename State>
    inline float update(State &state, std::pair<float, int> &newv) const {
        float old = state.get_v_current();
        state.set_v_current(newv.first, newv.second);
        return fabs(old - newv.first);
    }
};

struct OptimisticVal {
    template<typename State>
    inline float operator()(const State &state) const {
        return state.get_v_optimistic();
    }

    template<typename State>
    inline float update(State &state, std::pair<float, int> &newv) const {
        float old = state.get_v_optimistic();
        state.set_v_optimistic(newv.first, newv.second);
        return fabs(old - newv.first);
    }
};

template<typename SearchSpace, typename State, typename F>
struct _QValShared {
protected:
    F f;
    SearchSpace &search_space;
    void get_all_action_choices(const State &state, const std::pair<float, int> &qval, std::vector<int> &ops, float delta)
    {
        ops.clear();
        if (qval.second < 0) {
            ops.push_back(qval.second);
            return;
        }
        float val;
        typename SearchSpace::t_state_space::applicable_op_iterator op
            = search_space.get_underlying_state_space().applicable_operators(state);
        while (!op.end()) {
            val = compute_value(state, op);
            if (fabs(val - qval.first) <= delta) {
                ops.push_back(op.get_local_ref());
            }
            op++;
        }
    }
public:
    _QValShared(SearchSpace &search_space) : search_space(search_space) {}
    template<typename AOP>
    inline float compute_value(const State &state, AOP op) {
        float v = 0;
        typename SearchSpace::t_state_space::successor_iterator it =
            search_space.get_underlying_state_space().successors(state, op);
        while (!it.end()) {
            v += it.prob() * f(search_space.get_prob_state(it.state()));
            it++;
        }
        return v;
    }
    inline float get_value(const State &state) {
        return this->f(state);
    }
};

template<typename SearchSpace, typename State, typename F>
struct _QValMax : public _QValShared<SearchSpace, State, F> {
    _QValMax(SearchSpace &search_space)
        : _QValShared<SearchSpace, State, F>(search_space) {}
    int operator()(const State &state, std::pair<float, int> &newv)
    {
        newv.first = -1;
        newv.second = -1;
        int result = -1;
        float val;
        typename SearchSpace::t_state_space::applicable_op_iterator op
            = this->search_space.get_underlying_state_space().applicable_operators(state);
        //int opnum = 0;
        while (!op.end()) {
            val = this->compute_value(state, op);
            //if (ACTIMEL) {
            //    std::cout << opnum << ": " << val << " (" << op.get_local_ref() << ")" << std::flush;
            //}
            if (val > newv.first) {
                //if (ACTIMEL) std::cout << " -> currently best option";
                newv.first = val;
                newv.second = op.get_local_ref();
                result = *op;
            }
            //if (ACTIMEL) std::cout << std::endl;
            //opnum++;
            op++;
        }
        return result;
    }
    void operator()(const State &state, std::pair<float, int> &newv, std::vector<int> &ops, float delta) {
        (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
    }
    inline float update(State &state,
                           std::pair<float, int> &newv)
    {
        (*this)(state, newv);
        return this->f.update(state, newv);
    }
    inline int val(const State &state,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        std::pair<float, int> newv;
        int res = (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
        return res;
    }
    inline float update(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
        return this->f.update(state, newv);
    }
    /*
    template<typename V, typename Cmp>
    void val_prune(State &state,
            std::pair<float, int> &newv,
            V v,
            Cmp cmp
            ) {
        newv.first = -1;
        newv.second = -1;
        float val;
        typename SearchSpace::t_state_space::applicable_op_iterator op
            = this->search_space.get_underlying_state_space().applicable_operators(state);
        //int opnum = 0;
        for (; !op.end(); op++) {
            val = this->compute_value(state, op);
            if (cmp(v(state), val)) {
                op.delete_and_decrease();
                continue;
            }
            //if (ACTIMEL) {
            //    std::cout << opnum << ": " << val << " (" << op.get_local_ref() << ")" << std::flush;
            //}
            if (val > newv.first) {
                //if (ACTIMEL) std::cout << " -> currently best option";
                newv.first = val;
                newv.second = op.get_local_ref();
            }
            //if (ACTIMEL) std::cout << std::endl;
            //opnum++;
        }
    }
    template<typename V, typename Cmp>
    inline float update_prune(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
            V v,
            Cmp cmp,
                           float delta = 0
                           )
    {
        this->val_prune(state, newv, v, cmp);
        this->get_all_action_choices(state, newv, ops, delta);
        return this->f.update(state, newv);
    }
    inline float update_prune(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0) {
        return update(state, newv, ops, delta);
    }
    */
    bool compare(const State &state, float val, float epsilon) {
        return val < this->f(state) - epsilon;
    }
};

template<typename SearchSpace, typename State, typename F>
struct _QValMin : public _QValShared<SearchSpace, State, F> {
    _QValMin(SearchSpace &search_space)
        : _QValShared<SearchSpace, State, F>(search_space) {}
    int operator()(const State &state, std::pair<float, int> &newv)
    {
        newv.first = std::numeric_limits<float>::max();
        newv.second = -1;
        int result = -1;
        float val;
        typename SearchSpace::t_state_space::applicable_op_iterator op
            = this->search_space.get_underlying_state_space().applicable_operators(state);
        while (!op.end()) {
            val = this->compute_value(state, op);
            if (val < newv.first) {
                newv.first = val;
                newv.second = op.get_local_ref();
                result = *op;
            }
            op++;
        }
        return result;
    }
    void operator()(const State &state, std::pair<float, int> &newv, std::vector<int> &ops, float delta) {
        (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
    }
    inline float update(State &state,
                           std::pair<float, int> &newv)
    {
        (*this)(state, newv);
        return this->f.update(state, newv);
    }
    inline int val(const State &state,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        std::pair<float, int> newv;
        int res = (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
        return res;
    }
    inline float update(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        (*this)(state, newv);
        this->get_all_action_choices(state, newv, ops, delta);
        return this->f.update(state, newv);
    }
    bool compare(const State &state, float val, float epsilon) {
        return val > this->f(state) + epsilon;
    }
    /*
    template<typename V, typename Cmp>
    void val_prune(State &state,
            std::pair<float, int> &newv,
            V v,
            Cmp cmp
            ) {
        newv.first = std::numeric_limits<float>::max();
        newv.second = -1;
        float val;
        typename SearchSpace::t_state_space::applicable_op_iterator op
            = this->search_space.get_underlying_state_space().applicable_operators(state);
        for (; !op.end(); op++) {
            val = this->compute_value(state, op);
            if (cmp(v(state), val)) {
                op.delete_and_decrease();
                continue;
            }
            if (val < newv.first) {
                newv.first = val;
                newv.second = op.get_local_ref();
            }
        }
    }
    template<typename V, typename Cmp>
    inline float update_prune(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
            V v,
            Cmp cmp,
                           float delta = 0
                           )
    {
        this->val_prune(state, newv, v, cmp);
        this->get_all_action_choices(state, newv, ops, delta);
        return this->f.update(state, newv);
    }
    inline float update_prune(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0) {
        return update(state, newv, ops, delta);
    }
    */
};


template<typename SearchSpace, typename State, template<typename, typename, typename> class QVal, typename F1, typename F2>
struct DualQVal {
private:
    struct Less {
        bool operator()(const float &x, const float &y) const {
            return x < y;
        }
    };
public:
    QVal<SearchSpace, State, F1> q1;
    QVal<SearchSpace, State, F2> q2;
    DualQVal(SearchSpace &search_space) :
        q1(search_space),
        q2(search_space) {}
    int operator()(const State &state, std::pair<float, int> &newv)
    {
        return q2(state, newv);
    }
    void operator()(const State &state, std::pair<float, int> &newv, std::vector<int> &ops, float delta) {
        q2(state, newv, ops, delta);
    }
    inline float update(State &state,
                           std::pair<float, int> &newv)
    {
        q1.update(state, newv);
        return q2.update(state, newv);
    }
    inline int val(const State &state,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        return q2.val(state, ops, delta);
    }
    inline float update(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0)
    {
        q1.update(state, newv);
        return q2.update(state, newv, ops, delta);
    }
    inline float get_value(State &state) {
        return q2.get_value(state);
    }
    template<typename AOP>
    inline float compute_value(const State &state, AOP op) {
        return q2.compute_value(state, op);
    }
    bool compare(const State &state, float val, float epsilon) {
        return q2.compare(state, val, epsilon);
    }
    /*
    inline float update_prune(State &state,
                           std::pair<float, int> &newv,
                           std::vector<int> &ops,
                           float delta = 0) {
        q1.update(state, newv);
        return q2.update_prune(state, newv, ops, F2(), Less(), delta);
    }
    */
};



template<typename SearchSpace, typename State>
struct QValMin : public _QValMin<SearchSpace, State, CurrentVal>
{
    QValMin(SearchSpace &search_space)
        : _QValMin<SearchSpace, State, CurrentVal>(search_space) {}
};

template<typename SearchSpace, typename State>
struct QValMax : public _QValMax<SearchSpace, State, CurrentVal>
{
    QValMax(SearchSpace &search_space)
        : _QValMax<SearchSpace, State, CurrentVal>(search_space) {}
};

template<typename SearchSpace, typename State>
struct DualQValMin : public DualQVal<SearchSpace, State, _QValMin, CurrentVal, OptimisticVal>
{
    DualQValMin(SearchSpace &search_space)
        : DualQVal<SearchSpace, State, _QValMin, CurrentVal, OptimisticVal>(search_space) {}
};

template<typename SearchSpace, typename State>
struct DualQValMax : public DualQVal<SearchSpace, State, _QValMax, CurrentVal, OptimisticVal>
{
    DualQValMax(SearchSpace &search_space)
        : DualQVal<SearchSpace, State, _QValMax, CurrentVal, OptimisticVal>(search_space) {}
};

#endif
