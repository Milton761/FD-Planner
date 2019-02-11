#ifndef ADDITIVE_HEURISTIC_H
#define ADDITIVE_HEURISTIC_H

#include "priority_queue.h"
#include "relaxation_heuristic.h"

#include <cassert>
#include <map>
#include <deque>

class State;

template<typename T>
struct Queue {
private:
    std::map<float, std::deque<T> > q;
    size_t _size;
public:
    Queue() : _size(0) {}
    void push(float v, T element) {
        q[v].push_back(element);
        _size++;
    }
    std::pair<float, T> pop() {
        _size--;
        typename std::map<float, std::deque<T> >::iterator it;
        it = q.begin();
        std::pair<float, T> res = std::make_pair(it->first, it->second.back());
        it->second.pop_back();
        if (it->second.empty()) {
            q.erase(it);
        }
        return  res;
    }
    size_t size() const {
        return _size;
    }
    bool empty() const {
        return size() == 0;
    }
    void clear() {
        _size = 0;
        q.clear();
    }
};

class AdditiveHeuristic : public RelaxationHeuristic
{
    /* Costs larger than MAX_COST_VALUE are clamped to max_value. The
       precise value (100M) is a bit of a hack, since other parts of
       the code don't reliably check against overflow as of this
       writing. With a value of 100M, we want to ensure that even
       weighted A* with a weight of 10 will have f values comfortably
       below the signed 32-bit int upper bound.
     */
    static const int MAX_COST_VALUE = 100000000;

    Queue<Proposition *> queue;
    bool did_write_overflow_warning;

    void setup_exploration_queue();
    void setup_exploration_queue_state(const State &state);
    void relaxed_exploration();
    void mark_preferred_operators(const State &state, Proposition *goal);

    void enqueue_if_necessary(Proposition *prop, float cost, UnaryOperator *op) {
        assert(cost >= 0);
        if (prop->cost == -1 || prop->cost > cost) {
            prop->cost = cost;
            prop->reached_by = op;
            queue.push(cost, prop);
        }
        assert(prop->cost != -1 && prop->cost <= cost);
    }

    void increase_cost(float &cost, float amount) {
        assert(cost >= 0);
        assert(amount >= 0);
        cost += amount;
        if (cost > MAX_COST_VALUE) {
            write_overflow_warning();
            cost = MAX_COST_VALUE;
        }
    }

    void write_overflow_warning();
protected:
    virtual void initialize();
    virtual float compute_heuristic(const GlobalState &global_state);

    // Common part of h^add and h^ff computation.
    float compute_add_and_ff(const State &state);
public:
    AdditiveHeuristic(const Options &options);
    ~AdditiveHeuristic();
};

#endif
