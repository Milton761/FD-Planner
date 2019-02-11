
#ifndef MAS_STATE_SPACE_H
#define MAS_STATE_SPACE_H

//#include "merge_and_shrink/merge_and_shrink_heuristic.h"
#include "merge_and_shrink/merge_strategy.h"
#include "merge_and_shrink/shrink_strategy.h"
#include "merge_and_shrink/heuristic_representation.h"
#include "heuristic.h"
#include "evaluation_result.h"

#include "globals.h"
#include "utilities.h"

#include "task_proxy.h"

#include <vector>
#include <unordered_map>
#include <memory>

class Options;

template<typename State, typename SearchSpace>
class MASStateSpace
{
    const bool is_budget_limited;
    bool is_initialized;
public:
    class successor_iterator
    {
        friend class MASStateSpace;
        SearchSpace *sp;
        int budget;
        std::vector<int> *mas_state_ids;
        const std::unordered_map<size_t, size_t> *abstr_map;
        unsigned p_op;
        unsigned local_ref;
        successor_iterator(SearchSpace *sp, int budget,
                           std::vector<int> &mas_state_ids,
                           const std::unordered_map<size_t, size_t> &abstr_map,
                           unsigned p_op)
            : sp(sp),  budget(budget), mas_state_ids(&mas_state_ids),
              abstr_map(&abstr_map),
              p_op(p_op),
              local_ref(0) {}
    public:
        successor_iterator() : sp(NULL), budget(0), mas_state_ids(NULL),
            abstr_map(NULL), p_op(-1), local_ref(-1) {}
        successor_iterator(const successor_iterator &copy)
            : sp(copy.sp),
              budget(copy.budget),
              mas_state_ids(copy.mas_state_ids),
              abstr_map(copy.abstr_map),
              p_op(copy.p_op),
              local_ref(copy.local_ref) {}
        successor_iterator(const successor_iterator &&copy)
            : sp(copy.sp),
              budget(copy.budget),
              mas_state_ids(copy.mas_state_ids),
              abstr_map(copy.abstr_map),
              p_op(copy.p_op),
              local_ref(copy.local_ref) {}
        successor_iterator &operator=(const successor_iterator && copy) {
            this->sp = copy.sp;
            this->budget = copy.budget;
            this->mas_state_ids = copy.mas_state_ids;
            this->abstr_map = copy.abstr_map;
            this->p_op = copy.p_op;
            this->local_ref = copy.local_ref;
            return *this;
        }
        successor_iterator &operator=(const successor_iterator &copy) {
            this->sp = copy.sp;
            this->budget = copy.budget;
            this->mas_state_ids = copy.mas_state_ids;
            this->abstr_map = copy.abstr_map;
            this->p_op = copy.p_op;
            this->local_ref = copy.local_ref;
            return *this;
        }
        void dump() const {
            std::cout << "mas_successor_iterator("
                << "p_op = " << p_op
                << ", local_ref = " << local_ref
                << ", mas_state_ids = " << (mas_state_ids != NULL)
                << " (" << (mas_state_ids != NULL ? mas_state_ids->size() : 0) << ")"
                << ", budget = " << budget
                << ")" << std::endl;

        }
        inline size_t state() {
            assert(sp != NULL);
            assert(mas_state_ids != NULL && mas_state_ids->size() > local_ref);
            //std::cout << "reconstructing prob state id ... " << std::endl;
            //dump();
            //std::cout << "global state id = " << (*mas_state_ids)[local_ref] <<
            //    ", budget = " <<  budget << std::endl;
            //std::cout << "search space size: " << sp->size() << std::endl;
            size_t x = sp->lookup_id((*mas_state_ids)[local_ref], budget);
            //std::cout << "looked up state: " << x << std::endl;
            //std::cout << "checking abstraction ... " << std::endl;
            std::unordered_map<size_t, size_t>::const_iterator it
                = abstr_map->find(x);
            if (it != abstr_map->end()) {
                //std::cout << "state " << x << " is mapped to " << std::flush;
                x = it->second;
                //std::cout << x << std::endl;
            }
            //std::cout << "result: " << x << std::endl;
            return x;
        }
        inline size_t operator*() {
            return state();
        }
        inline float prob() const {
            return g_prob_operators[p_op][local_ref].first;
        }
        inline void operator++(int) {
            local_ref++;
        }
        inline bool end() const {
            return mas_state_ids == NULL || local_ref >= mas_state_ids->size();
        }
        inline void get_outcome(std::pair<unsigned, unsigned> &outcome) const {
            outcome.first = p_op;
            outcome.second = local_ref;
        }
        inline void update(size_t new_succ) {
            (*mas_state_ids)[local_ref] = sp->prob_to_global(new_succ);
        }
    };
    class applicable_op_iterator
    {
        friend class MASStateSpace;
        int budget;
        const std::vector<std::pair<int, std::vector<int> > > &successors;
        unsigned local_ref;
        applicable_op_iterator(int budget,
                               const std::vector<std::pair<int, std::vector<int> > > &successors)
            : budget(
                budget), successors(successors), local_ref(0) {
            next();
        }
        inline void next() {
            while (local_ref < successors.size()
                   && !budget_is_sufficient(budget,
                                            g_prob_operator_cost[successors[local_ref].first])) {
                local_ref++;
            }
        }
    public:
        inline unsigned get_local_ref() const {
            return local_ref;
        }
        inline unsigned operator*(void) const {
            return successors[local_ref].first;
        }
        inline float cost() const {
            return g_prob_operator_cost[ **this];
        }
        inline void operator++(int) {
            local_ref++;
            next();
        }
        inline bool end() const {
            return local_ref >= successors.size();
        }
        /*void delete_and_decrease() {
        }*/
    };
    typedef successor_iterator generate_successor_iterator;
    typedef applicable_op_iterator generate_applicable_op_iterator;
private:
    SearchSpace *_sp;

    bool hide_mas_output;
    std::shared_ptr<MergeStrategy> merge_strategy;
    std::vector<std::shared_ptr<ShrinkStrategy> > shrink_strategies;
    std::vector<std::vector<std::pair<int, std::vector<int> > > > state_space;
    std::vector<float> hvals;
    std::vector<bool> goals;
    int initial_state;
    int dead_ends_id;
    std::unordered_map<size_t, size_t> abstr_map;
    std::unordered_map<size_t, std::vector<size_t> > reverse_abstr_map;

    TaskProxy task_proxy;
    std::shared_ptr<HeuristicRepresentation> hrepr;
public:
    unsigned get_state_id(const GlobalState &state);
    int get_global_operator(const State &state, unsigned local_ref) {
        return state_space[state.get_global_state_id()][local_ref].first;
    }
    MASStateSpace(const Options &opts);
    void connect(SearchSpace *sp) {
        this->_sp = sp;
    }
    size_t size() const {
        return state_space.size();
    }
    //std::unordered_map<size_t, size_t> &get_abstraction() {
    //    return abstr_map;
    //}
    void replace_state(size_t state1, size_t state2) {
        assert(abstr_map.find(state1) == abstr_map.end());
        abstr_map[state1] = state2;

        std::vector<size_t> &state_mapping = reverse_abstr_map[state2];
        state_mapping.push_back(state1);
        std::unordered_map<size_t, std::vector<size_t> >::iterator it
            = reverse_abstr_map.find(state1);
        if (it != reverse_abstr_map.end()) {
            for (uint i = 0; i < it->second.size(); i++) {
                assert(abstr_map[it->second[i]] == state1);
                abstr_map[it->second[i]] = state2;
            }
            state_mapping.insert(state_mapping.end(), it->second.begin(), it->second.end());
            reverse_abstr_map.erase(it);
        }
    }
    void initialize();
    inline size_t successor(const State &state, int _outcome, int &local_ref) {
        local_ref = -1;
        const std::pair<int, int> &outcome = g_outcome_to_action[_outcome];
        int budget = compute_budget(state.get_budget(),
                                    g_prob_operator_cost[outcome.first]);
        const std::vector<std::pair<int, std::vector<int> > > &successors
            = state_space[state.get_global_state_id()];
        for (size_t i = 0; i < successors.size(); i++) {
            if (successors[i].first == outcome.first) {
                local_ref = i;
                return _sp->lookup_id(successors[i].second[outcome.second], budget);
            }
        }
        return -1;
    }
    inline size_t successor(const State &state, int _outcome) {
        const std::pair<int, int> &outcome = g_outcome_to_action[_outcome];
        int budget = compute_budget(state.get_budget(),
                                    g_prob_operator_cost[outcome.first]);
        const std::vector<std::pair<int, std::vector<int> > > &successors
            = state_space[state.get_global_state_id()];
        for (size_t i = 0; i < successors.size(); i++) {
            if (successors[i].first == outcome.first) {
                return _sp->lookup_id(successors[i].second[outcome.second], budget);
            }
        }
        return -1;
    }
    inline successor_iterator successors(const State &state,
                                         unsigned local_ref) {
        //std::cout << "successors iterator;;; search space contains "
        //    << _sp->size() << " states" << std::endl;
        std::vector<std::pair<int, std::vector<int> > > &arcs =
            state_space[state.get_global_state_id()];
        unsigned p_op = arcs[local_ref].first;
        return successor_iterator(_sp,
                                  compute_budget(state.get_budget(),
                                          g_prob_operator_cost[p_op]),
                                  arcs[local_ref].second,
                                  abstr_map,
                                  p_op);
    }
    inline successor_iterator successors(const State &state,
                                         const applicable_op_iterator &op) {
        return successors(state, op.get_local_ref());
    }
    inline applicable_op_iterator applicable_operators(
        const State &state) {
        return applicable_op_iterator(state.get_budget(),
                                      state_space[state.get_global_state_id()]);
    }
    inline generate_successor_iterator generate_successors(State &state,
            const generate_applicable_op_iterator &op) {
        return successors(state, op);
    }
    inline generate_applicable_op_iterator generate_applicable_operators(
        State &state) {
        return applicable_operators(state);
    }
    inline int get_initial_state() const {
        return initial_state;
    }
    inline bool is_goal_state(const int &state) const {
        return goals[state];
    }
    inline bool is_dead_end(const State &state, float &h) {
        if (state.get_global_state_id() == dead_ends_id) {
            h = EvaluationResult::INFINITE;
        } else {
            h = hvals[state.get_global_state_id()];
        }
        return (state.get_global_state_id() == dead_ends_id) ||
               (is_budget_limited && hvals[state.get_global_state_id()] > state.get_budget());
    }
    inline float compute_goal_estimation(Heuristic &,
                                         const State &state,
                                         bool &is_dead_end) const {
        is_dead_end = state.get_global_state_id() == dead_ends_id;
        return hvals[state.get_global_state_id()];
    }
    inline float compute_goal_estimation(Heuristic &h,
                                         const State &state,
                                         bool &is_dead_end,
                                         std::set<const GlobalOperator *> &)
    const {
        return compute_goal_estimation(h, state, is_dead_end);
    }
    inline unsigned get_prob_operator_id(const State &state,
                                         const unsigned &local_ref) const {
        return state_space[state.get_global_state_id()][local_ref].first;
    }
    inline unsigned get_outcome_id(const State &state, const unsigned &local_ref_op,
                                   const unsigned &local_ref_i) {
        unsigned op = get_prob_operator_id(state, local_ref_op);
        int succ =
            state_space[state.get_global_state_id()][local_ref_op].second[local_ref_i];
        return _sp->lookup_id(succ, compute_budget(state.get_budget(),
                              g_prob_operator_cost[op]));
    }
    void register_heuristic(Heuristic *) {}
    bool get_path_to_goal(const State &, std::vector<int> &) {
        exit_with(EXIT_INPUT_ERROR);
        /*
        int msid = state.get_global_state_id();
        if (msid == dead_ends_id) {
            return false;
        }
        while (!goals[msid]) {
            for (size_t i = 0; i < state_space[msid].size(); i++) {
                const std::pair<int, std::vector<int> > &successors
                    = state_space[msid][i];
                bool fund = false;
                for (size_t j = 0; j < successors.second.size(); j++) {
                    int succ = successors.second[j];
                    if (hvals[succ] + g_prob_operator_cost[successors.first] == hvals[msid]) {
                        fund = true;
                        msid = succ;
                        plan.push_back(g_prob_operators[successors.first][j].second);
                        break;
                    }
                }
                if (fund) {
                    break;
                }
            }
        }*/
        return true;
    }
    void clear_successors(State &state) {
        state_space[state.get_global_state_id()].clear();
    }
    void add_successors(State &state, unsigned p_op,
                        const std::vector<size_t> &succs) {
        std::vector<std::pair<int, std::vector<int> > > &edges =
            state_space[state.get_global_state_id()];
        edges.resize(edges.size() + 1);
        std::pair<int, std::vector<int> > &out = edges.back();
        out.first = p_op;
        out.second.resize(succs.size());
        for (size_t i = 0; i < succs.size(); i++) {
            out.second[i] = _sp->prob_to_global(succs[i]);
        }
    }
    void delete_applicable_operator(const State &, int) {
        exit_with(EXIT_INPUT_ERROR);
    }
};

namespace state_space_mas
{
static void copy_options(const Options &from, Options &to);
static void add_options_to_parser(OptionParser &parser);
}

#include "mas_state_space.cc"

#endif
