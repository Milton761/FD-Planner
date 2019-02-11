
#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include "globals.h"
#include "state_id.h"
#include "state_registry.h"

#include <vector>
#include <set>

#include "task_proxy.h"
#include "global_state.h"
#include "global_operator.h"
#include "successor_generator.h"

#include "segmented_vector.h"

#include "prob_search_node_info.h"

#include "scalar_evaluator.h"
#include "heuristic.h"
#include "deterministic_search_engine.h"

#include <set>
#include <stdlib.h>
#include <algorithm>
#include <unordered_map>

class Options;
class OptionParser;

namespace state_space_lookup
{
static void copy_options(const Options &from, Options &to);
static void add_options_to_parser(OptionParser &parser);
}

template<typename SState, typename SearchSpace>
class StateSpaceLookup
{
public:
    typedef SState State;
private:
    StateRegistry *state_registry;
    struct SuccessorInformation {
        static const size_t SELF = -1;

        bool initialized;
        /* pointer to successor ProbSearchStates */
        std::vector<std::vector<std::size_t> > successors;
        /* which operators were applicable and lead to which successor vector */
        std::vector<int> operators;
        size_t abstraction;
        SuccessorInformation() : initialized(false), abstraction(SELF) {}
    };
    SegmentedVector<SuccessorInformation> cached_successor_information;
    inline SuccessorInformation &get_successor_information(size_t id) {
        //assert(abstraction.find(id) == abstraction.end());
        if (id >= cached_successor_information.size()) {
            cached_successor_information.resize(id + 1);
        }
        return cached_successor_information[id];
    }
    SearchSpace *_sp;
    GlobalState global_state(const int &state) const {
        return state_registry->lookup_state(StateID::create(state));
    }
    const bool is_budget_limited;
    DeterministicSearchEngine *engine;
    std::vector<ScalarEvaluator *> heuristics;
    std::set<Heuristic *> involved_heuristics;
    size_t _stop;
public:
    unsigned get_state_id(const GlobalState &state) {
        return state.get_id().hash();
    }
    int get_global_operator(const State &state, unsigned local_ref) {
        return cached_successor_information[state.get_global_state_id()].operators[local_ref];
    }
    size_t size() const {
        return state_registry == NULL ? 0 : state_registry->size();
    }
    const std::vector<std::vector<size_t> > &get_all_successors(State &pstate) {
        return get_successor_information(pstate.get_prob_state_id()).successors;
    }
    const std::vector<int> &get_applicable_operators(State &pstate)  {
        return get_successor_information(pstate.get_prob_state_id()).operators;
    }
    StateSpaceLookup(const Options &opts);
    ~StateSpaceLookup() {
        delete state_registry;
    }

    /* just lookup */
    class successor_iterator
    {
        friend class StateSpaceLookup;
        std::vector<size_t> *successors;
        unsigned op_ref;
        unsigned local_ref;
        successor_iterator(std::vector<size_t> &successors,
                           const unsigned op_ref)
            : successors(&successors), op_ref(op_ref), local_ref(0) {}
    public:
        successor_iterator() : successors(NULL), op_ref(-1), local_ref(-1) {}
        successor_iterator(const successor_iterator &copy)
            : successors(copy.successors), op_ref(copy.op_ref), local_ref(copy.local_ref) {}
        successor_iterator(const successor_iterator &&copy)
            : successors(copy.successors), op_ref(copy.op_ref), local_ref(copy.local_ref) {}
        inline size_t state() const {
            return (*successors)[local_ref];
        }
        inline float prob() const {
            return g_prob_operators[op_ref][local_ref].first;
        }
        inline void operator++(int) {
            local_ref++;
        }
        inline bool end() const {
            return successors == NULL || local_ref >= successors->size();
        }
        inline void get_outcome(std::pair<unsigned, unsigned> &outcome) const {
            outcome.first = op_ref;
            outcome.second = local_ref;
        }
        inline unsigned operator*() const {
            return state();
        }
        successor_iterator &operator=(const successor_iterator && copy) {
            this->successors = copy.successors;
            this->op_ref = copy.op_ref;
            this->local_ref = copy.local_ref;
            return *this;
        }
        successor_iterator &operator=(const successor_iterator &copy) {
            this->successors = copy.successors;
            this->op_ref = copy.op_ref;
            this->local_ref = copy.local_ref;
            return *this;
        }
        inline void update(size_t new_succ) {
            (*successors)[local_ref] = new_succ;
        }
    };
    class applicable_op_iterator
    {
        friend class StateSpaceLookup;
    private:
        SuccessorInformation &info;
        unsigned local_ref;
        applicable_op_iterator(SuccessorInformation &info)
            : info(info), local_ref(0) {}
    public:
        inline unsigned get_local_ref() const {
            return local_ref;
        }
        inline unsigned operator*(void) const {
            return info.operators[local_ref];
        }
        inline float cost() const {
            return g_prob_operator_cost[ **this];
        }
        inline void operator++(int) {
            local_ref++;
        }
        inline bool end() const {
            return local_ref >= info.operators.size();
        }
        /*void delete_and_decrease() {
            info.successors.erase(info.successors.begin() + local_ref);
            info.operators.erase(info.operators.begin() + local_ref);
            local_ref -= 1;
        }*/
    };

    /* generate before lookup */
    typedef successor_iterator generate_successor_iterator;
    class generate_applicable_op_iterator
    {
        friend class StateSpaceLookup;
    private:
        const State &state;
        std::vector<OperatorProxy> ops;
        unsigned pos;
        SuccessorInformation &info;
        generate_applicable_op_iterator(const State &state,
                                        const GlobalState &gstate,
                                        SuccessorInformation &info)
            : state(state), pos(0), info(info) {
            g_prob_successor_generator->generate_applicable_ops(gstate, ops);
            move_to_next_applicable_op();
        }
        inline void move_to_next_applicable_op() {
            while (pos < ops.size() &&
                   !budget_is_sufficient(state.get_budget(), ops[pos].get_cost())) {
                pos++;
            }
            if (pos < ops.size()) {
                info.operators.push_back(ops[pos].get_id());
                info.successors.push_back(std::vector<size_t>());
            }
        }
    public:
        inline unsigned get_local_ref() const {
            return info.operators.size() - 1;
        }
        inline unsigned operator*(void) const {
            return ops[pos].get_id();
        }
        inline float cost() const {
            return ops[pos].get_cost();
        }
        inline void operator++(int) {
            pos++;
            move_to_next_applicable_op();
        }
        inline bool end() const {
            return pos >= ops.size();
        }
    };

    std::unordered_map<size_t, std::vector<size_t> > reverse_abstraction;
public:
    void replace_state(size_t state1, size_t state2) {
        // replace state with id state1 by the state with id state2
        //abstraction[state1] = state2;
        SuccessorInformation &info1 = get_successor_information(state1);
        assert(info1.abstraction == SuccessorInformation::SELF);
        info1.abstraction = state2;

        std::vector<size_t> &state_mapping = reverse_abstraction[state2];
        state_mapping.push_back(state1);
        std::unordered_map<size_t, std::vector<size_t> >::iterator it
            = reverse_abstraction.find(state1);
        if (it != reverse_abstraction.end()) {
            for (uint i = 0; i < it->second.size(); i++) {
                SuccessorInformation &info = get_successor_information(it->second[i]);
                assert(info.abstraction == state1);
                info.abstraction = state2;
            }
            state_mapping.insert(state_mapping.end(), it->second.begin(), it->second.end());
            reverse_abstraction.erase(it);
        }
    }
    inline size_t successor(const State &state, int _outcome, int &local_ref) {
        local_ref = -1;
        const std::pair<int, int> &outcome = g_outcome_to_action[_outcome];
        const SuccessorInformation &info = get_successor_information(
                                               state.get_prob_state_id());
        for (size_t i = 0; i < info.operators.size(); i++) {
            if (info.operators[i] == outcome.first) {
                local_ref = i;
                return info.successors[i][outcome.second];
            }
        }
        return -1;
    }
    inline size_t successor(const State &state, int _outcome) {
        const std::pair<int, int> &outcome = g_outcome_to_action[_outcome];
        const SuccessorInformation &info = get_successor_information(
                                               state.get_prob_state_id());
        for (size_t i = 0; i < info.operators.size(); i++) {
            if (info.operators[i] == outcome.first) {
                return info.successors[i][outcome.second];
            }
        }
        //applicable_op_iterator op = applicable_operators(state);
        //while (!op.end())  {
        //    if ((*op) == outcome.first) {
        //        successor_iterator it = successors(state, op);
        //        int i = 0;
        //        while (!it.end()) {
        //            if (i == outcome.second) {
        //                return (*it);
        //            }
        //            i++;
        //            it++;
        //        }
        //    }
        //    op++;
        //}
        return -1;
    }
    inline successor_iterator successors(const State &state,
                                         unsigned local_ref) {
        SuccessorInformation &info =
            get_successor_information(state.get_prob_state_id());
        //if (info.operators.size() <= local_ref) {
        //    std::cout << "INVALID ACCESS TO " << state.get_prob_state_id() << ": "
        //       << local_ref << "/" << info.operators.size() << std::endl;
        //    exit(1);
        //}
        assert(info.successors.size() > local_ref);
        assert(info.operators.size() > local_ref);
        return successor_iterator(
                   info.successors[local_ref], info.operators[local_ref]);
    }
    inline successor_iterator successors(const State &state,
                                         const applicable_op_iterator &op) {
        SuccessorInformation &info =
            get_successor_information(state.get_prob_state_id());
        assert(info.successors.size() > op.get_local_ref());
        assert(info.operators.size() > op.get_local_ref());
        return successor_iterator(info.successors[op.get_local_ref()],
                   *op);
    }
    inline applicable_op_iterator applicable_operators(
        const State &state) {
        return applicable_op_iterator(get_successor_information(
                                          state.get_prob_state_id()));
    }
    void clear_successors(State &state) {
        SuccessorInformation &info = get_successor_information(
                                         state.get_prob_state_id());
        info.operators.clear();
        info.successors.clear();
        //cout << "(clear_successors " << state.get_prob_state_id() << ": "
        //    << info.operators.size() << ")" << endl;
    }
    void delete_applicable_operator(const State &state, int local_ref) {
        SuccessorInformation &info = get_successor_information(state.get_prob_state_id());
        info.successors.erase(info.successors.begin() + local_ref);
        info.operators.erase(info.operators.begin() + local_ref);
    }
    void add_successors(State &state, unsigned p_op,
                        const std::vector<size_t> &succs) {
        SuccessorInformation &info = get_successor_information(
                                         state.get_prob_state_id());
        info.operators.push_back(p_op);
        info.successors.push_back(succs);
    }
    generate_successor_iterator generate_successors(State &state,
            const generate_applicable_op_iterator &op) {
        SuccessorInformation &info = get_successor_information(
                                         state.get_prob_state_id());
        std::vector<size_t> &successors = info.successors[op.get_local_ref()];
        std::vector<std::pair<float, int> > &outcomes = g_prob_operators[*op];
        GlobalState gstate = global_state(state.get_global_state_id());
        successors.resize(outcomes.size());
        int new_budget = compute_budget(state.get_budget(), g_prob_operator_cost[*op]);
        for (size_t i = 0; i < outcomes.size(); i++) {
            const GlobalOperator &gop = g_operators[outcomes[i].second];
            GlobalState succ = state_registry->get_successor_state(gstate, gop);
            for (std::set<Heuristic *>::iterator h = involved_heuristics.begin();
                 h != involved_heuristics.end(); h++) {
                (*h)->reach_state(gstate, gop, succ);
            }
            successors[i] = _sp->lookup_id(succ.get_id().get_value(),
                                           new_budget);
            const SuccessorInformation &succinfo = get_successor_information(successors[i]);
            if (succinfo.abstraction != SuccessorInformation::SELF) {
                successors[i] = succinfo.abstraction;
            }
            //std::unordered_map<size_t, size_t>::iterator
            //it = abstraction.find(successors[i]);
            //while (it != abstraction.end()) {
            //    successors[i] = it->second;
            //    it = abstraction.find(successors[i]);
            //}
        }
        return generate_successor_iterator(successors, *op);
    }
    inline generate_applicable_op_iterator generate_applicable_operators(
        State &state) {
        return generate_applicable_op_iterator(state,
                                               global_state(state.get_global_state_id()),
                                               get_successor_information(state.get_prob_state_id()));
    }
    inline unsigned get_prob_operator_id(const State &state,
                                         const unsigned &local_ref) {
        return get_successor_information(
                   state.get_prob_state_id()).operators[local_ref];
    }
    inline unsigned get_outcome_id(const State &state, const unsigned &local_ref_op,
                                   const unsigned &local_ref_i) {
        return get_successor_information(
                   state.get_prob_state_id()).successors[local_ref_op][local_ref_i];
    }
    inline int get_initial_state() const {
        return g_initial_state().get_id().get_value();
    }
    inline bool is_goal_state(const int &state) const {
        return test_goal(global_state(state));
    }
    inline void connect(SearchSpace *sp) {
        _sp = sp;
    }
    inline void initialize() {}
    inline bool is_dead_end(const State &state, float &max);
    inline float compute_goal_estimation(ScalarEvaluator &h,
                                         const State &state,
                                         bool &is_dead_end) const;
    inline float compute_goal_estimation(ScalarEvaluator &h,
                                         const State &state,
                                         bool &is_dead_end,
                                         std::set<const GlobalOperator *> &preferred_operators) const;
    void register_heuristic(ScalarEvaluator *h) {
        if (std::find(heuristics.begin(), heuristics.end(), h) == heuristics.end()) {
            heuristics.push_back(h);
        }
    }
    bool get_path_to_goal(const State &state, std::vector<int> &plan) {
        GlobalState gstate = global_state(state.get_global_state_id());
        engine->set_upper_bound(state.get_budget());
        return engine->compute_plan(gstate, plan);
    }
};

#include "state_space.cc"

#endif
