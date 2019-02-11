#ifndef GLOBALS_H
#define GLOBALS_H

#include <iosfwd>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

enum OptimizationCriterion {
    MAXP = 0,
    EXPC = 1
};

class AbstractTask;
class Axiom;
class AxiomEvaluator;
class CausalGraph;
class DomainTransitionGraph;
class GlobalOperator;
class GlobalState;
class SuccessorGenerator;
class IntPacker;
class LegacyCausalGraph;
class RandomNumberGenerator;
class Timer;
class StateRegistry;

bool test_goal(const GlobalState &state);
/*
  Set generates_multiple_plan_files to true if the planner can find more than
  one plan and should number the plans as FILENAME.1, ..., FILENAME.n.
*/
void save_plan(const std::vector<const GlobalOperator *> &plan,
               bool generates_multiple_plan_files = false);
int calculate_plan_cost(const std::vector<const GlobalOperator *> &plan);

void read_everything(std::istream &in);
void dump_everything();

// The following six functions are deprecated. Use task_tools.h instead.
bool is_unit_cost();
bool has_axioms();
void verify_no_axioms();
bool has_conditional_effects();
void verify_no_conditional_effects();
void verify_no_axioms_no_conditional_effects();

void check_magic(std::istream &in, std::string magic);

bool are_mutex(const std::pair<int, int> &a, const std::pair<int, int> &b);

void update_initial_state(const GlobalState &newstate);

extern bool g_use_metric;
extern float g_min_action_cost;
extern float g_max_action_cost;

// TODO: The following five belong into a new Variable class.
extern std::vector<std::string> g_variable_name;
extern std::vector<int> g_variable_domain;
extern std::vector<std::vector<std::string> > g_fact_names;
extern std::vector<int> g_axiom_layers;
extern std::vector<int> g_default_axiom_values;

extern IntPacker *g_state_packer;
// This vector holds the initial values *before* the axioms have been evaluated.
// Use the state registry to obtain the real initial state.
extern std::vector<int> g_initial_state_data;
// TODO The following function returns the initial state that is registered
//      in g_state_registry. This is only a short-term solution. In the
//      medium term, we should get rid of the global registry.
extern const GlobalState &g_initial_state();
extern std::vector<std::pair<int, int> > g_goal;

extern std::vector<GlobalOperator> g_operators;
extern std::vector<GlobalOperator> g_axioms;
extern AxiomEvaluator *g_axiom_evaluator;
extern SuccessorGenerator *g_successor_generator;
extern std::vector<DomainTransitionGraph *> g_transition_graphs;
extern LegacyCausalGraph *g_legacy_causal_graph;
extern Timer g_timer;
extern std::string g_plan_filename;
extern int g_num_previously_generated_plans;
extern bool g_is_part_of_anytime_portfolio;
extern RandomNumberGenerator g_rng;

// Marcel: new fields required for the support of probabilistic actions
// Probabilistic operator consistis of a list of outcomes associated with
// their respective outcome probability.
//template<typename Data>
//bool prob_test_goal(const ProbSearchState<Data> &state);
int compute_budget(int budget, int cost);
bool budget_is_sufficient(float budget, float cost);
extern std::vector<std::vector<std::pair<float, int> > > g_prob_operators;
extern std::vector<int> g_prob_operator_cost;
extern std::vector<std::string> g_prob_operator_name;
extern SuccessorGenerator *g_prob_successor_generator;
extern const float g_epsilon;
extern float g_giveup;
extern int g_budget;
extern bool g_store_policy;
extern bool g_use_upper_and_lower;
extern OptimizationCriterion g_optimization_criterion;
extern std::vector<std::pair<int, int> > g_outcome_to_action;
void print_probability(float value, const std::string &name = "I");
extern void dump_prob_state(const GlobalState &state, int budget = -1, bool newline = false, std::ostream &out = std::cout);
//struct prob_outcome_sampler
//{
//private:
//    std::vector<std::pair<unsigned, float> > intervals;
//    struct comp
//    {
//        bool operator()(const std::pair<unsigned, float> &p1,
//                const std::pair<unsigned, float> &p2) const {
//            return p1.second < p2.second;
//        }
//    };
//public:
//    prob_outcome_sampler(const std::vector<std::pair<float, int> >& outcomes);
//    unsigned operator()(float p);
//};
//extern std::vector<prob_outcome_sampler> g_prob_outcome_sampler;

// Only one global object for now. Could later be changed to use one instance
// for each problem in this case the method GlobalState::get_id would also have to be
// changed.
extern StateRegistry *g_state_registry;

extern const std::shared_ptr<AbstractTask> g_root_task();
extern const std::shared_ptr<AbstractTask> g_prob_task();

#endif
