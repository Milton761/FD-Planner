#ifndef DET_SEARCH_ENGINE_H
#define DET_SEARCH_ENGINE_H

#include <vector>

class Heuristic;
class OptionParser;
class Options;

#include "global_operator.h"
#include "operator_cost.h"
#include "search_engine.h"
#include "search_progress.h"
#include "search_space.h"
#include "search_statistics.h"
#include "scalar_evaluator.h"

class DeterministicSearchEngine : public SearchEngine, public ScalarEvaluator
{
public:
    typedef std::vector<const GlobalOperator *> Plan;
private:
    Plan plan;
    OperatorCost cost_type;
    bool replanning;
protected:
    SearchSpace *search_space;
    SearchProgress *search_progress;
    SearchStatistics *statistics;
    int bound;
    float upper_bound;

    virtual void reset();
    virtual void initialize() {}
    virtual SearchStatus step() = 0;

    void set_plan(const Plan &plan);
    bool check_goal_and_set_plan(const GlobalState &state);
public:
    DeterministicSearchEngine(const Options &opts);
    virtual ~DeterministicSearchEngine();
    virtual void save_plan_if_necessary() const;
    const Plan &get_plan() const;
    const SearchStatistics &get_statistics() const {
        return *statistics;
    }
    virtual void set_bound(int b) {
        bound = b;
    }
    virtual int get_bound() {
        return bound;
    }
    virtual void set_upper_bound(float b) { upper_bound = b;}
    virtual float get_upper_bound() { return upper_bound; }
    virtual bool dead_ends_are_reliable() const
    {
        return true;
    }
    virtual void get_involved_heuristics(std::set<Heuristic*> &) {}
    virtual EvaluationResult compute_result(EvaluationContext &eval_context);
    bool compute_plan(const GlobalState &state, std::vector<int> &plan);
    virtual void search();
    static void add_options_to_parser(OptionParser &parser);
};

/*
  Print heuristic values of all heuristics evaluated in the evaluation context.
*/
void print_initial_h_values(const EvaluationContext &eval_context);

#endif
